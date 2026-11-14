"""Shared pieces for the reference-tag SAR study: data, poses, visits, holograms, channel self-calibration, evaluation.
Frames: odom (or lidar map) xy; per-visit local frame u = along travel, v = lateral toward boresight (robot left), z up."""
import os, json, numpy as np, pandas as pd, torch
C = 299792458.0; IN = 0.0254; dev = 'cuda' if torch.cuda.is_available() else 'cpu'
# Run selection (defaults = the c1 pass): RFID_READS = reads CSV prefix (no .csv), RFID_BAGNPZ = extract_bag.py output,
# RFID_CLK = host - reader clock offset (s); for a new run without RFID_CLK it is the median over that run's reads.
D = os.environ.get('RFID_READS', '/home/student/rfid_db/datasets/c1_lift030_fwd/c1_lift030_fwd_20261021_124947')
BAGNPZ = os.environ.get('RFID_BAGNPZ', '/home/student/rfid_db/loc/bag_c1.npz')
# Antenna height calibration: RFID_ANTZ='a,b' -> antenna back-face centre z = a + b * lift (tape-measured at two lift readings);
# default a = OFF[2] = 0.5588 (handover tape measure, lift at bottom), b = 1.
_AZ = [float(x) for x in os.environ['RFID_ANTZ'].split(',')] if 'RFID_ANTZ' in os.environ else [0.5588, 1.0]
def ant_z(lift): return _AZ[0] + _AZ[1] * np.asarray(lift)
WH = {1: (22, 26), 2: (22, 40), 3: (22, 25), 4: (22, 26), 5: (28, 35.5), 6: (49, 25), 7: (14.5, 25.5)}
OFF = [-0.2371, 0.2232, 0.5588]          # base_link -> antenna back-face centre (lift added to z)
CLK = -166.6076                           # host_time - reader_us/1e6 (median, constant over the run)

def hop_table():
    h = json.load(open(D + '_reader.json'))['hop_table_mhz']; return np.array(json.loads(h) if isinstance(h, str) else h) * 1e6

def load_reads(category=None):
    df = pd.read_csv(D + '.csv', comment='#')
    if category: df = df[df.category == category].copy()
    df['f'] = hop_table()[df.channel.values - 1]
    clk = float(os.environ['RFID_CLK']) if 'RFID_CLK' in os.environ else CLK if 'RFID_READS' not in os.environ else \
        float(np.median(df.host_time_s - df.reader_us / 1e6))
    df['t'] = df.reader_us / 1e6 + clk     # robot clock, per read
    return df

def load_traj(src='odom'):
    """returns t, x, y, yaw arrays (robot clock) and lift (t, m)"""
    z = np.load(BAGNPZ, allow_pickle=True)
    if src == 'odom': o = z['odom']; tr = (o[:, 0], o[:, 1], o[:, 2], np.unwrap(o[:, 3]))
    else:
        a = np.loadtxt(src); tr = (a[:, 0], a[:, 1], a[:, 2], np.unwrap(a[:, 3]))
    return tr, (z['lift'][:, 0], z['lift'][:, 1])

def pose_reads(df, traj, lift, dt=0.0, face=0.0, fwd=0.0, side=0.0, dz=0.0):
    """antenna point per read; face = extra offset along boresight (phase centre in front of back face)"""
    (tt, x, y, yaw), (lt, lv) = traj, lift
    t = df.t.values + dt
    bx, by, th = np.interp(t, tt, x), np.interp(t, tt, y), np.interp(t, tt, yaw)
    L = np.interp(t, lt, lv); c, s = np.cos(th), np.sin(th)
    ox, oy = OFF[0] + fwd, OFF[1] + face + side
    df = df.copy()
    df['ant_x'] = bx + ox * c - oy * s; df['ant_y'] = by + ox * s + oy * c; df['ant_z'] = ant_z(L) + dz
    df['bore_x'] = -s; df['bore_y'] = c; df['base_yaw'] = th; df['lift_m'] = L; df['base_x'] = bx; df['base_y'] = by
    return df

def visits(df, gap=8.0, min_reads=120, keep_q=0.5, maxn=3000, beam=None):
    out = []
    for name, g in df.groupby('name'):
        g = g.sort_values('t'); tt = g.t.values
        vid = np.concatenate([[0], np.cumsum(np.diff(tt) > gap)])
        for v in np.unique(vid):
            h = g[vid == v]
            if len(h) < min_reads: continue
            h = h[h.rssi_dbm >= h.rssi_dbm.quantile(keep_q)]
            top = h[h.rssi_dbm >= h.rssi_dbm.quantile(0.8)]
            # local frame from the strong-read antenna track
            A = top[['ant_x', 'ant_y']].values; c = A.mean(0)
            bore = np.array([top.bore_x.mean(), top.bore_y.mean()]); bore /= np.linalg.norm(bore)
            u = np.array([bore[1], -bore[0]])                  # along travel: boresight is robot left, so forward = bore rotated -90
            if beam is not None:                               # keep reads whose direction to the coarse tag guess is within beam deg of boresight
                p0 = c + 0.8 * bore; d = p0[None] - h[['ant_x', 'ant_y']].values
                b = h[['bore_x', 'bore_y']].values; cosang = (d * b).sum(1) / np.linalg.norm(d, axis=1)
                h = h[cosang >= np.cos(np.radians(beam))]
                if len(h) < 60: continue
            if len(h) > maxn: h = h.sample(maxn, random_state=0)
            out.append(dict(name=name, shelf=int(name.split()[1][1:]) if name.startswith('Ref') else 0, t=float(h.t.median()), g=h, c=c, u=u, v=bore,
                            za=float(h.ant_z.median()), n=len(h)))
    return out

T = lambda a: torch.tensor(np.asarray(a), device=dev, dtype=torch.float32)
def _score(P, A, k, ph, w=None):
    S = torch.empty(P.shape[0], device=dev)
    for i0 in range(0, P.shape[0], 8192):
        a = ph[None] - k[None] * torch.cdist(P[i0:i0 + 8192], A)
        if w is None: S[i0:i0 + 8192] = torch.cos(a).sum(1) ** 2 + torch.sin(a).sum(1) ** 2
        else: S[i0:i0 + 8192] = (torch.cos(a) * w).sum(1) ** 2 + (torch.sin(a) * w).sum(1) ** 2
    return S

def local_grid(V, ur=(-1.5, 1.5), vr=(0.15, 1.8), zr=(0.2, 2.2), step=0.04, vfix=None):
    us = torch.arange(*ur, step, device=dev); zs = torch.arange(*zr, step, device=dev)
    vs = torch.arange(*vr, step, device=dev) if vfix is None else torch.tensor([vfix], device=dev)
    G = torch.stack(torch.meshgrid(us, vs, zs, indexing='ij'), -1).reshape(-1, 3)
    c, u, v = T(V['c']), T(V['u']), T(V['v'])
    P = torch.stack([c[0] + G[:, 0] * u[0] + G[:, 1] * v[0], c[1] + G[:, 0] * u[1] + G[:, 1] * v[1], G[:, 2]], 1)
    return G, P

def hologram(V, theta, sign=-1.0, **kw):
    g = V['g']; A = T(g[['ant_x', 'ant_y', 'ant_z']].values); k = T(4 * np.pi * g.f.values / C)
    ph = T(sign * g.phase_rad.values) - T(np.asarray(theta)[g.channel.values - 1])
    G, P = local_grid(V, **kw); return G, P, _score(P, A, k, ph)

def locate(V, theta, sign=-1.0, vfix=None, step=0.04, fine=0.01):
    G, P, S = hologram(V, theta, sign, vfix=vfix, step=step); i = int(S.argmax()); g0 = G[i].cpu().numpy()
    kw = dict(ur=(g0[0] - step, g0[0] + step), zr=(g0[2] - step, g0[2] + step), step=fine)
    if vfix is None: kw['vr'] = (g0[1] - step, g0[1] + step)
    else: kw['vfix'] = vfix
    G2, P2, S2 = hologram(V, theta, sign, **kw); j = int(S2.argmax())
    return G2[j].cpu().numpy(), P2[j].cpu().numpy(), float(S2[j] / S.mean()), float(S2[j] / (V['n'] ** 2))

def calib(Vs, pos, sign=-1.0, nch=50):
    """per-channel offsets given tag positions; per-visit constant psi removed"""
    acc = np.zeros(nch, complex)
    for V, p in zip(Vs, pos):
        g = V['g']; d = np.linalg.norm(g[['ant_x', 'ant_y', 'ant_z']].values - p, axis=1)
        r = np.exp(1j * (sign * g.phase_rad.values - 4 * np.pi * g.f.values / C * d)); ch = g.channel.values - 1
        psi = np.angle(r.sum())
        for _ in range(3):
            q = r * np.exp(-1j * psi); th = np.angle(np.bincount(ch, q.real, nch) + 1j * np.bincount(ch, q.imag, nch))
            psi = np.angle((r * np.exp(-1j * th[ch])).sum())
        q = r * np.exp(-1j * psi); acc += np.bincount(ch, q.real, nch) + 1j * np.bincount(ch, q.imag, nch)
    return np.angle(acc), np.abs(acc)

def pairs(rows, key='L'):
    """rows: name, shelf, t, L (local u,v,z), P (world) -> per-pair errors in the local frame of the first tag's visit"""
    R = pd.DataFrame(rows); E = []
    for n, g in R.groupby('shelf'):
        W, H = WH[n]
        for a, b, L, kind in (('TL', 'TR', W, 'h'), ('BL', 'BR', W, 'h'), ('TL', 'BL', H, 'v'), ('TR', 'BR', H, 'v')):
            ga, gb = g[g.name.str.endswith(a)], g[g.name.str.endswith(b)]
            for _, ra in ga.iterrows():
                dt = (gb.t - ra.t).abs()
                if not len(dt) or dt.min() > 25: continue
                rb = gb.loc[dt.idxmin()]
                d3 = np.linalg.norm(ra.P - rb.P); dl = rb.L - ra.L
                E.append(dict(shelf=n, pair=a + b, kind=kind, t=ra.t, true=L * IN, err=d3 - L * IN, du=dl[0], dv=dl[1], dz=dl[2]))
    E = pd.DataFrame(E)
    # component errors: horizontal pairs should be |du|=W, dv=0, dz=0 ; vertical pairs du=0, dv=0, |dz|=H
    E['e_u'] = np.where(E.kind == 'h', E.du.abs() - E.true, E.du)
    E['e_z'] = np.where(E.kind == 'v', E.dz.abs() - E.true, E.dz)
    return E

def summary(E, label=''):
    a = E.err.abs()
    return dict(label=label, n=len(E), med_cm=round(a.median() * 100, 1), p90_cm=round(a.quantile(.9) * 100, 1),
                u_med=round(E.e_u.abs().median() * 100, 1), v_med=round(E.dv.abs().median() * 100, 1), z_med=round(E.e_z.abs().median() * 100, 1))

# ---------- dwell-differential hologram: only phase differences inside one channel dwell (theta- and multipath-constant-free) ----------
def dwell_ids(g, gap=0.25):
    g = g.sort_values('t'); ch = g.channel.values; tt = g.t.values
    new = np.r_[True, (ch[1:] != ch[:-1]) | (np.diff(tt) > gap)]
    gid = np.cumsum(new) - 1; cnt = np.bincount(gid)
    keep = cnt[gid] >= 2
    g = g[keep]; gid = pd.factorize(gid[keep])[0]
    return g, gid

def score_dwell(P, A, k, ph, gid, ng, wts=None):
    S = torch.empty(P.shape[0], device=dev); n = torch.bincount(gid, minlength=ng).float()
    for i0 in range(0, P.shape[0], 4096):
        a = ph[None] - k[None] * torch.cdist(P[i0:i0 + 4096], A)
        re = torch.zeros(a.shape[0], ng, device=dev).index_add_(1, gid, torch.cos(a))
        im = torch.zeros(a.shape[0], ng, device=dev).index_add_(1, gid, torch.sin(a))
        q = (re * re + im * im - n[None]) / (n * (n - 1))[None]       # mean pair cos per dwell, in [-1, 1]
        S[i0:i0 + 4096] = (q * (n - 1)[None]).sum(1) if wts is None else (q * wts[None]).sum(1)
    return S

def prep_dwell(V, sign):
    g, gid = dwell_ids(V['g']); V = dict(V, gd=g)
    V['A'] = T(g[['ant_x', 'ant_y', 'ant_z']].values); V['k'] = T(4 * np.pi * g.f.values / C)
    V['ph'] = T(sign * g.phase_rad.values); V['gid'] = torch.tensor(gid, device=dev); V['ng'] = int(gid.max()) + 1 if len(gid) else 0
    return V

def holo_dwell(V, **kw):
    G, P = local_grid(V, **kw); return G, P, score_dwell(P, V['A'], V['k'], V['ph'], V['gid'], V['ng'])

def locate_dwell(V, step=0.04, fine=0.01, vfix=None, **kw):
    G, P, Sc = holo_dwell(V, step=step, vfix=vfix, **kw); i = int(Sc.argmax()); g0 = G[i].cpu().numpy()
    k2 = dict(ur=(g0[0] - step, g0[0] + step), zr=(g0[2] - step, g0[2] + step), step=fine)
    if vfix is None: k2['vr'] = (g0[1] - step, g0[1] + step)
    else: k2['vfix'] = vfix
    G2, P2, S2 = holo_dwell(V, **k2); j = int(S2.argmax())
    return G2[j].cpu().numpy(), P2[j].cpu().numpy(), float(S2[j] / max(V['ng'], 1))
