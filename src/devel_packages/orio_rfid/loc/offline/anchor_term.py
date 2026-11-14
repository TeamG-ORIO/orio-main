"""Rank 2 of the literature report: per-dwell reference anchoring -> coherent hologram over whole passes, as an extra grid term.
psi(D) = angle of sum over reference tags read in global dwell D (excluding the scored tag) of w * exp(j r), r = -phi - 4 pi f d / c with
d from the reference's known position (map frame, through the single map->room tie). Dwells with reference consensus < 0.5 are dropped.
For each strong pass (within 3 dB of the tag's best, as the phase term), anchored phasors u = exp(j(-phi - psi(D))) are summed coherently
across ALL its dwells: A_pass(P) = |sum w u exp(-j 4 pi f |a - P| / c)| / sum w, in [0, 1]; the term is sum over passes of log(A_pass + 1e-3).
Unlike the dwell-differential term it keeps range (curvature across dwells), so it can constrain depth and height.
Evaluated on the same cells as grids3d_<tag>.pkl (where the phase term was evaluated). GPU.
--em <cv csv> <variant>: EM step of report rank 5: items join the per-dwell offset estimate as weak anchors (weight 0.2) at their
current estimates from that CV file (own tag always excluded) -> anchor_<tag>_em.pkl
Usage: anchor_term.py <pose src> <tag> [--em <cv csv> <variant>]   -> anchor_<tag>.pkl {name: float32 array over that tag's grid}, plus a reference sanity check"""
import sys, pickle, numpy as np, pandas as pd, torch
sys.path.insert(0, '/home/student/rfid_db/loc'); import sarlib as S, eval_v1 as E
L = '/home/student/rfid_db/loc/'; src, tag = sys.argv[1], sys.argv[2]; GATE = 3.0; dev = S.dev
COSW = '--cosw' in sys.argv; SLOPE = '--slope' in sys.argv; DRS = np.arange(-1.5, 1.51, 0.25)   # rank-2 refinements (report)
GR = pickle.load(open(L + f'grids3d_{tag}.pkl', 'rb')); meta = np.load(L + f'dwell_meta_{tag}.npz', allow_pickle=True); R, t = meta['R'], meta['t']
df = S.load_reads(); df = df[df.name.notna()]; traj, lift = S.load_traj(src); df = S.pose_reads(df.sort_values('t').reset_index(drop=True), traj, lift)
new = np.r_[True, (df.channel.values[1:] != df.channel.values[:-1]) | (np.diff(df.t.values) > 0.25)]; df['gid'] = np.cumsum(new) - 1
df['ph'] = -df.phase_rad.values; df['k'] = 4 * np.pi * df.f.values / S.C
# reference residuals per (dwell, ref)
refs = [n for n in df.name.unique() if n.startswith('Ref ') and E.truth(n) is not None]
Pm = {n: np.r_[R.T @ (E.truth(n)[1] - t), E.truth(n)[2]] for n in refs}
EM = sys.argv[sys.argv.index('--em') + 1:sys.argv.index('--em') + 3] if '--em' in sys.argv else None; WI = {}
if EM:                                                     # items as weak anchors at their current estimates (map frame)
    cv = pd.read_csv(EM[0]); cv = cv[(cv.variant == EM[1]) & ~cv.ref]
    for r in cv.itertuples(): Pm[r.name] = np.array([r.x, r.y, r.z]); WI[r.name] = 0.2
    refs = refs + [n for n in WI if n in set(df.name)]
rf = df[df.name.isin(refs)].copy(); tp = np.stack(rf.name.map(Pm).values)
rf['z'] = np.exp(1j * (rf.ph.values - rf.k.values * np.linalg.norm(rf[['ant_x', 'ant_y', 'ant_z']].values - tp, axis=1)))
RZ = rf.groupby(['gid', 'name']).z.sum(); RZ = RZ / np.abs(RZ) * np.minimum(rf.groupby(['gid', 'name']).size(), 10)   # unit phasor x min(reads,10)
if WI: RZ = RZ * pd.Series(RZ.index.get_level_values(1).map(lambda n: WI.get(n, 1.0)), index=RZ.index)
tot = RZ.groupby(level=0).sum(); wabs = RZ.abs().groupby(level=0).sum()
def psi_for(name, gids):
    """offset per dwell from references other than `name`; NaN where consensus < 0.5 or no reference"""
    s = tot.reindex(gids).values.astype(complex); w = wabs.reindex(gids).values.astype(float)
    if name in refs:
        own = RZ.xs(name, level=1).reindex(gids).fillna(0).values; s = s - own; w = w - np.abs(own)
    s = np.nan_to_num(s); w = np.nan_to_num(w); ok = (w > 0) & (np.abs(s) >= 0.5 * w)
    return np.where(ok, np.angle(s), np.nan)
Vs = S.visits(df, keep_q=0.0, maxn=4000, min_reads=30); best = {}
for V in Vs: best[V['name']] = max(best.get(V['name'], -99), V['g'].rssi_dbm.max())
OUT, chk = {}, []
for name, gr in GR.items():
    G = gr['G'].astype(np.float32); cells = np.isfinite(gr['PH']); idx = np.flatnonzero(cells)
    AN = np.zeros(len(G), np.float32); used = 0
    Gt = torch.tensor(G[idx], device=dev)
    for V in [V for V in Vs if V['name'] == name and V['g'].rssi_dbm.max() >= best[name] - GATE]:
        g = V['g']; ps = psi_for(name, g.gid.values); m = np.isfinite(ps)
        if m.sum() < 30: continue
        g = g[m]; w = torch.tensor(10 ** ((g.rssi_dbm.values - g.rssi_dbm.max()) / 20), device=dev, dtype=torch.float32)
        A = torch.tensor(g[['ant_x', 'ant_y', 'ant_z']].values, device=dev, dtype=torch.float32)
        Bo = torch.tensor(np.c_[g.bore_x.values, g.bore_y.values, np.zeros(len(g))], device=dev, dtype=torch.float32)
        k = torch.tensor(g.k.values, device=dev, dtype=torch.float32); u = torch.tensor(g.ph.values - ps[m], device=dev, dtype=torch.float32)
        acc = torch.empty(len(idx), device=dev)
        for i0 in range(0, len(idx), 8192):
            a = u[None] - k[None] * torch.cdist(Gt[i0:i0 + 8192], A)
            Pc = Gt[i0:i0 + 8192]
            if COSW:                                      # weight by |cos| of the angle off boresight toward the candidate cell
                dv = Pc[:, None, :] - A[None]; ww = w[None] * (dv * Bo[None]).sum(-1).clamp(min=0) / dv.norm(dim=-1).clamp(min=1e-3)
            else: ww = w[None].expand(len(Pc), -1)
            if SLOPE:                                     # per-pass residual frequency slope as a nuisance: max over a range offset dR
                bm = torch.zeros(len(Pc), device=dev)
                for dR in DRS:
                    a2 = a - k[None] * dR; bm = torch.maximum(bm, torch.hypot((torch.cos(a2) * ww).sum(1), (torch.sin(a2) * ww).sum(1)) / ww.sum(1).clamp(min=1e-9))
                acc[i0:i0 + 8192] = bm
            else: acc[i0:i0 + 8192] = torch.hypot((torch.cos(a) * ww).sum(1), (torch.sin(a) * ww).sum(1)) / ww.sum(1).clamp(min=1e-9)
        AN[idx] += torch.log(acc + 1e-3).cpu().numpy(); used += 1
    AN[~cells] = AN[idx].min() if len(idx) else 0
    OUT[name] = AN
    if used:                                                     # sanity: argmax of the anchored term alone vs truth
        i = int(np.argmax(np.where(cells, AN, -np.inf))); n, xy, z = E.truth(name); room = R @ G[i, :2] + t
        chk.append(dict(name=name, ref=name.startswith('Ref '), passes=used, horiz=np.linalg.norm(room - xy), zabs=abs(G[i, 2] - z)))
    print(name, used, 'passes', flush=True)
pickle.dump(OUT, open(L + f'anchor_{tag}' + ('_em' if EM else '') + ('_cosw' if COSW else '') + ('_slope' if SLOPE else '') + '.pkl', 'wb'))
C = pd.DataFrame(chk); C = C[~C.name.isin(set(pd.read_csv(L + __import__('os').environ.get('RFID_TESTSET', 'testset_locked.csv')).name))]   # dev only
for nm, d in (('items', C[~C.ref]), ('refs (leave-self-out offsets)', C[C.ref])):
    print(f'anchored term alone, {nm}: n={len(d)} | horiz med {d.horiz.median()*100:.0f} cm | height med {d.zabs.median()*100:.0f} cm')
