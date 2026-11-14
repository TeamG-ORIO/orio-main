"""Belief-over-time for the v2 run viewer: the four final methods of final_apply.py (F1..F4, fixed settings, PLAN_methods_v2.md) rebuilt
CAUSALLY on one room-frame grid shared by all tags. At every snapshot only what was recorded up to that moment is used:
  read/RSSI  : fold read model, dwells added in time order (as viz_beliefs.py)
  phase      : per height cell (lift moves), channel dwells added once complete; a pass counts if it has >= 30 reads and >= 15 dwells so far
               and is within 3 dB of the tag's best pass so far
  e2e        : the fold network on the tag's windows cut at the snapshot time (the networks were trained on whole windows, so mid-pass
               output is rougher than the end-of-run output)
  fingerprint: RSSI-over-dwells profiles cut at the snapshot (Ts .02, sxy .3, sz .5 as picked by fp_term.py on the reference tags)
Tags of one object share evidence (F1-F3) once each has >= 5 reads. Shelf priors leave out the tag's object and the locked test set.
Per snapshot and method: the method's own point estimate, P(not on any shelf) and most likely shelf (F3, F4), and the 50% / 90% highest-density
regions of the temperature-flattened belief (final_T.json) as ellipsoids.
Usage (v2env.sh + RFID_TESTSET): viz_beliefs_v2.py <pose src> <tag> <e2e tag> [snap s] -> viz/beliefs_<tag>_final.pkl"""
import sys, os, json, pickle, time, numpy as np, pandas as pd, torch, torch.nn as nn, torch.nn.functional as Fn, relmodel as RM, sarlib as S
sys.path.insert(0, '/home/student/rfid_db/loc'); import shelfprior_lib as SPL, eval_v1 as EV
from scipy import ndimage
torch.set_num_threads(4); dev = 'cuda'; L = '/home/student/rfid_db/loc/'
src, tag, etag = sys.argv[1], sys.argv[2], sys.argv[3]; SNAP = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
ONLY = os.environ.get('VIZ_ONLY')                                             # debugging: comma-separated object names
GATE, MINREADS, MINMASS, NMODE, STEP, ZSTEP = 3.0, 5, 0.05, 3, 0.05, 0.10
F = dict(F1=dict(b=.03, k=.3, l=1.0, f=0.0, pk=1, pr='none', est='lmean'), F2=dict(b=.01, k=.3, l=.3, f=1.0, pk=1, pr='none', est='mode'),
         F3=dict(b=.01, k=.3, l=.3, f=.3, pk=1, pr='box', est='mode'), F4=dict(b=.01, k=.3, l=.3, f=0.0, pk=0, pr='kde', est='mode'))
EPS = dict(box=0.01, kde=0.001); TM = json.load(open(L + 'final_T.json')); FP_TS, FP_SXY, FP_SZ = 0.02, 0.3, 0.5
meta = np.load(L + f'dwell_meta_{tag}.npz', allow_pickle=True); R, tm = meta['R'], meta['t']
D = pd.DataFrame(meta['D']).sort_values('t').reset_index(drop=True); ND = len(D)
T = pd.read_pickle(L + f'dwells_{tag}.pkl'); folds = pd.read_csv(L + f'relmodel_{tag}_folds.csv', index_col=0).iloc[:, 0]
TEST = set(pd.read_csv(L + os.environ.get('RFID_TESTSET', 'testset_locked.csv')).name)
gt = SPL.SHELF_GT; OBJ = gt.object.where(gt.object.notna(), gt.index.to_series())
models = {}
for k in sorted(set(folds)):
    m = RM.Rel().to(dev); m.load_state_dict(torch.load(L + f'relmodel_{tag}_fold{k}.pt')); models[k] = m.eval()
# ---- grid (room frame), map-frame copy for the evidence terms
tr = np.loadtxt(src); path = tr[:, 1:3] @ R.T + tm; lo, hi = path.min(0) - 1.8, path.max(0) + 1.8
xs, ys, zs = np.arange(lo[0], hi[0], STEP), np.arange(lo[1], hi[1], STEP), np.arange(0.0, 1.6001, ZSTEP); shape = (len(xs), len(ys), len(zs))
Groom = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3); Gmap = np.c_[(Groom[:, :2] - tm) @ R, Groom[:, 2]]
G = torch.tensor(Gmap, device=dev, dtype=torch.float32); Gr = torch.tensor(Groom, device=dev, dtype=torch.float64); NC = len(G); CELL = STEP * STEP * ZSTEP
print(f'grid {shape} = {NC} cells, room x {lo[0]:.1f}..{hi[0]:.1f} y {lo[1]:.1f}..{hi[1]:.1f}', flush=True)
A = torch.tensor(D[['ax', 'ay', 'az', 'yaw']].values, device=dev, dtype=torch.float32)
snaps = np.arange(D.t.min() + SNAP, D.t.max() + SNAP, SNAP); dsnap = np.searchsorted(D.t.values, snaps, side='right'); NS = len(snaps)
gpos = {g: i for i, g in enumerate(D.gid.values)}
CG = torch.tensor(RM.CGRID, device=dev)
# ---- phase inputs
df = S.load_reads(); df = df[df.name.isin(T.name.unique())]; trj, lift = S.load_traj(src); df = S.pose_reads(df, trj, lift)
VT = {}
for V in S.visits(df, keep_q=0.0, maxn=4000, min_reads=30): VT.setdefault(V['name'], []).append(V)
# ---- reference fingerprint: similarity of RSSI profiles cut at each snapshot; anchor Gaussians on the grid
names_all = sorted(T.name.unique()); ix = {n: i for i, n in enumerate(names_all)}
Vr = np.zeros((len(names_all), ND), np.float32); r_ = T[T.read == 1]; Vr[r_.name.map(ix).values, r_.gid.map(gpos).values] = r_.rssi.values + 80
Vr = torch.tensor(Vr, device=dev); ker = torch.tensor([.25, .5, 1, .5, .25], device=dev).view(1, 1, 5)
Pm = T.drop_duplicates('name').set_index('name').loc[names_all, ['tag_x', 'tag_y', 'tag_z']].values
refs = np.array([i for i, n in enumerate(names_all) if n.startswith('Ref ')])
SIM = []
for i1 in dsnap:
    v = Vr.clone(); v[:, i1:] = 0; v = Fn.conv1d(v[:, None], ker, padding=2)[:, 0]; v = v / v.norm(dim=1, keepdim=True).clamp(min=1e-9); SIM.append((v @ v.T)[:, refs])
Pa = torch.tensor(Pm[refs], device=dev, dtype=torch.float32)
EA = torch.stack([torch.exp(-0.5 * (((G[:, :2] - p[:2]) ** 2).sum(1) / FP_SXY ** 2 + (G[:, 2] - p[2]) ** 2 / FP_SZ ** 2)) for p in Pa])   # (refs, NC)
def fp_term(i, si):
    s = SIM[si][i].clone() / FP_TS; own = torch.tensor(refs == i, device=dev); s[own] = -1e9
    w = torch.softmax(s, 0); return torch.log((w[:, None] * EA).sum(0).double() + 1e-300)
# ---- e2e (functions as e2e.py)
Dd = pickle.load(open(L + f'e2e_{etag}.pkl', 'rb')); te, Am, bore = Dd['t'], Dd['Am'], Dd['bore']
assert len(te) == ND and np.allclose(te, D.t.values, atol=1e-3), 'e2e dwells differ from the dwell table'
LMAX, HALF, EGAP, MINREAD, K, NF = 160, 15.0, 8.0, 8, 4, 19
def rot(v, ang):
    c, s = np.cos(ang), np.sin(ang); return np.stack([c * v[..., 0] - s * v[..., 1], s * v[..., 0] + c * v[..., 1]], -1)
def tokens(ref, tok, a):
    b = bore[ref]; rel = rot(Am[tok, :2] - Am[ref, :2], -b); da = a['da'][tok]; rd = a['read'][tok].astype(float); rs = np.where(rd > 0, a['rssi'][tok], 0.0)
    return np.c_[rel / 3, Am[tok, 2] - 1.1, np.cos(bore[tok] - b), np.sin(bore[tok] - b), rot(da[:, :2], -b) * 20, da[:, 2] * 20,
                 a['dr'][tok] * 20, (a['n'][tok] >= 2), rd, np.log1p(a['n'][tok]) / 2, np.where(rd > 0, (rs - a['rssi'][ref]) / 10, 0),
                 np.where(rd > 0, (rs + 60) / 10, 0), (te[tok] - te[ref]) / HALF, tok == ref, np.repeat(np.eye(3)[a['type']][None], len(tok), 0)].astype(np.float32)
class Net(nn.Module):
    def __init__(s, d=96, layers=3):
        super().__init__(); s.inp = nn.Sequential(nn.Linear(NF, d), nn.SiLU(), nn.Linear(d, d))
        s.enc = nn.TransformerEncoder(nn.TransformerEncoderLayer(d, 4, 2 * d, dropout=0.1, batch_first=True, norm_first=True), layers, enable_nested_tensor=False)
        s.q = nn.Parameter(torch.randn(1, 1, d) * 0.1); s.pool = nn.MultiheadAttention(d, 4, batch_first=True)
        s.out = nn.Sequential(nn.Linear(d, d), nn.SiLU(), nn.Linear(d, K * 7))
    def forward(s, X, M):
        h = s.enc(s.inp(X), src_key_padding_mask=M); p, _ = s.pool(s.q.expand(len(X), -1, -1), h, h, key_padding_mask=M); o = s.out(p[:, 0]).view(len(X), K, 7)
        return torch.log_softmax(o[..., 0], -1), o[..., 1:4] + torch.tensor([1.0, 0.0, 0.6], device=dev), o[..., 4:7].clamp(-3.5, 1.0)
nets = {}
for k in sorted(set(folds)):
    ck = torch.load(L + f'e2e_{etag}_fold{k}.pt'); n_ = Net().to(dev); n_.load_state_dict(ck['state']); nets[k] = (n_.eval(), float(ck['tau']))
REAL = {}
for name, g in Dd['F'].groupby('name'):
    a = dict(read=np.zeros(ND, bool), n=np.zeros(ND), rssi=np.zeros(ND), dr=np.zeros(ND), da=np.zeros((ND, 3))); i = g.gid.values
    a['read'][i] = True; a['n'][i] = g.n; a['rssi'][i] = g.rssi; a['dr'][i] = g.dr; a['da'][i] = g[['dax', 'day', 'daz']].values
    a['type'] = RM.TYPES.index(Dd['types'].get(name, 'regular')); REAL[name] = a
def windows_upto(a, i1):
    """the tag's windows from dwells 0..i1-1 only: read segments (gap 8 s, >= 8 read dwells), reference = strongest read so far, tokens = +-15 s cut at i1"""
    idx = np.flatnonzero(a['read'][:i1]); out = []
    if len(idx) < MINREAD: return out
    for seg in np.split(idx, np.flatnonzero(np.diff(te[idx]) > EGAP) + 1):
        if len(seg) < MINREAD: continue
        ref = seg[np.argmax(a['rssi'][seg])]; lo_, hi_ = np.searchsorted(te, te[ref] - HALF), min(np.searchsorted(te, te[ref] + HALF), i1); tok = np.arange(lo_, hi_)
        if len(tok) > LMAX: tok = np.sort(tok[np.argsort(np.abs(te[tok] - te[ref]))[:LMAX]])
        out.append((int(ref), tok))
    return out
@torch.no_grad()
def e2e_grid(name, W):
    net, tau = nets[int(folds[name])]; a = REAL[name]; items = [tokens(r, tk, a) for r, tk in W]; Lm = max(len(x) for x in items)
    X = np.zeros((len(items), Lm, NF), np.float32); M = np.ones((len(items), Lm), bool)
    for j, x in enumerate(items): X[j, :len(x)] = x; M[j, :len(x)] = False
    lw, mu, ls = net(torch.tensor(X, device=dev), torch.tensor(M, device=dev)); ls = ls + float(np.log(tau)); out = torch.zeros(NC, device=dev)
    for j, (r, _) in enumerate(W):
        c, s = float(np.cos(-bore[r])), float(np.sin(-bore[r])); dx, dy = G[:, 0] - float(Am[r, 0]), G[:, 1] - float(Am[r, 1])
        rel = torch.stack([c * dx - s * dy, s * dx + c * dy, G[:, 2]], -1)
        z = (rel[:, None] - mu[j][None]) / torch.exp(ls[j])[None]; lp = torch.logsumexp(lw[j][None] - 0.5 * (z ** 2).sum(-1) - ls[j].sum(-1)[None] - 2.7568, -1)
        out += torch.logaddexp(lp, torch.tensor(float(np.log(0.05 / 50.0)), device=dev))
    return out.double()
# ---- estimators and regions
def gk(sig):
    r = int(4 * sig + 0.5); x = torch.arange(-r, r + 1, device=dev, dtype=torch.float64); k = torch.exp(-0.5 * (x / sig) ** 2); return (k / k.sum()), r
KX, RX = gk(0.2 / STEP); KZ, RZ = gk(0.2 / ZSTEP)
def blur_mode(p):
    q = p.view(1, 1, *shape)
    q = Fn.conv3d(q, KX.view(1, 1, -1, 1, 1), padding=(RX, 0, 0)); q = Fn.conv3d(q, KX.view(1, 1, 1, -1, 1), padding=(0, RX, 0)); q = Fn.conv3d(q, KZ.view(1, 1, 1, 1, -1), padding=(0, 0, RZ))
    return int(q.argmax())
def lmean(p, i, r):
    m = (Gr - Gr[i]).norm(dim=1) < r; return ((p[m] @ Gr[m]) / p[m].sum()).cpu().numpy()
LEVELS = (0.5, 0.9)
def regions(p, itru):
    """p: normalised belief (flattened by the method's temperature) -> ({level: [ellipsoid]}, mass of the cells at least as likely as the truth cell)"""
    sp, order = torch.sort(p, descending=True); cs = torch.cumsum(sp, 0); res = {}
    hp = float(sp[sp >= p[itru]].sum()) if itru is not None else None
    cuts = [min(int(torch.searchsorted(cs, torch.tensor(lev, device=dev, dtype=sp.dtype))) + 1, NC) for lev in LEVELS]
    idx_all = order[:cuts[-1]].cpu().numpy(); w_all = sp[:cuts[-1]].cpu().numpy()
    for lev, cut in zip(LEVELS, cuts):
        idx, pw = idx_all[:cut], w_all[:cut]; ijk = np.stack(np.unravel_index(idx, shape), 1); o = ijk.min(0); sub = np.zeros(ijk.max(0) - o + 1, bool)
        sub[tuple((ijk - o).T)] = True; lab, nl = ndimage.label(sub, structure=np.ones((3, 3, 3))); lb = lab[tuple((ijk - o).T)]; out = []
        mass = np.bincount(lb, weights=pw, minlength=nl + 1)[1:]
        for j in np.argsort(-mass)[:NMODE]:
            if mass[j] < MINMASS * lev / 0.9: break
            sel = lb == j + 1; w = pw[sel] / pw[sel].sum(); P = Groom[idx[sel]]
            mu = w @ P; Cv = ((P - mu) * w[:, None]).T @ (P - mu) + np.diag([STEP, STEP, ZSTEP]) ** 2 / 12
            ev, evec = np.linalg.eigh(Cv); ax = np.sqrt(np.maximum(ev, 1e-6)); vol = sel.sum() * CELL; ax = ax * (vol / (4 / 3 * np.pi * np.prod(ax))) ** (1 / 3)
            ax = np.minimum(ax, np.quantile(np.abs((P - mu) @ evec), 0.95, axis=0) + np.array([STEP, STEP, ZSTEP]) / 2)
            if np.linalg.det(evec) < 0: evec[:, 0] *= -1
            out.append(dict(c=mu.astype(np.float32), R=evec.astype(np.float32), r=np.maximum(ax, 0.025).astype(np.float32), m=float(mass[j])))
        res[lev] = out
    return res, hp

class Tag:
    def __init__(s, name):
        g = T[T.name == name]; s.name = name; s.model = models[int(folds[name])]; s.tt = torch.tensor(RM.TYPES.index(g.tag_type.iloc[0]), device=dev)
        rd = g[g.read == 1]; reads = dict(zip(rd.gid, rd.rssi)); s.i = ix[name]
        s.y = torch.tensor([1.0 if gg in reads else 0.0 for gg in D.gid.values], device=dev); s.rs = torch.tensor([reads.get(gg, 0.0) for gg in D.gid.values], device=dev, dtype=torch.float32)
        s.nread = np.cumsum([gg in reads for gg in D.gid.values])
        s.LD = torch.zeros(len(CG), NC, device=dev); s.S0, s.S1, s.S2, s.SL = (torch.zeros(NC, device=dev) for _ in range(4)); s.i0 = 0
        s.vis = []
        for V in sorted(VT.get(name, []), key=lambda V: V['g'].t.min()):
            gd, gid = S.dwell_ids(V['g']); tend = pd.Series(gd.t.values).groupby(gid).transform('max').values if len(gd) else np.zeros(0)
            s.vis.append(dict(t=V['g'].t.values, r=V['g'].rssi_dbm.values, A=gd[['ant_x', 'ant_y', 'ant_z']].values, k=4 * np.pi * gd.f.values / S.C, ph=-gd.phase_rad.values,
                              gid=gid, tend=tend, acc=torch.zeros(NC, device=dev), ng=0))
        s.vbest = -99.0; s.e2 = None; s.sig = None
        tr_ = EV.truth(name); s.itru = int(np.argmin(np.linalg.norm(Groom - np.r_[tr_[1], tr_[2]], axis=1))) if tr_ else None
    @torch.no_grad()
    def advance(s, si):
        ts, i1 = snaps[si], int(dsnap[si]); t_prev = snaps[si - 1] if si else -np.inf; ch = max(1, int(2.0e6 // NC))
        for a0 in range(s.i0, i1, ch):
            a = A[a0:min(a0 + ch, i1)]; dx = G[None, :, 0] - a[:, None, 0]; dy = G[None, :, 1] - a[:, None, 1]; dz = G[None, :, 2] - a[:, None, 2]
            rh = torch.hypot(dx, dy); bo = torch.atan2(torch.cos(a[:, 3]), -torch.sin(a[:, 3]))[:, None]; az = torch.remainder(torch.atan2(dy, dx) - bo + np.pi, 2 * np.pi) - np.pi
            lg, mu, sg = s.model(RM.feats(rh, az, a[:, None, 2].expand_as(rh), s.tt, dz)); yy = s.y[a0:a0 + len(a), None]; r_ = s.rs[a0:a0 + len(a), None]
            for j in range(len(CG)):
                l2 = lg + CG[j]; s.LD[j] += torch.where(yy > 0, -Fn.softplus(-l2), -Fn.softplus(l2)).sum(0)
            w = yy / sg ** 2; res = r_ - mu; s.S0 += w.sum(0); s.S1 += (w * res).sum(0); s.S2 += (w * res ** 2).sum(0); s.SL += (yy * torch.log(sg)).sum(0)
        s.i0 = i1
        for v in s.vis:                                                              # phase: channel dwells that ended in (t_prev, ts]
            m = (v['tend'] > t_prev) & (v['tend'] <= ts)
            if m.any():
                gi = pd.factorize(v['gid'][m])[0]; ng = int(gi.max()) + 1; v['ng'] += ng
                v['acc'] += S.score_dwell(G, S.T(v['A'][m]), S.T(v['k'][m]), S.T(v['ph'][m]), torch.tensor(gi, device=dev), ng)
            seen = v['t'] <= ts
            if seen.any(): s.vbest = max(s.vbest, float(v['r'][seen].max()))
        if s.nread[i1 - 1] < MINREADS: return None
        PH = torch.zeros(NC, device=dev)
        for v in s.vis:
            seen = v['t'] <= ts
            if seen.sum() >= 30 and v['ng'] >= 15 and v['r'][seen].max() >= s.vbest - GATE: PH += v['acc']
        b = (s.S1 / s.S0.clamp(min=1e-9)).clamp(-RM.BMAX, RM.BMAX); LP = (s.LD.max(0).values - 0.5 * (s.S2 - 2 * b * s.S1 + b * b * s.S0) - s.SL).double()
        W = windows_upto(REAL[s.name], i1) if s.name in REAL else []; sig = (tuple((r, int(tk[0]), int(tk[-1])) for r, tk in W), int(s.nread[i1 - 1]))
        if sig != s.sig: s.sig = sig; s.e2 = e2e_grid(s.name, W) if W else None
        E2 = s.e2 - s.e2.max() if s.e2 is not None else torch.zeros(NC, device=dev, dtype=torch.float64)
        return dict(LP=LP - LP.max(), PH=PH.double(), E2=E2, FP=fp_term(s.i, si), e2e=s.e2 is not None)

@torch.no_grad()
def run_object(names):
    tags = [Tag(n) for n in names]; ex = TEST | set(OBJ.index[OBJ == OBJ[names[0]]]) if names[0] in OBJ.index else TEST | set(names)
    sh = SPL.shelf_of(Gmap, R, tm, SPL.map_depths(ex)); kde = SPL.kde_weight(Gmap, R, tm, sh, ex); sh_t = torch.tensor(sh, device=dev); nsh = int(max(EV.SH)) + 1
    PR = {k: torch.tensor(EPS[k] / NC + (1 - EPS[k]) * w / max(w.sum(), 1e-9), device=dev, dtype=torch.float64) for k, w in (('box', (sh > 0).astype(float)), ('kde', kde))}
    out = {n: [] for n in names}
    for si in range(NS):
        ev = {t.name: t.advance(si) for t in tags}
        for t in tags:
            e = ev[t.name]
            if e is None: out[t.name].append(None); continue
            fr = dict(e2e=e['e2e'])
            for fn, c in F.items():
                def comb(q): v = c['b'] * q['LP'] + c['k'] * q['PH'] + c['l'] * q['E2'] + c['f'] * q['FP']; return v - v.max()
                v = comb(e)
                if c['pk']:
                    for m in tags:
                        if m is not t and ev[m.name] is not None: v = v + comb(ev[m.name])
                p = torch.softmax(v, 0); plost = shelf = None
                if c['pr'] != 'none':
                    q = p * PR[c['pr']]; Z = q.sum(); plost = float((p * EPS[c['pr']] / NC).sum() / Z); p = q / Z
                    ms = torch.zeros(nsh, device=dev, dtype=torch.float64).index_add_(0, sh_t, p); sp_ = int(ms[1:].argmax()) + 1; shelf = (sp_, float(ms[sp_]))
                est = lmean(p, int(p.argmax()), 0.3) if c['est'] == 'lmean' else lmean(p, blur_mode(p), 0.2)
                reg, hp = regions(torch.softmax(torch.log(p + 1e-300) / TM[fn], 0), t.itru)
                fr[fn] = dict(est=est.astype(np.float32), ell=reg, hpd=hp, plost=plost, shelf=shelf)
            out[t.name].append(fr)
    return out

t0 = time.time(); OUT = {}
objs = {}
for n in names_all: objs.setdefault(OBJ[n] if n in OBJ.index else n, []).append(n)
if ONLY: objs = {k: v for k, v in objs.items() if k in ONLY.split(',')}
for j, (o, names) in enumerate(objs.items()):
    OUT.update(run_object(names))
    if j % 10 == 0 or ONLY: print(f'{j + 1}/{len(objs)} {o} {names}: {[sum(f is not None for f in OUT[n]) for n in names]} snapshots with a belief; {(time.time() - t0) / 60:.1f} min', flush=True)
pickle.dump(dict(snaps=snaps, beliefs=OUT, grid=dict(lo=lo, hi=hi, shape=shape), methods=F, T=TM), open(L + f'viz/beliefs_{tag}_final{"_dbg" if ONLY else ""}.pkl', 'wb'))
print('done', f'{(time.time() - t0) / 60:.1f} min')
