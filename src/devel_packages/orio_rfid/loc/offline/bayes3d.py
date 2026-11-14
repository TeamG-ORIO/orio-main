"""Structure-free 3D tag posterior: learned robot-relative read/RSSI model (relmodel, per-tag gain nuisance) + dwell-phase
term (sarlib, strong passes only) on one map-frame x,y,z grid. Each tag uses the fold model that never saw its location
cluster; the weights (beta, kappa) are chosen per fold on tags outside it. Scored against room XYZ ground truth.
Usage: bayes3d.py <pose src> <tag>   -> grids3d_<tag>.pkl, loc_v3_<tag>_all.pkl, loc_v3_<tag>_cv.csv"""
import sys, os, pickle, numpy as np, pandas as pd, torch, relmodel as RM, sarlib as S
sys.path.insert(0, '/home/student/rfid_db/loc'); import eval_v1 as E
src, tag = sys.argv[1], sys.argv[2]; GATE = 3.0
FLAT = '--flat-phase' in sys.argv     # phase constrains x,y only: max over z per column (the ring makes it flat in z)
BETAS, KAPPAS = (0.003, 0.01, 0.03, 0.1), (0.0, 0.1, 0.3, 1.0, 3.0)
T = pd.read_pickle(f'/home/student/rfid_db/loc/dwells_{tag}.pkl'); meta = np.load(f'/home/student/rfid_db/loc/dwell_meta_{tag}.npz', allow_pickle=True)
D = pd.DataFrame(meta['D']); R, t = meta['R'], meta['t']
folds = pd.read_csv(f'/home/student/rfid_db/loc/relmodel_{tag}_folds.csv', index_col=0).iloc[:, 0]
models = {}
for k in sorted(set(folds)):
    m = RM.Rel().to(RM.dev); m.load_state_dict(torch.load(f'/home/student/rfid_db/loc/relmodel_{tag}_fold{k}.pt')); models[k] = m.eval()
gpath = f'/home/student/rfid_db/loc/grids3d_{tag}.pkl'
if os.path.exists(gpath): GR = pickle.load(open(gpath, 'rb'))
else:
    df = S.load_reads(); df = df[df.name.isin(T.name)]; traj, lift = S.load_traj(src); df = S.pose_reads(df, traj, lift)
    Vs = S.visits(df, keep_q=0.0, maxn=4000, min_reads=30); best = {}
    for V in Vs: best[V['name']] = max(best.get(V['name'], -99), V['g'].rssi_dbm.max())
    GR = {}
    for name, g in T.groupby('name'):
        rd = g[g.read == 1]
        if len(rd) < 5: continue
        reads = dict(zip(rd.gid, rd.rssi)); Dg = D.set_index('gid').loc[rd.gid]
        w = 10 ** ((rd.rssi.values - rd.rssi.max()) / 10)
        c = np.array([(Dg.ax.values * w).sum() / w.sum(), (Dg.ay.values * w).sum() / w.sum()])
        G, LP, shp = RM.posterior(models[int(folds[name])], D, RM.TYPES.index(g.tag_type.iloc[0]), reads, c)
        p = np.exp(0.01 * LP); keep = p > p.max() * 1e-4
        Gk = torch.tensor(G[keep], device=S.dev, dtype=torch.float32); ph = torch.zeros(len(Gk), device=S.dev)
        for V in [V for V in Vs if V['name'] == name and V['g'].rssi_dbm.max() >= best[name] - GATE]:
            Vd = S.prep_dwell(V, -1.0)
            if Vd['ng'] >= 15: ph += S.score_dwell(Gk, Vd['A'], Vd['k'], Vd['ph'], Vd['gid'], Vd['ng'])
        PH = np.full(len(G), np.nan, np.float32); PH[keep] = ph.cpu().numpy()
        GR[name] = dict(G=G.astype(np.float32), LP=LP.astype(np.float32), PH=PH, tm=np.array([g.tag_x.iloc[0], g.tag_y.iloc[0], g.tag_z.iloc[0]]))
        print(name, len(rd), 'reads', flush=True)
    pickle.dump(GR, open(gpath, 'wb'))
rows = []
for beta in BETAS:
    for kappa in KAPPAS:
        for name, gr in GR.items():
            G = gr['G'].astype(np.float64); ph = gr['PH'].astype(np.float64)
            if FLAT:
                nz = len(np.unique(G[:, 2])); c = np.nanmax(np.where(np.isfinite(ph), ph, -np.inf).reshape(-1, nz), 1)
                ph = np.repeat(np.where(np.isfinite(c), c, np.nan), nz)
            lp = beta * gr['LP'] + kappa * np.nan_to_num(ph, nan=np.nanmin(ph) if np.isfinite(ph).any() else 0.0)
            lp -= lp.max(); p = np.exp(lp); p /= p.sum(); i = int(p.argmax())
            order = np.argsort(-p); cs = np.cumsum(p[order]); rank = np.empty_like(order); rank[order] = np.arange(len(order))
            it = int(np.argmin(np.linalg.norm(G - gr['tm'], axis=1)))
            n, xy, z = E.truth(name); room = R @ G[i, :2] + t; e = room - xy; ez = G[i, 2] - z
            far = np.linalg.norm(G - G[i], axis=1) > 0.5
            rows.append(dict(beta=beta, kappa=kappa, name=name, ref=name.startswith('Ref '), fold=int(folds[name]),
                             horiz=np.linalg.norm(e), along=abs(e @ E.SH[n]['u']), zabs=abs(ez), err3=np.hypot(np.linalg.norm(e), ez),
                             hpd=cs[rank[it]], v90=(np.searchsorted(cs, 0.9) + 1) * 0.05 * 0.05 * 0.1, bimodal=p[far].sum() > 0.3,
                             x=G[i, 0], y=G[i, 1], z=G[i, 2]))
sfx = '_flat' if FLAT else ''
L = pd.DataFrame(rows); L.to_pickle(f'/home/student/rfid_db/loc/loc_v3_{tag}{sfx}_all.pkl')
def crit(d): return d.err3.median() * 100 + 100 * abs((d.hpd <= .9).mean() - .9)
out = []
for k in sorted(L.fold.unique()):
    tr = L[L.fold != k]; b = min((crit(g), bb, kk) for (bb, kk), g in tr.groupby(['beta', 'kappa']))
    out.append(L[(L.fold == k) & (L.beta == b[1]) & (L.kappa == b[2])].assign(pick=f'b={b[1]} k={b[2]}'))
O = pd.concat(out); O.to_csv(f'/home/student/rfid_db/loc/loc_v3_{tag}{sfx}_cv.csv', index=False)
print('per-fold picks:', O.groupby('fold').pick.first().to_dict())
for nm, d in (('items', O[~O.ref]), ('corners', O[O.ref])):
    print(f'{nm}: n={len(d)} | 3D med {d.err3.median()*100:.0f} cm (p90 {d.err3.quantile(.9)*100:.0f}) | horiz {d.horiz.median()*100:.0f} | '
          f'along {d.along.median()*100:.0f} | height {d.zabs.median()*100:.0f} | 3D within 30 cm {np.mean(d.err3 < .3):.2f}, within 50 cm {np.mean(d.err3 < .5):.2f} | '
          f'cov50 {np.mean(d.hpd <= .5):.2f} cov90 {np.mean(d.hpd <= .9):.2f} | 90% volume {d.v90.median():.2f} m3 | bimodal {d.bimodal.mean():.2f}')
