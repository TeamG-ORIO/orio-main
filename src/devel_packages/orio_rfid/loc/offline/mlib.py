"""Shared harness for the 2026-11-10 method study (PLAN_methods_v2.md): cached evidence terms of a CV tag (grids3d, e2e, anchor), shelf priors
with the scored tag, its packet mates and the locked test set left out, estimators, and one scorer. Environment as v2env.sh + RFID_TESTSET."""
import os, sys, pickle, numpy as np, pandas as pd
sys.path.insert(0, '/home/student/rfid_db/loc'); import eval_v1 as E, shelfprior_lib as SPL
from scipy.ndimage import gaussian_filter
L = '/home/student/rfid_db/loc/'
class Run:
    def __init__(s, tag, e2e_file=None):
        s.tag = tag; s.GR = pickle.load(open(L + f'grids3d_{tag}.pkl', 'rb')); s.GLP = pickle.load(open(L + (e2e_file or f'e2e_{tag}_gridlp_v3.pkl'), 'rb'))
        s.AN = pickle.load(open(L + f'anchor_{tag}.pkl', 'rb')); m = np.load(L + f'dwell_meta_{tag}.npz', allow_pickle=True); s.R, s.t = m['R'], m['t']
        s.FP = pickle.load(open(L + f'fp_{tag}.pkl', 'rb')) if os.path.exists(L + f'fp_{tag}.pkl') else None    # reference-fingerprint term (fp_term.py)
        s.folds = pd.read_csv(L + f'relmodel_{tag}_folds.csv', index_col=0).iloc[:, 0]
        s.TEST = set(pd.read_csv(L + os.environ.get('RFID_TESTSET', 'testset_locked.csv')).name)
        gt = SPL.SHELF_GT; s.obj = gt.object.where(gt.object.notna(), gt.index.to_series()) if 'object' in gt else gt.index.to_series()
        s.names = [n for n in s.GR if n in s.GLP and n in s.AN and E.truth(n) is not None]
        s.items = [n for n in s.names if not n.startswith('Ref ')]
    def mates(s, name): return [n for n in s.obj.index[s.obj == s.obj[name]]]
    def terms(s, name):
        gr = s.GR[name]; G = gr['G'].astype(np.float64); ph = gr['PH'].astype(np.float64)
        ph = np.nan_to_num(ph, nan=np.nanmin(ph) if np.isfinite(ph).any() else 0.0)
        e2 = s.GLP[name].astype(np.float64); an = s.AN[name].astype(np.float64)
        fp = s.FP[name].astype(np.float64) if s.FP is not None else np.zeros(len(G))
        return dict(FP=fp - fp.max(), G=G, LP=gr['LP'].astype(np.float64), PH=ph, PHf=SPL.flat_phase(G, gr['PH']), E2=e2 - e2.max(), AN=an - an.max(),
                    shp=(len(np.unique(G[:, 0])), len(np.unique(G[:, 1])), len(np.unique(G[:, 2]))))
    def priors(s, name, G):
        ex = s.TEST | set(s.mates(name)); gd = os.environ.get('RFID_GENERIC_DEPTH')    # RFID_GENERIC_DEPTH=0.45: boxes from the corner tags + one nominal depth, no measured depths
        sh = SPL.shelf_of(G, s.R, s.t, float(gd) if gd else SPL.map_depths(ex))
        return sh, (sh > 0).astype(float), SPL.kde_weight(G, s.R, s.t, sh, ex)
    def err(s, name, P):
        n, xy, z = E.truth(name); e = s.R @ P[:2] + s.t - xy; return dict(horiz=float(np.linalg.norm(e)), zabs=float(abs(P[2] - z)), err3=float(np.hypot(np.linalg.norm(e), P[2] - z)))
def norm(lp): lp = lp - lp.max(); p = np.exp(lp); return p / p.sum()
def with_prior(p, inn, eps):
    q = p * (eps / len(p) + (1 - eps) * inn / max(inn.sum(), 1e-9)); return q / q.sum()
def lmean(G, p, i, r=0.3):
    m = np.linalg.norm(G - G[i], axis=1) < r; return (p[m] @ G[m]) / p[m].sum()
def blur_mode(G, p, shp, s):
    """dominant mode: argmax of the belief smoothed with a Gaussian of s metres (cells 5 x 5 x 10 cm) = where the most mass is, then local mean"""
    q = gaussian_filter(p.reshape(shp), (s / .05, s / .05, s / .10), mode='constant').ravel(); return int(q.argmax())
def hpd(p, it):
    o = np.argsort(-p); cs = np.cumsum(p[o]); r = np.empty_like(o); r[o] = np.arange(len(o)); return float(cs[r[it]])
def resample(Gs, v, shp, Gq, fill):
    """values v on grid Gs (regular, shp) looked up at points Gq (nearest cell); outside -> fill"""
    o = Gs[0]; ix = np.rint((Gq[:, 0] - o[0]) / .05).astype(int); iy = np.rint((Gq[:, 1] - o[1]) / .05).astype(int); iz = np.rint((Gq[:, 2] - o[2]) / .10).astype(int)
    ok = (ix >= 0) & (ix < shp[0]) & (iy >= 0) & (iy < shp[1]) & (iz >= 0) & (iz < shp[2]); out = np.full(len(Gq), fill)
    out[ok] = v.reshape(shp)[ix[ok], iy[ok], iz[ok]]; return out
def pick_cv(A, keys, folds, TEST, col='err3', extra=None):
    """per location fold: the setting with the lowest dev median of `col` on items of the OTHER folds -> rows of that setting for the fold"""
    A = A.assign(fold=A.name.map(folds), test=A.name.isin(TEST)); out = []
    for k in sorted(A.fold.unique()):
        tr = A[(A.fold != k) & ~A.test]; b = tr.groupby(keys)[col].median().idxmin(); b = b if isinstance(b, tuple) else (b,)
        m = A.fold == k
        for kk, v in zip(keys, b): m &= A[kk] == v
        out.append(A[m].assign(pick=str(b)))
    return pd.concat(out)
def report(O, label):
    r = []
    for sp, d in (('dev', O[~O.test]), ('test', O[O.test])):
        r.append(f'{sp} n={len(d)} 3D {d.err3.median()*100:.0f} (p90 {d.err3.quantile(.9)*100:.0f}) horiz {d.horiz.median()*100:.0f} height {d.zabs.median()*100:.0f} <0.62 {np.mean(d.err3 < .62):.2f}'
                 + (f' cov90 {np.mean(d.hpd <= .9):.2f}' if 'hpd' in d and d.hpd.notna().any() else ''))
    print(f'{label:46s} | ' + ' | '.join(r), flush=True)
    return [dict(method=label, split=sp, n=len(d), horiz=d.horiz.median() * 100, height=d.zabs.median() * 100, err3=d.err3.median() * 100, err3_p90=d.err3.quantile(.9) * 100,
                 within62=np.mean(d.err3 < .62), cov90=np.mean(d.hpd <= .9) if 'hpd' in d and d.hpd.notna().any() else np.nan) for sp, d in (('dev', O[~O.test]), ('test', O[O.test]))]
