"""Score loc_v1 outputs against ground truth. The estimator never saw shelves; here ONE global map->room rigid 2D transform
is fit on the 28 reference-tag estimates (robust; each ref scored leave-one-out), standing in for the measured
room<->map tie. Height: map z = room Z (base_link is at floor level); scored absolute (z_abs) and relative to the same
shelf's BL estimate (z_rel). Ground truth: rfid_db/ground_truth/ground_truth_20261026_full.csv (all 110 tags, room XYZ).
Shelf geometry is used only to decompose errors (along / depth) and for the post-hoc nearest-shelf check.
Usage: eval_v1.py loc_v1_<a>.csv [loc_v1_<b>.csv ...]"""
import sys, numpy as np, pandas as pd
IN = 0.0254
MEAS = {1: (22, 26, 0, 0, 90), 2: (22, 40, 0, 70, 90), 3: (22, 25, -1.25, 148.5, 0), 4: (22, 26, 52.5, 148.5, 0),
        5: (28, 35.5, 116, 168, 270), 6: (49, 25, 108.75, 104.5, 270), 7: (14.5, 25.5, 154, 31.25, 180)}
SH = {}
for n, (W, H, X, Y, hd) in MEAS.items():
    th = np.radians(hd); u = np.array([np.cos(th), np.sin(th)]); SH[n] = dict(a=np.array([X, Y]) * IN, u=u, nrm=np.array([u[1], -u[0]]), W=W * IN, H=H * IN)
import os, json
if 'RFID_SITE' in os.environ:                # other arrangement (e.g. v2, 11 shelves): ground_truth/site_<x>.json, metres, same heading convention
    SH = {}
    for _n, _q in json.load(open(os.environ['RFID_SITE'])).items():
        _th = np.radians(_q['head']); _u = np.array([np.cos(_th), np.sin(_th)])
        SH[int(_n)] = dict(a=np.array([_q['X'], _q['Y']]), u=_u, nrm=np.array([_u[1], -_u[0]]), W=_q['W'], H=_q['H'])
GT_PATH = os.environ.get('RFID_GT', '/home/student/rfid_db/ground_truth/ground_truth_20261026_full.csv')   # per-run ground truth (moved items)
gt = pd.read_csv(GT_PATH).set_index('name')
def truth(k):
    """-> (shelf, room xy, room Z) or None"""
    if k in gt.index: r = gt.loc[k]; return int(r.shelf), np.array([r.X_room_m, r.Y_room_m]), float(r.Z_room_m)
    return None
def rigid(A, B):
    ca, cb = A.mean(0), B.mean(0); U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb)); d = np.sign(np.linalg.det(U @ Vt))
    R = (U @ np.diag([1, d]) @ Vt).T; return R, cb - R @ ca
def robust(A, B):
    R, t = rigid(A, B)
    for _ in range(3):
        e = np.linalg.norm(A @ R.T + t - B, axis=1); k = e <= np.quantile(e, .75); R, t = rigid(A[k], B[k])
    return R, t
def seg(p, n):
    s = SH[n]; d = p - s['a']; a = np.clip(d @ s['u'], 0, s['W']); return np.linalg.norm(d - a * s['u'])
if __name__ == '__main__':
    rows = []
    for f in sys.argv[1:]:
        L = pd.read_csv(f).set_index('name')
        for est in ('rssi', 'sar1', 'sarF'):
            X = L[[f'{est}_x', f'{est}_y', f'{est}_z']].dropna()
            refs = [k for k in X.index if k.startswith('Ref ')]; A = X.loc[refs, [f'{est}_x', f'{est}_y']].values
            B = np.array([truth(k)[1] for k in refs])
            R, t = robust(A, B); room = {k: R @ X.loc[k].values[:2] + t for k in X.index}
            for i, k in enumerate(refs):                                  # leave-one-out tie for the refs themselves
                m = np.arange(len(refs)) != i; Ri, ti = robust(A[m], B[m]); room[k] = Ri @ A[i] + ti
            res = []
            for k in X.index:
                tr = truth(k)
                if tr is None: continue
                n, xy, z = tr; s = SH[n]; e = room[k] - xy
                bl = f'Ref S{n} BL'; zr = X.loc[k, f'{est}_z'] - X.loc[bl, f'{est}_z'] if bl in X.index and k != bl else np.nan
                zbl = truth(bl)[2]
                ds = sorted((seg(room[k], j), j) for j in SH)
                res.append(dict(name=k, ref=k.startswith('Ref '), err=np.linalg.norm(e), along=abs(e @ s['u']), depth=abs(e @ s['nrm']),
                                zerr=abs(zr - (z - zbl)) if k != bl else np.nan, zabs=abs(X.loc[k, f'{est}_z'] - z), near_ok=ds[0][1] == n))
            d = pd.DataFrame(res); it, rf = d[~d.ref], d[d.ref]
            rows.append(dict(poses=f.split('loc_v1_')[1][:-4], est=est, items=len(it), horiz_med=it.err.median(), horiz_p90=it.err.quantile(.9),
                             along_med=it.along.median(), depth_med=it.depth.median(), z_abs_med=it.zabs.median(), z_rel_med=it.zerr.median(),
                             refs_horiz_med=rf.err.median(), refs_z_rel_med=rf.zerr.median(), nearest_shelf=f'{it.near_ok.sum()}/{len(it)}'))
            d.to_csv(f.replace('.csv', f'_{est}_eval.csv'), index=False)
    T = pd.DataFrame(rows); cm = [c for c in T if c.endswith('med') or c.endswith('p90')]; T[cm] = (T[cm] * 100).round(0)
    print('errors in cm; z_rel = height relative to the same shelf BL estimate');  print(T.to_string(index=False))
