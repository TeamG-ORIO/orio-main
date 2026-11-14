"""Pack the v2 run viewer's assets (room frame, metres) -> viz/site_v2/: path.b64.txt, est.b64.txt (float32 [method][snapshot][tag][xyz], NaN = no
belief yet), ell_F1..F4.b64.txt (14-float ellipsoid records as viz_pack.py), meta.json (tags, shelf boxes for the scene, per-method notes).
Reads viz/beliefs_<tag>_final.pkl (viz_beliefs_v2.py). Usage (v2env.sh + RFID_TESTSET): viz_pack_v2.py <pose src> <tag>"""
import sys, os, json, pickle, base64, numpy as np, pandas as pd
from scipy.spatial.transform import Rotation as Rot
import sarlib as S
sys.path.insert(0, '/home/student/rfid_db/loc'); import shelfprior_lib as SPL, eval_v1 as EV
src, tag = sys.argv[1], sys.argv[2]; L = '/home/student/rfid_db/loc/'; OUT = L + 'viz/site_v2'; os.makedirs(OUT, exist_ok=True)
meta = np.load(L + f'dwell_meta_{tag}.npz', allow_pickle=True); R, tm = meta['R'], meta['t']
B = pickle.load(open(L + f'viz/beliefs_{tag}_final.pkl', 'rb')); snaps = B['snaps']; NS = len(snaps); MS = ['F1', 'F2', 'F3', 'F4']
TEST = set(pd.read_csv(L + os.environ['RFID_TESTSET']).name); G = SPL.SHELF_GT
tr = np.loadtxt(src); t0 = tr[0, 0]; tt = np.arange(tr[0, 0], tr[-1, 0], 0.1)
xy = np.c_[np.interp(tt, tr[:, 0], tr[:, 1]), np.interp(tt, tr[:, 0], tr[:, 2])] @ R.T + tm
yaw = np.interp(tt, tr[:, 0], np.unwrap(tr[:, 3])) + np.arctan2(R[1, 0], R[0, 0])
lift = np.load(S.BAGNPZ, allow_pickle=True)['lift']; az = S.ant_z(np.interp(tt, lift[:, 0], lift[:, 1]))
def b64(name, arr): open(f'{OUT}/{name}.b64.txt', 'w').write(base64.b64encode(np.ascontiguousarray(arr, np.float32).tobytes()).decode())
b64('path', np.c_[tt - t0, xy, yaw, az])
names = sorted(G.index, key=lambda n: (int(G.shelf[n]), not n.startswith('Ref '), int(n.split()[-1]) if n.startswith('Item') else 0, n))
EST = np.full((len(MS), NS, len(names), 3), np.nan, np.float32); recs = {m: [] for m in MS}; tags = []; fin = []
for ti, n in enumerate(names):
    fr = [None] * NS if n.startswith('Ref ') else B['beliefs'].get(n, [None] * NS);   # corner tags are markers only (their positions are inputs to F2-F4)
    g = G.loc[n, ['X_room_m', 'Y_room_m', 'Z_room_m']].values.astype(float); ferr = {}; pl = {}; msh = {}
    for mi, m in enumerate(MS):
        for si, f in enumerate(fr):
            if f is None: continue
            EST[mi, si, ti] = f[m]['est']
            for li, lev in enumerate((0.5, 0.9)):
                for e in f[m]['ell'][lev]: recs[m].append([si, ti, li, *e['c'], *Rot.from_matrix(e['R'].astype(np.float64)).as_quat(), *e['r'], e['m']])
        last = fr[-1]
        ferr[m] = None if last is None else round(float(np.linalg.norm(last[m]['est'] - g)), 3)
        if last is not None and not n.startswith('Ref '): fin.append(dict(method=m, name=n, test=n in TEST, err3=ferr[m], hpd=last[m]['hpd']))
        if m in ('F3', 'F4'):
            pl[m] = [None if f is None else round(f[m]['plost'], 3) for f in fr]; msh[m] = [None if f is None else [f[m]['shelf'][0], round(f[m]['shelf'][1], 3)] for f in fr]
    obj = G.object[n] if 'object' in G and isinstance(G.object[n], str) else None
    tags.append(dict(name=n, shelf=int(G.shelf[n]), ref=n.startswith('Ref '), type=G.tag_type[n], test=n in TEST, object=obj, read=n in B['beliefs'],
                     gt=[round(float(v), 4) for v in g], final_err=ferr, plost=pl, mshelf=msh))
b64('est', EST)
for m in MS: b64('ell_' + m, np.array(recs[m], np.float32))
Fd = pd.DataFrame(fin); NOTES = {}
DESC = dict(F1='F1, no shelf map and no reference tags: reads, non-reads, RSSI, phase per height and the learned end-to-end network. Tags on one object share their evidence. Would find an item anywhere.',
            F2='F2, reference tags but no shelf map: F1 plus a term that pulls a tag toward the corner tags whose signal-strength pattern over the run looks like its own.',
            F3='F3, shelf boxes: F2 multiplied by a prior that puts 99% of the probability inside the mapped shelf boxes and 1% everywhere else.',
            F4='F4, shelf boxes weighted by where the other measured items sit (depth and board height on that shelf); 0.1% of the probability stays off the shelves.')
for m in MS:
    d = Fd[Fd.method == m]; dv, te = d[~d.test], d[d.test]
    NOTES[m] = dict(model=DESC[m], coverage=f'At the end of the run in this viewer: median 3D error {dv.err3.median()*100:.0f} cm on the {len(dv)} dev items and {te.err3.median()*100:.0f} cm on the '
                    f'{len(te)} locked test items. The true position was inside the 50% and 90% regions for {np.mean(dv.hpd <= .5)*100:.0f}% and {np.mean(dv.hpd <= .9)*100:.0f}% of dev items.')
    print(m, NOTES[m]['coverage'], '| all items', round(d.err3.median() * 100), 'cm')
dep = SPL.map_depths(TEST)
shelves = [dict(id=int(s), a=[float(v) for v in g['a']], u=[float(v) for v in g['u']], nrm=[float(v) for v in g['nrm']], W=float(g['W']), H=float(g['H']), z0=float(SPL.BL[s]), depth=float(dep[s]))
           for s, g in sorted(EV.SH.items())]
json.dump(dict(notes=NOTES, methods=MS, t0=float(t0), duration=float(tt[-1] - t0), snaps=[round(float(s - t0), 2) for s in snaps], tags=tags, shelves=shelves,
               nell={m: len(recs[m]) for m in MS}, npath=len(tt), antenna_offset=[S.OFF[0], S.OFF[1]]), open(f'{OUT}/meta.json', 'w'))
for f in sorted(os.listdir(OUT)): print(f, os.path.getsize(f'{OUT}/{f}') // 1024, 'KB')
# scene point cloud: the robot's dense ORB-SLAM map cloud (map-building run of 2026-11-06, same map frame the run localised in), copied to
# datasets/v2/map_cloud/. Cropped to the working area, floor thinned -> cloud.b64.txt float32 [x, y, z (room, m), grey 0..1]
cp = '/home/student/rfid_db/datasets/v2/map_cloud/'
if os.path.exists(cp + 'dense_orbslam_xyz.npy'):
    X = np.load(cp + 'dense_orbslam_xyz.npy').astype(np.float64); gr = np.load(cp + 'dense_orbslam_rgb.npy').astype(np.float64).mean(1) / 255
    X = np.c_[X[:, :2] @ R.T + tm, X[:, 2]]; lo_, hi_ = xy.min(0) - 2.5, xy.max(0) + 2.5; rng = np.random.default_rng(0)
    k = (X[:, 0] > lo_[0]) & (X[:, 0] < hi_[0]) & (X[:, 1] > lo_[1]) & (X[:, 1] < hi_[1]) & (X[:, 2] > -0.05) & (X[:, 2] < 2.0); k &= (X[:, 2] > 0.08) | (rng.random(len(X)) < 0.35)
    b64('cloud', np.c_[X[k], gr[k]]); print('cloud', int(k.sum()), 'points', os.path.getsize(f'{OUT}/cloud.b64.txt') // 1024, 'KB')
