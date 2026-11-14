#!/usr/bin/env python3
"""
check_run2.py - score the live localiser against the tape-measured positions and against abuela's own offline result.

    python3 engine.py --once ~/orio/datasets/corridor_v2_rfid_2laps_run2/reads_posed.csv --method F4 --out /tmp/f4.csv
    python3 check_run2.py /tmp/f4.csv F4 [offline results] [offline tie]

offline results   folder with final_r2cvL.csv (run 2, the default: ~/orio/rfid_loc/run2), or the final_<tag>.csv of another
                  recording made on abuela with final_apply.py (e.g. final_run1bF.csv for corridor_v2_rfid_2laps)
offline tie       dwell_meta_<run>.npz of that recording: abuela fits the map -> room tie per recording, so its estimates
                  of another run are only comparable through that run's own tie (default: the site file's, from run 2)

Prints the median 3D error on the dev and the locked test items next to the offline numbers of the same method, how
far the two sets of estimates are from each other, and the mean offset of the live estimates from the tape positions
(a whole-map shift shows up there).
"""
import json, os, sys
import numpy as np, pandas as pd
LOC = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
est, method = pd.read_csv(sys.argv[1]).set_index('item_name'), sys.argv[2]
ref_dir = os.path.expanduser(sys.argv[3] if len(sys.argv) > 3 else '~/orio/rfid_loc/run2')
site = json.load(open(os.path.join(LOC, 'site', 'live_site_v2.json'))); R, t = np.array(site['map_to_room']['R']), np.array(site['map_to_room']['t'])
gt = pd.read_csv(os.path.join(LOC, 'site', site['ground_truth'])).set_index('name')
test = set(pd.read_csv(os.path.join(LOC, 'site', site['testset'])).name)
off = pd.read_csv(os.path.join(ref_dir, 'final_r2cvL.csv') if os.path.isdir(ref_dir) else ref_dir); off = off[off.method == method].set_index('name')
Ro, to = R, t
if len(sys.argv) > 4:
    m = np.load(os.path.expanduser(sys.argv[4]), allow_pickle=True); Ro, to = m['R'], m['t']
rows = []
for n in off.index:
    g = gt.loc[n, ['X_room_m', 'Y_room_m', 'Z_room_m']].values.astype(float)
    o = np.r_[Ro @ off.loc[n, ['x', 'y']].values.astype(float) + to, off.z[n]]        # offline estimate, map -> room (its own tie)
    r = dict(name=n, test=n in test, off=np.linalg.norm(o - g), shelf=int(gt.shelf[n]))
    if n in est.index:
        e = est.loc[n, ['x_m', 'y_m', 'z_m']].values.astype(float)
        r.update(live=np.linalg.norm(e - g), apart=np.linalg.norm(e - o), horiz=np.linalg.norm(e[:2] - g[:2]), height=abs(e[2] - g[2]), dx=e[0] - g[0], dy=e[1] - g[1], dz=e[2] - g[2],
                 shelf_ok=None if pd.isna(est.shelf[n]) else int(est.shelf[n]) == int(gt.shelf[n]))
    rows.append(r)
d = pd.DataFrame(rows); cm = lambda v: '%.0f' % (100 * v)
print('%s: %d items scored offline, %d of them placed live' % (method, len(d), d.live.notna().sum()))
for name, k in (('dev', ~d.test), ('locked test', d.test), ('all', d.test | ~d.test)):
    q = d[k & d.live.notna()]
    print('  %-11s n=%3d  live median 3D %s cm (p90 %s, horiz %s, height %s)   offline %s cm (p90 %s)   live vs offline apart: median %s cm, p90 %s' % (
        name, len(q), cm(q.live.median()), cm(q.live.quantile(.9)), cm(q.horiz.median()), cm(q.height.median()), cm(q.off.median()), cm(q.off.quantile(.9)),
        cm(q.apart.median()), cm(q.apart.quantile(.9))))
if d.shelf_ok.notna().any():
    print('  right shelf: %d of %d' % (d.shelf_ok.sum(), d.shelf_ok.notna().sum()))
q = d[d.live.notna()]
print('  mean offset of the live estimates from the tape positions: dx %+.0f cm, dy %+.0f cm, dz %+.0f cm' % (100 * q.dx.mean(), 100 * q.dy.mean(), 100 * q.dz.mean()))
