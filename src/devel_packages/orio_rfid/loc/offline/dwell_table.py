"""Training table for the robot-relative sensor model: one row per (known-position tag, global channel dwell) while the
antenna is within RMAX m horizontally. Read / not read, RSSI, and the tag's position relative to the antenna
(horizontal range, azimuth from boresight) plus antenna height. Ground truth enters the map frame through ONE global
map->room rigid tie fitted on the corner-tag estimates of loc_v1_<tag>.csv (sarF); no shelf information is a feature.
Map z = room Z (base_link at floor level): rows carry the tag's height, 3D range and elevation as well.
Usage: dwell_table.py <pose src> <loc_v1 tag>   -> dwells_<tag>.pkl, dwell_meta_<tag>.npz"""
import sys, numpy as np, pandas as pd, sarlib as S
sys.path.insert(0, '/home/student/rfid_db/loc'); import eval_v1 as E
src, tag = sys.argv[1], sys.argv[2]; RMAX = 12.0
df = S.load_reads(); (tt, x, y, yaw), (lt, lv) = S.load_traj(src)
df = df.sort_values('t').reset_index(drop=True)
# global dwells: the reader hops all tags together; new dwell on channel change or a gap > 0.25 s
new = np.r_[True, (df.channel.values[1:] != df.channel.values[:-1]) | (np.diff(df.t.values) > 0.25)]
df['gid'] = np.cumsum(new) - 1
D = df.groupby('gid').agg(t0=('t', 'min'), t1=('t', 'max'), channel=('channel', 'first'), f=('f', 'first'), nreads=('t', 'size'))
D['t'] = (D.t0 + D.t1) / 2
th = np.interp(D.t, tt, yaw); c, s = np.cos(th), np.sin(th)
D['ax'] = np.interp(D.t, tt, x) + S.OFF[0] * c - S.OFF[1] * s; D['ay'] = np.interp(D.t, tt, y) + S.OFF[0] * s + S.OFF[1] * c
D['az'] = S.ant_z(np.interp(D.t, lt, lv)); D['yaw'] = th
D['speed'] = np.hypot(np.gradient(np.interp(D.t, tt, x), D.t), np.gradient(np.interp(D.t, tt, y), D.t))
# ground truth -> map frame via the global tie (inverse of map->room)
L = pd.read_csv(f'/home/student/rfid_db/loc/loc_v1_{tag}.csv').set_index('name')
refs = [k for k in L.index if k.startswith('Ref ') and pd.notna(L.loc[k, 'sarF_x'])]
R, t = E.robust(L.loc[refs, ['sarF_x', 'sarF_y']].values, np.array([E.truth(k)[1] for k in refs]))
known = {k: E.truth(k) for k in L.index if E.truth(k) is not None}
types = df.drop_duplicates('name').set_index('name').tag_type
rows = []
per = df.groupby(['gid', 'name']).agg(n=('t', 'size'), rssi=('rssi_dbm', 'max')).reset_index()
for k, (shelf, xy, zs) in known.items():
    pm = R.T @ (xy - t)                                   # map-frame tag xy
    dx, dy = pm[0] - D.ax.values, pm[1] - D.ay.values; rh = np.hypot(dx, dy)
    m = rh < RMAX
    bore = np.arctan2(np.cos(D.yaw.values), -np.sin(D.yaw.values))       # boresight = robot left
    az = np.angle(np.exp(1j * (np.arctan2(dy, dx) - bore)))
    dz = zs - D.az.values[m]
    T = pd.DataFrame(dict(gid=D.index.values[m], name=k, rh=rh[m], az=az[m], ant_z=D.az.values[m], f=D.f.values[m],
                          dz=dz, r3=np.hypot(rh[m], dz), el=np.arctan2(dz, rh[m]), tag_z=zs,
                          speed=D.speed.values[m], dwell_reads=D.nreads.values[m], tag_type=types.get(k, 'regular'),
                          tag_x=pm[0], tag_y=pm[1], ref=k.startswith('Ref ')))
    rows.append(T)
T = pd.concat(rows).merge(per, on=['gid', 'name'], how='left')
T['read'] = T.n.notna().astype(np.int8); T['n'] = T.n.fillna(0).astype(np.int16)
T.to_pickle(f'/home/student/rfid_db/loc/dwells_{tag}.pkl')
np.savez(f'/home/student/rfid_db/loc/dwell_meta_{tag}.npz', R=R, t=t, D=D.reset_index().to_records(index=False))
print(f'{len(D)} global dwells, median {D.nreads.median():.0f} reads/dwell; {len(T)} rows for {T.name.nunique()} known tags; '
      f'read rate {T.read.mean():.3f}; within 1 m: {T[T.rh < 1].read.mean():.2f}, 2-3 m: {T[(T.rh > 2) & (T.rh < 3)].read.mean():.2f}, '
      f'>4 m: {T[T.rh > 4].read.mean():.2f}')
