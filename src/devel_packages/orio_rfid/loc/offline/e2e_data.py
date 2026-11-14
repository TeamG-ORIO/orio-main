"""Per-dwell inputs for the end-to-end localizer (e2e.py). Global channel dwells over ALL reads (same segmentation as
dwell_table.py), antenna pose at each dwell's start / middle / end, and per (tag, dwell): read count, max RSSI, and the
within-dwell phase change expressed as range change (sum of wrapped consecutive differences / k = rho_last - rho_first)
with the antenna displacement between the first and last read. Ground truth (map frame) only as training targets.
Usage: e2e_data.py <pose src> <tag>  -> e2e_<tag>.pkl"""
import sys, pickle, numpy as np, pandas as pd, sarlib as S
src, tag = sys.argv[1], sys.argv[2]
df = S.load_reads(); traj, lift = S.load_traj(src); df = S.pose_reads(df, traj, lift)
df = df.sort_values('t').reset_index(drop=True)
new = np.r_[True, (df.channel.values[1:] != df.channel.values[:-1]) | (np.diff(df.t.values) > 0.25)]
df['gid'] = np.cumsum(new) - 1
D = df.groupby('gid').agg(t0=('t', 'min'), t1=('t', 'max'), f=('f', 'first')).reset_index()
(tt, x, y, yaw), (lt, lv) = traj, lift
def ant(tq):
    th = np.interp(tq, tt, yaw); c, s = np.cos(th), np.sin(th)
    return np.c_[np.interp(tq, tt, x) + S.OFF[0] * c - S.OFF[1] * s, np.interp(tq, tt, y) + S.OFF[0] * s + S.OFF[1] * c,
                 S.ant_z(np.interp(tq, lt, lv))], th
A0, _ = ant(D.t0.values); A1, _ = ant(D.t1.values); D['t'] = (D.t0 + D.t1) / 2; Am, th = ant(D.t.values)
bore = th + np.pi / 2                                                     # boresight = robot left
# per (tag, dwell) features
df = df.sort_values(['name', 'gid', 't']).reset_index(drop=True)
k = 4 * np.pi * df.f.values / S.C
same = (df.name.values[1:] == df.name.values[:-1]) & (df.gid.values[1:] == df.gid.values[:-1])
dph = np.angle(np.exp(1j * -(df.phase_rad.values[1:] - df.phase_rad.values[:-1]))) / k[1:]
df['dr'] = np.r_[0.0, np.where(same, dph, 0.0)]
g = df.groupby(['name', 'gid'])
F = g.agg(n=('t', 'size'), rssi=('rssi_dbm', 'max'), dr=('dr', 'sum'),
          ax0=('ant_x', 'first'), ay0=('ant_y', 'first'), az0=('ant_z', 'first'),
          ax1=('ant_x', 'last'), ay1=('ant_y', 'last'), az1=('ant_z', 'last')).reset_index()
F['dax'], F['day'], F['daz'] = F.ax1 - F.ax0, F.ay1 - F.ay0, F.az1 - F.az0
F = F[['name', 'gid', 'n', 'rssi', 'dr', 'dax', 'day', 'daz']]
types = df.drop_duplicates('name').set_index('name').tag_type.to_dict()
dt = pd.read_pickle(f'/home/student/rfid_db/loc/dwells_{tag}.pkl')                 # map-frame ground truth per known tag
gt = dt.drop_duplicates('name').set_index('name')[['tag_x', 'tag_y', 'tag_z']]
pickle.dump(dict(t=D.t.values, A0=A0, A1=A1, Am=Am, bore=bore, f=D.f.values, F=F, types=types, gt=gt), open(f'/home/student/rfid_db/loc/e2e_{tag}.pkl', 'wb'))
print(f'{len(D)} dwells, {len(F)} (tag, dwell) read cells for {F.name.nunique()} tags; ground truth for {len(gt)}; '
      f'dwells with >=2 reads: {(F.n >= 2).mean():.2f}; median |dr| {F[F.n >= 2].dr.abs().median()*100:.2f} cm')
