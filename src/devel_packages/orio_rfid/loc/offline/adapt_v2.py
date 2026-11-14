"""v2 recording (robot_sync logger folder: raw reads CSV, _reader.json, base_link.tum, lift.csv, odom.csv) -> the inputs the frozen
pipeline needs from loc_v1.py onward. I/O only, no estimator logic.
Outputs in $RFID_ROOT/datasets/v2/<label>/:
  <label>.csv            reads, same columns; rows with no registry name (stray EPCs) dropped; tag_type 'unknown' -> 'regular' for items,
                         'metal' for reference tags (user decision 2026-11-09)
  <label>_reader.json    copy (hop table)
  bag_<label>.npz        lift (N,2: t, lift_m), odom (N,4: t, x, y, yaw)  -- what sarlib.load_traj reads
  base_traj_<label>_{raw,med,cf}.txt   t x y yaw (map frame, robot clock) from base_link.tum:
       raw = TUM as logged; med = 5-sample running median on x, y, yaw; cf = odometry-propagated local median (each pose = median of the
       SLAM poses within +-1 s carried to this instant by wheel-odometry relative motion). In all three the stretch before the first SLAM
       fix (pose frozen at the start waypoint, then a jump) is replaced by odometry aligned to the first 3 s after the fix.
Usage: adapt_v2.py <run dir> <label>"""
import sys, os, glob, shutil, json, numpy as np, pandas as pd
ROOT = os.environ.get('RFID_ROOT', '/home/student/rfid_db')
run, label = sys.argv[1].rstrip('/'), sys.argv[2]
out = f'{ROOT}/datasets/v2/{label}'; os.makedirs(out, exist_ok=True)
raw = [f for f in glob.glob(run + '/*.csv') if os.path.exists(f[:-4] + '_reader.json')]
assert len(raw) == 1, raw
df = pd.read_csv(raw[0], comment='#')
n0 = len(df); stray = df[df.name.isna()]; df = df[df.name.notna()].copy()
unk = df.tag_type.eq('unknown'); isref = df.name.str.startswith('Ref ')
df.loc[unk & isref, 'tag_type'] = 'metal'; df.loc[unk & ~isref, 'tag_type'] = 'regular'
other = sorted(set(df.tag_type) - {'regular', 'metal', 'liquid'}); assert not other, other
df.to_csv(f'{out}/{label}.csv', index=False)
shutil.copy(raw[0][:-4] + '_reader.json', f'{out}/{label}_reader.json')
lift = pd.read_csv(run + '/lift.csv'); odom = pd.read_csv(run + '/odom.csv')
O = odom[['t', 'x', 'y', 'yaw']].values.astype(float); O[:, 3] = np.unwrap(O[:, 3])
np.savez(f'{out}/bag_{label}.npz', lift=lift[['t', 'lift_m']].values.astype(float), odom=O)

a = np.loadtxt(run + '/base_link.tum'); t = a[:, 0]; x, y = a[:, 1].copy(), a[:, 2].copy()
yaw = np.unwrap(2 * np.arctan2(a[:, 6], a[:, 7]))
ox, oy, oyaw = (np.interp(t, O[:, 0], O[:, k]) for k in (1, 2, 3))
# first SLAM fix: before it the logged pose is odometry carried from the start waypoint, so (SLAM yaw - odom yaw) is constant;
# the fix is the first sample after the last one that still agrees with that constant to 0.01 rad, before the first >0.05 rad departure
dyo = yaw - oyaw; dev0 = np.abs(dyo - np.median(dyo[:20])); jmp = int(np.argmax(dev0 > 0.05))
ifix = int(np.flatnonzero(dev0[:jmp] < 0.01)[-1]) + 1
def odom_carry(i_from, i_to, dyaw):
    """SLAM pose i_from carried to instant i_to by odometry relative motion (dyaw = map yaw - odom yaw)"""
    c, s = np.cos(dyaw), np.sin(dyaw); dx, dy = ox[i_to] - ox[i_from], oy[i_to] - oy[i_from]
    return x[i_from] + c * dx - s * dy, y[i_from] + s * dx + c * dy
post = np.flatnonzero((t >= t[ifix]) & (t <= t[ifix] + 3.0))
dy0 = float(np.median(yaw[post] - oyaw[post]))
pre = np.arange(ifix)
cx = np.median([odom_carry(j, pre, dy0)[0] for j in post], 0); cy = np.median([odom_carry(j, pre, dy0)[1] for j in post], 0)
x[pre], y[pre] = cx, cy; yaw[pre] = oyaw[pre] + dy0
def save(tag, X, Y, TH): np.savetxt(f'{out}/base_traj_{label}_{tag}.txt', np.c_[t, X, Y, TH], fmt='%.6f %.5f %.5f %.6f')
save('raw', x, y, yaw)
def runmed(v, k=5):
    p = np.pad(v, k // 2, mode='edge'); return np.median(np.lib.stride_tricks.sliding_window_view(p, k), 1)
save('med', runmed(x), runmed(y), runmed(yaw))
# cf: odometry-propagated local median, window +-1 s
N = len(t); W = int(round(1.0 / np.median(np.diff(t)))); dyw = runmed(yaw - oyaw, 2 * W + 1)
ks = np.arange(-W, W + 1); idx = np.clip(np.arange(N)[None] + ks[:, None], 0, N - 1)
c, s = np.cos(dyw)[None], np.sin(dyw)[None]; dx, dy = ox[None] - ox[idx], oy[None] - oy[idx]
fx = np.median(x[idx] + c * dx - s * dy, 0); fy = np.median(y[idx] + s * dx + c * dy, 0)
save('cf', fx, fy, oyaw + dyw)
print(f'{label}: {n0} reads, {len(stray)} stray rows dropped ({stray.epc.nunique()} EPCs), {int(unk.sum())} reads with unknown type remapped; '
      f'{df.name.nunique()} named tags; SLAM fix at +{t[ifix]-t[0]:.2f} s ({ifix} pre-fix poses replaced); '
      f'host-reader clock {np.median(df.host_time_s - df.reader_us/1e6):.4f} s; '
      f'raw vs cf: median {100*np.median(np.hypot(x-fx, y-fy)):.1f} cm, p95 {100*np.quantile(np.hypot(x-fx, y-fy), .95):.1f} cm, max {100*np.max(np.hypot(x-fx, y-fy)):.1f} cm')
