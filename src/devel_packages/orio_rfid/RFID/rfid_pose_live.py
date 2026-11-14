#!/usr/bin/env python3
"""
rfid_pose_live.py - rfid_pose_join.py while the run is still going: every RFID read gets the antenna pose a moment
after it is logged, and the posed rows are appended to <run>/reads_posed_live.csv.

    rfid_pose_live.py <run dir> [--out FILE] [--wait 1.0] [--replay-from DIR --speed 1]

It follows three files that the run writes (scripts/rfid_nav_run.sh in orio_stretch):
    <label>_<date>.csv   raw reads (rfid_log.py)          host_time_s, epc, ..., rssi_dbm
    base_link.tum        live map pose (pose_logger.py)   t x y z qx qy qz qw
    lift.csv             lift height (pass_sidecar.py)    t, lift_m, ...
A read is written once a pose sample newer than the read exists (so it is interpolated, as in the offline join),
or after --wait seconds without one (then pose_dt_s says how far the nearest sample was).
Same columns and same maths as rfid_pose_join.py: the file can be used wherever reads_posed.csv is.
robot_sync.py --follow and the item localiser (loc/live) read this file. Runs until SIGINT/SIGTERM.

--replay-from DIR  rehearsal without the robot: plays an old run's reads_posed.csv into <run dir> at --speed times
                   real time, with the times shifted to now. Nothing else is needed in <run dir>.

Standard library + PyYAML. No ROS.
"""
import argparse
import csv
import glob
import io
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import rfid_pose_join as J      # noqa: E402  (same folder)

POSE_COLS = ['base_x', 'base_y', 'base_yaw', 'lift_m', 'ant_x', 'ant_y', 'ant_z', 'bore_x', 'bore_y', 'pose_src', 'pose_dt_s']
KEEP_S = 120.0      # pose / lift history kept in memory


class Tail:
    """complete new lines of a file that is being appended to (the file may not exist yet)"""

    def __init__(self, path):
        self.path, self.pos, self.rest = path, 0, ''

    def lines(self):
        try:
            with open(self.path) as f:
                f.seek(self.pos)
                chunk = f.read()
                self.pos = f.tell()
        except OSError:
            return []
        if not chunk:
            return []
        parts = (self.rest + chunk).split('\n')
        self.rest = parts.pop()
        return [p for p in parts if p.strip()]


def find_raw(run):
    """the rfid_log.py CSV of this run: has host_time_s but no pose columns"""
    for p in sorted(glob.glob(os.path.join(run, '*.csv'))):
        try:
            head = open(p).readline()
        except OSError:
            continue
        if head.startswith('host_time_s,') and 'base_x' not in head:
            return p
    return None


def replay(src, out, speed, stop):
    rows = list(csv.reader(l for l in open(os.path.join(src, 'reads_posed.csv')) if not l.startswith('#')))
    head, rows = rows[0], rows[1:]
    it = head.index('host_time_s')
    t0 = float(rows[0][it]); start = time.time()
    with open(out, 'w', newline='', buffering=1) as f:
        w = csv.writer(f); w.writerow(head)
        for r in rows:
            due = start + (float(r[it]) - t0) / speed
            while time.time() < due and not stop['now']:
                time.sleep(max(0.0, min(0.05, due - time.time())))
            if stop['now']:
                break
            r[it] = '%.6f' % (start + (float(r[it]) - t0) / speed)
            w.writerow(r)
    print('rfid_pose_live: replayed %s -> %s' % (src, out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run'); ap.add_argument('--out', default=None)
    ap.add_argument('--wait', type=float, default=1.0, help='seconds to wait for a newer pose sample before posing a read anyway')
    ap.add_argument('--antenna', default=os.path.join(os.path.dirname(os.path.realpath(__file__)), 'antenna_transform.yaml'))
    ap.add_argument('--replay-from', default=None); ap.add_argument('--speed', type=float, default=1.0)
    a = ap.parse_args()
    out = a.out or os.path.join(a.run, 'reads_posed_live.csv')
    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *x: stop.update(now=True)); signal.signal(signal.SIGTERM, lambda *x: stop.update(now=True))
    if a.replay_from:
        os.makedirs(a.run, exist_ok=True)
        return replay(a.replay_from, out, a.speed, stop)

    off = J.load_yaml_offset(a.antenna)
    tum, lift = Tail(os.path.join(a.run, 'base_link.tum')), Tail(os.path.join(a.run, 'lift.csv'))
    raw, head, pending = None, None, []        # pending: (t, row) not yet posed
    ts, xs, ys, yaws, lt, lv = [], [], [], [], [], []
    f = w = None
    n = n_far = 0
    while True:
        for line in tum.lines():
            if line.startswith('#'):
                continue
            try:
                v = [float(x) for x in line.split()]
                ts.append(v[0]); xs.append(v[1]); ys.append(v[2]); yaws.append(J.quat_to_yaw(v[4], v[5], v[6], v[7]))
            except (ValueError, IndexError):
                pass
        for line in lift.lines():
            c = line.split(',')
            try:
                lt.append(float(c[0])); lv.append(float(c[1]))
            except (ValueError, IndexError):
                pass                              # the header
        if raw is None:
            p = find_raw(a.run)
            raw = Tail(p) if p else None
        if raw is not None:
            for line in raw.lines():
                if line.startswith('#'):
                    continue
                row = next(csv.reader(io.StringIO(line)))
                if head is None:
                    head = row
                    f = open(out, 'w', newline='', buffering=1); w = csv.writer(f); w.writerow(head + POSE_COLS)
                    continue
                try:
                    pending.append((float(row[0]), row))
                except (ValueError, IndexError):
                    pass
        now, done = time.time(), 0
        for t, row in pending:
            have_newer = bool(ts) and ts[-1] >= t
            if not ts or not (have_newer or now - t > a.wait or stop['now']):
                break
            bx, by, yaw, dt = J.interp_pose(ts, xs, ys, yaws, t)
            L = J.interp_lift(lt, lv, t)
            c, s = math.cos(yaw), math.sin(yaw)
            ax = bx + off[0] * c - off[1] * s; ay = by + off[0] * s + off[1] * c; az = off[2] + L
            w.writerow(row + ['%.4f' % bx, '%.4f' % by, '%.4f' % yaw, '%.4f' % L, '%.4f' % ax, '%.4f' % ay, '%.4f' % az,
                              '%.4f' % -s, '%.4f' % c, 'live', '%.3f' % dt])
            done += 1; n += 1; n_far += abs(dt) > 0.2
        del pending[:done]
        if ts and ts[-1] - ts[0] > 2 * KEEP_S:       # old history is no longer needed: reads arrive in time order
            k = next(i for i, t in enumerate(ts) if t > ts[-1] - KEEP_S)
            del ts[:k], xs[:k], ys[:k], yaws[:k]
        if lt and lt[-1] - lt[0] > 2 * KEEP_S:
            k = next(i for i, t in enumerate(lt) if t > lt[-1] - KEEP_S)
            del lt[:k], lv[:k]
        if stop['now']:
            break
        time.sleep(0.2)
    if f:
        f.close()
    print('rfid_pose_live: %d reads posed -> %s (%d without a pose within 0.2 s, %d left unposed)' % (n, out, n_far, len(pending)))


if __name__ == '__main__':
    main()
