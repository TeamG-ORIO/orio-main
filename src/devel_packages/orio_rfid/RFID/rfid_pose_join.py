#!/usr/bin/env python3
"""
rfid_pose_join.py - give every RFID read the antenna pose at that instant.

    rfid_pose_join.py <pass.csv> --bag <run.bag> [--traj base_link.tum] [--out reads_posed.csv]
    rfid_pose_join.py <pass.csv> --lift-csv <run>/lift.csv --traj base_link.tum      (c2 passes: no bag scan needed)
    ... [--antenna-port 2=antenna_transform_port2.yaml]   a second panel on another reader port

The RFID log (host_time_s) and the rosbag share the robot's clock, so the two are joined by ABSOLUTE time; it does
not matter when each command was started, only the overlap counts. Base pose comes from --traj (TUM: t x y z qx qy qz qw
of base_link in the map, e.g. <run>/orbslam/ir-rgbd/base_link.tum from scripts/orbslam_run.sh) or, without it, from
/odom in the bag (odometry frame, drifts). Lift height comes from /joint_states (joint_lift) or the sidecar lift.csv, linearly interpolated (the c2 passes
move the lift during the pass). The antenna point is
antenna_transform.yaml's base_link offset with the lift added to z; the boresight is base +y.

Output columns: the input columns + base_x base_y base_yaw lift_m ant_x ant_y ant_z bore_x bore_y pose_src pose_dt_s
"""
import argparse
import bisect
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))


def load_yaml_offset(path):
    import yaml
    y = yaml.safe_load(open(path))
    return y['base_link_to_antenna_back_centre']['xyz_m']


def quat_to_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def interp_pose(ts, xs, ys, yaws, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return xs[0], ys[0], yaws[0], ts[0] - t
    if i >= len(ts):
        return xs[-1], ys[-1], yaws[-1], t - ts[-1]
    t0, t1 = ts[i - 1], ts[i]; a = (t - t0) / (t1 - t0) if t1 > t0 else 0
    dyaw = (yaws[i] - yaws[i - 1] + math.pi) % (2 * math.pi) - math.pi
    return xs[i - 1] + a * (xs[i] - xs[i - 1]), ys[i - 1] + a * (ys[i] - ys[i - 1]), yaws[i - 1] + a * dyaw, 0.0


def interp_lift(lt, lv, t):
    if not lv:
        return 0.0
    j = bisect.bisect_left(lt, t)
    if j <= 0:
        return lv[0]
    if j >= len(lt):
        return lv[-1]
    t0, t1 = lt[j - 1], lt[j]
    return lv[j - 1] + (lv[j] - lv[j - 1]) * ((t - t0) / (t1 - t0) if t1 > t0 else 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv'); ap.add_argument('--bag', default=None); ap.add_argument('--traj', default=None)
    ap.add_argument('--lift-csv', default=None, help='sidecar lift.csv (t, lift_m, ...) instead of /joint_states from the bag')
    ap.add_argument('--antenna-port', action='append', default=[], metavar='PORT=YAML', help='offset file for another reader port (default: --antenna for every port)')
    ap.add_argument('--antenna', default=os.path.join(HERE, 'antenna_transform.yaml')); ap.add_argument('--out', default=None)
    ap.add_argument('--max-dt', type=float, default=0.2, help='reads farther than this from any pose sample are flagged (pose_dt_s)')
    a = ap.parse_args()
    off = load_yaml_offset(a.antenna)
    port_off = {}
    for spec in a.antenna_port:
        port, path = spec.split('=', 1); port_off[port.strip()] = load_yaml_offset(path)
    ts, xs, ys, yaws = [], [], [], []; lt, lv = [], []
    src = 'odom'
    if a.lift_csv:
        for r in csv.DictReader(open(a.lift_csv)):
            lt.append(float(r['t'])); lv.append(float(r['lift_m']))
    if lt:
        o2 = sorted(range(len(lt)), key=lambda i: lt[i]); lt, lv = [lt[i] for i in o2], [lv[i] for i in o2]
    need = ([] if a.lift_csv else ['/joint_states']) + ([] if a.traj else ['/odom'])
    if need and not a.bag:
        sys.exit('need --bag for %s (or --lift-csv and --traj)' % ' '.join(need))
    if need:
        import rosbag
        print('reading %s (%s) ...' % (os.path.basename(a.bag), ' + '.join(need)))
    for topic, msg, t in (rosbag.Bag(a.bag).read_messages(topics=need) if need else []):
        if topic == '/joint_states':
            if 'joint_lift' in msg.name:
                lt.append(msg.header.stamp.to_sec()); lv.append(msg.position[msg.name.index('joint_lift')])
        else:
            p = msg.pose.pose; ts.append(msg.header.stamp.to_sec()); xs.append(p.position.x); ys.append(p.position.y); yaws.append(quat_to_yaw(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
    if a.traj:
        src = 'traj'
        for line in open(a.traj):
            if line.startswith('#') or not line.strip():
                continue
            v = [float(x) for x in line.split()]; ts.append(v[0]); xs.append(v[1]); ys.append(v[2]); yaws.append(quat_to_yaw(v[4], v[5], v[6], v[7]))
        order = sorted(range(len(ts)), key=lambda i: ts[i]); ts, xs, ys, yaws = [[l[i] for i in order] for l in (ts, xs, ys, yaws)]
    print('%d pose samples (%s), %d lift samples' % (len(ts), src, len(lt)))
    rows = [r for r in csv.DictReader(l for l in open(a.csv) if not l.startswith('#'))]
    out = a.out or a.csv.replace('.csv', '_posed.csv'); n_ok = 0
    with open(out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(list(rows[0].keys()) + ['base_x', 'base_y', 'base_yaw', 'lift_m', 'ant_x', 'ant_y', 'ant_z', 'bore_x', 'bore_y', 'pose_src', 'pose_dt_s'])
        for r in rows:
            t = float(r['host_time_s']); bx, by, yaw, dt = interp_pose(ts, xs, ys, yaws, t)
            lift = interp_lift(lt, lv, t)
            o = port_off.get(str(r.get('antenna', '')).strip(), off)
            c, s = math.cos(yaw), math.sin(yaw)
            ax = bx + o[0] * c - o[1] * s; ay = by + o[0] * s + o[1] * c; az = o[2] + lift
            ok = abs(dt) <= a.max_dt; n_ok += ok
            w.writerow(list(r.values()) + ['%.4f' % bx, '%.4f' % by, '%.4f' % yaw, '%.4f' % lift, '%.4f' % ax, '%.4f' % ay, '%.4f' % az, '%.4f' % -s, '%.4f' % c, src, '%.3f' % dt])
    print('wrote %s: %d reads, %d within %.2f s of a pose sample' % (out, len(rows), n_ok, a.max_dt))


if __name__ == '__main__':
    main()
