#!/usr/bin/env python3
"""
Compare goto_joints(dynamic=True) vs goto_joints(dynamic=False) on identical
random trajectories constrained to the straight-down EE manifold.

Both modes execute the exact same pre-generated waypoint sequences.
Results are printed in a side-by-side table and a per-segment comparison so
it is easy to see where dynamic streaming diverges from the static baseline.

Usage:
    python test_dynamic_vs_static.py                          # default (unconstrained q_dist)
    python test_dynamic_vs_static.py --min-q-dist 2.5        # force large inter-waypoint jumps
    python test_dynamic_vs_static.py --min-q-dist 3.5        # stress test: very large jumps
    python test_dynamic_vs_static.py --arm 2 --num-traj 5 --num-waypoints 4
    python test_dynamic_vs_static.py --seg-duration 6.0 --seed 7
"""

import argparse
import sys
import time
import numpy as np

sys.path.insert(0, "/home/ros_ws/src/devel_packages/prm")
import SimpleFranka
import RobotUtil as rt

import rospy
from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

# ── Constants ─────────────────────────────────────────────────────────────────

# "Straight down" in the real robot frame (O_T_EE / fa.get_pose()).
R_DESIRED = np.array([[ 1,  0,  0],
                      [ 0, -1,  0],
                      [ 0,  0, -1]], dtype=float)

# IK/DLS target in SimpleFranka (panda_link8) frame.
# O_T_EE = O_T_link8 @ Rz(-pi/4), so for R_real = R_DESIRED:
#   R_SF_target = R_DESIRED @ Rz(+pi/4)
_c45, _s45 = np.cos(np.pi / 4), np.sin(np.pi / 4)
R_DESIRED_SF = R_DESIRED @ np.array([[ _c45, -_s45, 0],
                                      [ _s45,  _c45, 0],
                                      [    0,     0, 1]], dtype=float)

# Workspace bounds from acc_repeat.py
X_MIN, X_MAX = 0.2, 0.7
Y_MIN, Y_MAX = -0.6, 0.4
Z_MIN, Z_MAX = 0.15, 0.6

# Sampling bounds: FC.JOINT_LIMITS pulled 0.05 rad inward on each side so
# sampled configs always satisfy the strict-inequality fa.is_joints_reachable().
# Joint 1 further capped at ±1.52 to keep the arm in front of the robot.
_MARGIN = 0.05
_FC_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_FC_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
Q_MIN   = np.maximum(_FC_MIN + _MARGIN, np.array([-1.52, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]))
Q_MAX   = np.minimum(_FC_MAX - _MARGIN, np.array([ 1.52,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf]))

STREAM_RATE_HZ = 1000.0

ARM_SENSOR_TOPIC = {
    1: "/franka_ros_interface/sensor",
    2: "/franka_ros_interface_2/sensor",
}

# Impedance gains used for dynamic mode (same as franka_prm_single_arm)
K_GAINS = [600, 600, 600, 600, 600, 300, 200]
D_GAINS = [ 50,  50,  50,  50,  30,  25,  15]

# ── IK ────────────────────────────────────────────────────────────────────────

def _project_to_vertical_down(cc, q, lam=0.01, max_iter=200, r_eps=1e-3):
    q = np.array(q, dtype=float)
    for _ in range(max_iter):
        Tcurr, J = cc.ForwardKin(q)
        R_err = R_DESIRED_SF @ np.array(Tcurr[-1][:3, :3]).T
        axis, ang = rt.R2axisang(R_err)
        r_err = np.array(axis) * ang
        if np.linalg.norm(r_err) < r_eps:
            break
        if abs(ang) > 0.1:
            r_err = np.array(axis) * 0.1
        Jo = J[3:6, :]
        dq = Jo.T @ np.linalg.inv(Jo @ Jo.T + lam**2 * np.eye(3)) @ r_err
        q  = np.clip(q + dq, Q_MIN, Q_MAX)
    return q


def sample_manifold(cc, rng, max_attempts=200, ori_tol_rad=np.radians(3)):
    """Sample one valid joint config on the vertical-down manifold via joint-space sampling.

    Samples q uniformly in joint space, DLS-projects onto the constraint manifold,
    then checks workspace bounds and orientation tolerance.  This is O(1) and
    guaranteed to produce a config on the manifold — the same method used by
    PRMGenerator_DLS_JPI.  No IK required.

    Returns (q, ee_pos) or None if max_attempts exhausted.
    """
    for _ in range(max_attempts):
        q_rand = rng.uniform(Q_MIN, Q_MAX)
        q_proj = _project_to_vertical_down(cc, q_rand)
        Tcurr, _ = cc.ForwardKin(q_proj)
        ee_pos = Tcurr[-1][:3, 3].copy()

        # Workspace filter: EE must be in front of the robot and at a safe height
        if ee_pos[0] < X_MIN or ee_pos[0] > X_MAX:
            continue
        if ee_pos[1] < Y_MIN or ee_pos[1] > Y_MAX:
            continue
        if ee_pos[2] < Z_MIN or ee_pos[2] > Z_MAX:
            continue

        # Orientation check: DLS may not have fully converged
        R_err = R_DESIRED_SF @ np.array(Tcurr[-1][:3, :3]).T
        _, ang = rt.R2axisang(R_err)
        if abs(ang) > ori_tol_rad:
            continue

        return q_proj, ee_pos

    return None


def generate_trajectories(cc, num_traj, num_waypoints, rng,
                          min_q_dist=0.0, max_wp_attempts=500):
    """Pre-generate all trajectories by sampling directly in joint space.

    Samples random configs on the vertical-down manifold via DLS projection
    (same as PRMGenerator).  This is O(1) per waypoint and handles min_q_dist
    efficiently without Cartesian IK retries.

    Args:
        min_q_dist : minimum joint-space L2 distance (rad) between consecutive
                     waypoints.  The home config is the predecessor of wp0.
    """
    q_home = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    trajectories = []
    for ti in range(num_traj):
        print(f"  Trajectory {ti+1}/{num_traj}:")
        waypoints = []
        q_prev = q_home
        for wi in range(num_waypoints):
            found = False
            for attempt in range(max_wp_attempts):
                result = sample_manifold(cc, rng)
                if result is None:
                    continue
                q, ee_pos = result
                q_dist = np.linalg.norm(q - q_prev)
                if q_dist < min_q_dist:
                    continue
                waypoints.append((q, ee_pos))
                print(f"    wp{wi+1}: ee_pos={np.round(ee_pos,3)}  "
                      f"q_dist={q_dist:.3f}rad  q={np.round(q,3)}")
                q_prev = q
                found = True
                break
            if not found:
                raise RuntimeError(
                    f"Could not find waypoint for traj {ti+1} wp {wi+1} "
                    f"after {max_wp_attempts} attempts "
                    f"(min_q_dist={min_q_dist:.2f} rad may be too large for workspace)")
        trajectories.append(waypoints)
    return trajectories

# ── Trajectory building ───────────────────────────────────────────────────────

def _minjerk(q0, qf, n_pts):
    t = np.linspace(0, 1, n_pts + 1)[1:]
    w = 10*t**3 - 15*t**4 + 6*t**5
    return q0 + np.outer(w, qf - q0)


def build_traj(q_start, q_end, duration_s):
    n = max(2, round(duration_s * STREAM_RATE_HZ))
    traj  = _minjerk(np.asarray(q_start), np.asarray(q_end), n)
    dwell = np.tile(traj[-1], (0.5 * n, 1))
    return np.vstack([traj, dwell])

# ── Execution ─────────────────────────────────────────────────────────────────

def run_static_segment(fa, q_target, duration_s):
    """Blocking MinJerk goto_joints."""
    fa.goto_joints(q_target.tolist(), duration=duration_s)


def run_dynamic_segment(fa, pub, q_start, q_target, duration_s, buffer_time=30.0):
    """PassThrough streaming goto_joints."""
    traj  = build_traj(q_start, q_target, duration_s)
    n_pts = traj.shape[0]

    fa.goto_joints(traj[0].tolist(),
                   duration=duration_s + buffer_time,
                   dynamic=True, buffer_time=buffer_time,
                   k_gains=K_GAINS, d_gains=D_GAINS)

    init_time = rospy.Time.now().to_time()
    rate      = rospy.Rate(STREAM_RATE_HZ)
    late      = 0
    dt        = 1.0 / STREAM_RATE_HZ

    for i in range(1, n_pts):
        t0 = rospy.Time.now().to_time()
        pub.publish(make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                JointPositionSensorMessage(id=i,
                                           timestamp=t0 - init_time,
                                           joints=traj[i].tolist()),
                SensorDataMessageType.JOINT_POSITION)))
        rate.sleep()
        if rospy.Time.now().to_time() - t0 > dt * 1.5:
            late += 1

    pub.publish(make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            ShouldTerminateSensorMessage(
                timestamp=rospy.Time.now().to_time() - init_time,
                should_terminate=True),
            SensorDataMessageType.SHOULD_TERMINATE)))
    fa.wait_for_skill()
    return late

# ── Metrics ───────────────────────────────────────────────────────────────────

def measure(fa, q_target, task_pos):
    """Read robot state and return (q_err, pos_err_m, tilt_deg)."""
    q_actual = np.array(fa.get_joints())
    q_err    = np.linalg.norm(q_actual - q_target)

    pose     = fa.get_pose()
    R_curr   = pose.rotation
    R_err    = R_DESIRED @ R_curr.T
    _, ang   = rt.R2axisang(R_err)
    tilt_deg = np.degrees(abs(ang))

    pos_err  = np.linalg.norm(pose.translation - task_pos)
    return q_err, pos_err, tilt_deg

# ── Run one mode over all trajectories ───────────────────────────────────────

def run_mode(fa, pub, cc, trajectories, seg_duration, mode):
    """Execute all trajectories in the given mode ('static' or 'dynamic').

    Returns a list-of-lists of per-segment dicts.
    """
    assert mode in ('static', 'dynamic')
    all_results = []

    for ti, waypoints in enumerate(trajectories):
        print(f"\n  [{mode}] Trajectory {ti+1}/{len(trajectories)}")
        fa.reset_joints()
        q_current = np.array(fa.get_joints())
        seg_results = []

        for wi, (q_target, task_pos) in enumerate(waypoints):
            # Joint-space distance from current position to target
            q_dist = np.linalg.norm(q_target - q_current)

            t0   = time.time()
            late = 0

            if mode == 'static':
                run_static_segment(fa, q_target, seg_duration)
            else:
                # Always pre-position statically before dynamic streaming.
                # Read actual position after static move as the true start,
                # then stream the dense min-jerk path from there to q_target.
                run_static_segment(fa, q_target, seg_duration)
                q_actual_start = np.array(fa.get_joints())
                late = run_dynamic_segment(fa, pub, q_actual_start,
                                           q_target, seg_duration)

            elapsed  = time.time() - t0
            q_err, pos_err, tilt_deg = measure(fa, q_target, task_pos)
            q_current = np.array(fa.get_joints())

            method = "static" if mode == 'static' else "static→dynamic"
            passed = q_err < 0.05 and tilt_deg < 5.0

            print(f"    seg{wi+1}  method={method}  q_dist={q_dist:.3f}rad  "
                  f"q_err={q_err:.4f}rad  pos_err={pos_err:.4f}m  "
                  f"tilt={tilt_deg:.2f}°  {'PASS' if passed else 'FAIL'}"
                  + (f"  [{late} late]" if late else ""))

            seg_results.append(dict(
                traj=ti, seg=wi,
                q_target=q_target, task_pos=task_pos,
                q_dist=q_dist,
                q_err=q_err, pos_err=pos_err, tilt_deg=tilt_deg,
                passed=passed, elapsed=elapsed, method=method,
            ))

        all_results.append(seg_results)

    return all_results

# ── Comparison summary ────────────────────────────────────────────────────────

def print_comparison(static_results, dynamic_results, tol_deg=5.0, tol_q=0.05):
    segs_s = [s for tr in static_results  for s in tr]
    segs_d = [s for tr in dynamic_results for s in tr]
    assert len(segs_s) == len(segs_d)

    # Dynamic mode now does static→dynamic on every segment, so all segments
    # are comparable. The q_err difference isolates the dynamic streaming step.
    dynamic_segs = list(zip(segs_s, segs_d))

    print(f"\n{'='*80}")
    print("SEGMENT-BY-SEGMENT COMPARISON  (static vs static→dynamic)")
    print(f"{'='*80}")
    print(f"  {'T-S':4}  {'q_dist':>8}  "
          f"{'q_err_S':>9}  {'q_err_D':>9}  "
          f"{'tilt_S':>8}  {'tilt_D':>8}  "
          f"{'pos_S':>8}  {'pos_D':>8}  result")
    print(f"  {'----':4}  {'--------':>8}  "
          f"{'-------':>9}  {'-------':>9}  "
          f"{'------':>8}  {'------':>8}  "
          f"{'-----':>8}  {'-----':>8}  ------")

    regressions = 0
    for s, d in dynamic_segs:
        label = f"{s['traj']+1}-{s['seg']+1}"
        s_ok  = s['q_err'] < tol_q and s['tilt_deg'] < tol_deg
        d_ok  = d['q_err'] < tol_q and d['tilt_deg'] < tol_deg
        if s_ok and not d_ok:
            result = "REGRESSION"
            regressions += 1
        elif not s_ok and d_ok:
            result = "improvement"
        elif s_ok and d_ok:
            result = "both pass"
        else:
            result = "both fail"
        print(f"  {label:4}  {s['q_dist']:8.3f}  "
              f"{s['q_err']:9.4f}  {d['q_err']:9.4f}  "
              f"{s['tilt_deg']:8.2f}  {d['tilt_deg']:8.2f}  "
              f"{s['pos_err']:8.4f}  {d['pos_err']:8.4f}  {result}")

    n = len(dynamic_segs)
    print(f"\n  Total segments: {n}  |  regressions (static pass, dynamic fail): {regressions}")

    print(f"\n{'='*80}")
    print("AGGREGATE STATISTICS  (static vs static→dynamic, all segments)")
    print(f"{'='*80}")

    def _stats(segs, key):
        vals = [s[key] for s in segs]
        return np.mean(vals), np.max(vals)

    for label, segs_pair in [("ALL segments", dynamic_segs)]:
        ss = [p[0] for p in segs_pair]
        ds = [p[1] for p in segs_pair]
        n  = len(ss)
        ss = [p[0] for p in segs_pair]
        ds = [p[1] for p in segs_pair]
        n  = len(ss)
        s_pass = sum(1 for s in ss if s['passed'])
        d_pass = sum(1 for d in ds if d['passed'])

        s_q_mean, s_q_max = _stats(ss, 'q_err')
        d_q_mean, d_q_max = _stats(ds, 'q_err')
        s_t_mean, s_t_max = _stats(ss, 'tilt_deg')
        d_t_mean, d_t_max = _stats(ds, 'tilt_deg')
        s_p_mean, s_p_max = _stats(ss, 'pos_err')
        d_p_mean, d_p_max = _stats(ds, 'pos_err')

        print(f"\n  {label}  (n={n})")
        print(f"  {'':25}  {'static':>10}  {'dynamic':>10}")
        print(f"  {'pass rate':25}  {s_pass}/{n}{'':6}  {d_pass}/{n}")
        print(f"  {'q_err mean / max (rad)':25}  "
              f"{s_q_mean:6.4f}/{s_q_max:6.4f}  "
              f"{d_q_mean:6.4f}/{d_q_max:6.4f}")
        print(f"  {'EE tilt mean / max (°)':25}  "
              f"{s_t_mean:6.2f}/{s_t_max:6.2f}  "
              f"{d_t_mean:6.2f}/{d_t_max:6.2f}")
        print(f"  {'pos_err mean / max (m)':25}  "
              f"{s_p_mean:6.4f}/{s_p_max:6.4f}  "
              f"{d_p_mean:6.4f}/{d_p_max:6.4f}")

    print(f"\n  Pass criterion: q_err < {tol_q} rad  AND  EE_tilt < {tol_deg}°")
    print(f"{'='*80}")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compare goto_joints dynamic=True vs dynamic=False")
    parser.add_argument("--arm",           type=int,   default=1)
    parser.add_argument("--num-traj",      type=int,   default=3)
    parser.add_argument("--num-waypoints", type=int,   default=3)
    parser.add_argument("--seg-duration",  type=float, default=5.0)
    parser.add_argument("--seed",          type=int,   default=51)
    parser.add_argument("--min-q-dist",    type=float, default=0.0,
                        help="Minimum joint-space L2 distance (rad) between "
                             "consecutive waypoints. Use e.g. 2.5 to force "
                             "large motions that stress the dynamic controller.")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    print("=" * 70)
    print("dynamic=True vs dynamic=False comparison")
    print(f"  arm={args.arm}  num_traj={args.num_traj}  "
          f"num_waypoints={args.num_waypoints}  "
          f"seg_duration={args.seg_duration}s  seed={args.seed}  "
          f"min_q_dist={args.min_q_dist:.2f}rad")
    print("=" * 70)

    cc = SimpleFranka.SimpleFrankArm(arm_number=args.arm)

    print("\nGenerating trajectories (offline)…")
    trajectories = generate_trajectories(
        cc, args.num_traj, args.num_waypoints, rng,
        min_q_dist=args.min_q_dist)

    rospy.init_node("test_dynamic_vs_static", anonymous=True)
    fa  = FrankaArm(with_gripper=False, old_gripper=False,
                    robot_num=args.arm, init_node=False)
    pub = rospy.Publisher(ARM_SENSOR_TOPIC[args.arm],
                          SensorDataGroup, queue_size=1000)

    # ── Static pass ───────────────────────────────────────────────────────────
    print(f"\n{'─'*70}")
    print("PASS 1 — static (dynamic=False, blocking MinJerk)")
    print(f"{'─'*70}")
    static_results = run_mode(fa, pub, cc, trajectories, args.seg_duration, 'static')

    # ── Dynamic pass ──────────────────────────────────────────────────────────
    print(f"\n{'─'*70}")
    print("PASS 2 — dynamic (dynamic=True, PassThrough streaming)")
    print(f"{'─'*70}")
    dynamic_results = run_mode(fa, pub, cc, trajectories, args.seg_duration, 'dynamic')

    # ── Comparison ────────────────────────────────────────────────────────────
    print_comparison(static_results, dynamic_results)

    fa.reset_joints()


if __name__ == "__main__":
    main()
