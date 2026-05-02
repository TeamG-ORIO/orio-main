#!/usr/bin/env python3
"""
Diagnostic script to isolate the source of EE orientation drift seen in
test_goto_joints_dynamic.py (mean tilt 13.7°, max 36.9°).

Three independent tests — run them in order and read the output:

  TEST A — FK consistency
      Does SimpleFranka.ForwardKin(q) agree with fa.get_pose() at the
      same q?  If they disagree, the tilt we're reporting is an artifact
      of the wrong FK model, not a real robot error.

  TEST B — Static goto_joints (non-dynamic, blocking)
      Command two "hard" target configs (large joint moves, low z) with
      plain goto_joints (MinJerk, no streaming).  Compare q_actual vs
      q_target and EE tilt from BOTH FK sources.  If q_err is small here
      but was large in dynamic mode → streaming/gains are the problem.

  TEST C — Dynamic streaming gain sweep
      Repeat the same two targets with goto_joints(dynamic=True) using
      three different k_gain/d_gain sets.  Compare q_err across sets to
      see whether the impedance gains are causing the robot to not track.

Usage:
    python debug_orientation_drift.py            # all tests, arm 1
    python debug_orientation_drift.py --arm 2
    python debug_orientation_drift.py --tests A B   # only specific tests
    python debug_orientation_drift.py --tests C     # gain sweep only
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

# ── Shared constants ──────────────────────────────────────────────────────────

R_DESIRED = np.array([[ 1,  0,  0],
                       [ 0, -1,  0],
                       [ 0,  0, -1]], dtype=float)

ARM_SENSOR_TOPIC = {
    1: "/franka_ros_interface/sensor",
    2: "/franka_ros_interface_2/sensor",
}

STREAM_RATE_HZ = 1000.0

# Two representative "hard" target configs from the failing trajectories.
# These produced q_err > 0.3 rad in the dynamic test.
# Format: (label, q_target_7dof)
HARD_TARGETS = [
    ("low-z forward-reach",
     np.array([ 0.462, -0.535, -0.405, -2.853, -0.282,  2.334,  0.306])),
    ("high-tilt wrist",
     np.array([ 0.877, -1.155, -0.561, -1.906, -0.654,  0.929,  0.199])),
    ("near-singularity",
     np.array([-0.163,  0.311, -0.000, -0.486,  0.000,  0.797, -0.163])),
]

# Gain sets for TEST C
GAIN_SETS = [
    ("default (franka internal)",
     None, None),                                               # use_impedance=False
    ("soft impedance",
     [250, 250, 250, 250, 150,  50,  50],
     [ 25,  25,  25,  25,  15,   5,   5]),
    ("medium impedance (prm executor)",
     [600, 600, 600, 600, 600, 300, 200],
     [ 50,  50,  50,  50,  30,  25,  15]),
    ("stiff impedance",
     [800, 800, 800, 800, 800, 500, 300],
     [ 60,  60,  60,  60,  40,  30,  20]),
]

# ── Helpers ───────────────────────────────────────────────────────────────────

def _sep(title=""):
    bar = "─" * 70
    if title:
        print(f"\n{bar}\n  {title}\n{bar}")
    else:
        print(bar)


def _minjerk(q0, qf, n_pts):
    t = np.linspace(0.0, 1.0, n_pts + 1)[1:]
    w = 10*t**3 - 15*t**4 + 6*t**5
    return q0 + np.outer(w, qf - q0)


def _build_traj(q_start, q_end, duration_s, rate_hz=STREAM_RATE_HZ):
    n_move = max(2, round(duration_s * rate_hz))
    traj   = _minjerk(np.asarray(q_start), np.asarray(q_end), n_move)
    dwell  = np.tile(traj[-1], (2 * n_move, 1))
    return np.vstack([traj, dwell])


def _simplefk_orientation_error(cc, q):
    """Return EE tilt in degrees from R_DESIRED using SimpleFranka FK."""
    Tcurr, _ = cc.ForwardKin(q)
    R_curr = np.array(Tcurr[-1][:3, :3])
    R_err  = R_DESIRED @ R_curr.T
    _, ang = rt.R2axisang(R_err)
    return np.degrees(abs(ang)), Tcurr[-1][:3, 3].copy(), R_curr.copy()


def _fa_orientation_error(fa):
    """Return EE tilt in degrees from R_DESIRED using fa.get_pose() (libfranka FK)."""
    pose = fa.get_pose()                              # autolab_core.RigidTransform
    R_fa = pose.rotation                              # 3×3 ndarray, world ← tool
    R_err = R_DESIRED @ R_fa.T
    _, ang = rt.R2axisang(R_err)
    return np.degrees(abs(ang)), pose.translation.copy(), R_fa.copy()


def _stream_dynamic(fa, pub, q_start, q_end, duration_s,
                    k_gains=None, d_gains=None, buffer_time=30.0):
    """Execute one segment via goto_joints(dynamic=True) streaming."""
    use_impedance_arg = k_gains is not None

    traj  = _build_traj(q_start, q_end, duration_s)
    n_pts = traj.shape[0]

    if use_impedance_arg:
        fa.goto_joints(traj[0].tolist(),
                       duration=duration_s + buffer_time,
                       dynamic=True, buffer_time=buffer_time,
                       k_gains=k_gains, d_gains=d_gains)
    else:
        # No k/d gains → franka internal controller via use_impedance=False
        # dynamic=True forces use_impedance=True internally, so we pass
        # explicit gains to keep it comparable; use softest available.
        fa.goto_joints(traj[0].tolist(),
                       duration=duration_s + buffer_time,
                       dynamic=True, buffer_time=buffer_time,
                       k_gains=[250, 250, 250, 250, 150, 50, 50],
                       d_gains=[ 25,  25,  25,  25,  15,  5,  5])

    init_time = rospy.Time.now().to_time()
    rate = rospy.Rate(STREAM_RATE_HZ)

    for i in range(1, n_pts):
        t_now = rospy.Time.now().to_time()
        pub.publish(make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                JointPositionSensorMessage(
                    id=i,
                    timestamp=t_now - init_time,
                    joints=traj[i].tolist(),
                ), SensorDataMessageType.JOINT_POSITION)
        ))
        rate.sleep()

    pub.publish(make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            ShouldTerminateSensorMessage(
                timestamp=rospy.Time.now().to_time() - init_time,
                should_terminate=True,
            ), SensorDataMessageType.SHOULD_TERMINATE)
    ))
    fa.wait_for_skill()


# ── TEST A — FK consistency ───────────────────────────────────────────────────

def test_a_fk_consistency(fa, cc):
    """
    Move to each HARD_TARGET with plain goto_joints (non-dynamic, blocking),
    then compare:
      - SimpleFranka.ForwardKin(q_actual) rotation
      - fa.get_pose() rotation (libfranka)
    at the same encoder-read q_actual.

    If the two rotations agree → FK models are consistent; drift is real.
    If they disagree → one model is wrong and tilt numbers are unreliable.
    """
    _sep("TEST A — FK model consistency (SimpleFranka vs libfranka/fa.get_pose)")
    print("  Moves to each target with plain goto_joints (non-dynamic).")
    print("  Reads q_actual from encoders, computes FK two ways, compares.\n")

    for label, q_target in HARD_TARGETS:
        print(f"  Target: {label}")
        print(f"    q_target = {np.round(q_target, 4)}")

        fa.reset_joints()
        fa.goto_joints(q_target.tolist(), duration=5.0)

        q_actual = np.array(fa.get_joints())
        q_err    = np.linalg.norm(q_actual - q_target)
        print(f"    q_actual = {np.round(q_actual, 4)}")
        print(f"    q_err (norm) = {q_err:.4f} rad")

        # SimpleFranka FK at q_actual
        sf_tilt, sf_pos, sf_R = _simplefk_orientation_error(cc, q_actual)
        # libfranka FK at q_actual (real robot state)
        fa_tilt, fa_pos, fa_R = _fa_orientation_error(fa)

        print(f"\n    SimpleFranka FK:")
        print(f"      EE pos   = {np.round(sf_pos, 4)}")
        print(f"      EE R     =\n{np.round(sf_R, 4)}")
        print(f"      tilt from R_DESIRED = {sf_tilt:.2f}°")

        print(f"\n    libfranka get_pose FK:")
        print(f"      EE pos   = {np.round(fa_pos, 4)}")
        print(f"      EE R     =\n{np.round(fa_R, 4)}")
        print(f"      tilt from R_DESIRED = {fa_tilt:.2f}°")

        R_diff = sf_R @ fa_R.T
        _, ang_diff = rt.R2axisang(R_diff)
        print(f"\n    Rotation difference between FK models: {np.degrees(ang_diff):.2f}°")
        print(f"    {'[OK] FK models agree' if np.degrees(ang_diff) < 3.0 else '[MISMATCH] FK models differ — SimpleFranka reports wrong orientation'}")
        print()

    fa.reset_joints()


# ── TEST B — Static goto_joints baseline ─────────────────────────────────────

def test_b_static_goto_joints(fa, cc):
    """
    Command each HARD_TARGET with plain (non-dynamic) goto_joints and verify
    joint tracking.  This isolates whether the robot CAN physically reach these
    configs — if q_err is small here but large in dynamic mode, the streaming
    or gains are the problem, not the targets themselves.
    """
    _sep("TEST B — Static goto_joints baseline (non-dynamic, blocking)")
    print("  Uses MinJerk trajectory generator (no streaming).")
    print("  If q_err is small here but was large in dynamic mode → streaming/gains are the issue.\n")

    rows = []
    for label, q_target in HARD_TARGETS:
        print(f"  Target: {label}")
        print(f"    q_target = {np.round(q_target, 4)}")
        fa.reset_joints()

        t0 = time.time()
        fa.goto_joints(q_target.tolist(), duration=5.0)
        elapsed = time.time() - t0

        q_actual = np.array(fa.get_joints())
        q_err    = np.linalg.norm(q_actual - q_target)
        sf_tilt, sf_pos, _ = _simplefk_orientation_error(cc, q_actual)
        fa_tilt, fa_pos, _ = _fa_orientation_error(fa)

        status = "PASS" if q_err < 0.05 else "FAIL"
        print(f"    [{status}] q_err={q_err:.4f} rad  elapsed={elapsed:.1f}s")
        print(f"    SimpleFranka tilt={sf_tilt:.2f}°   libfranka tilt={fa_tilt:.2f}°")
        print(f"    SimpleFranka EE pos={np.round(sf_pos,4)}")
        print(f"    libfranka    EE pos={np.round(fa_pos,4)}")
        rows.append((label, q_err, sf_tilt, fa_tilt))
        print()

    print("  Summary:")
    for label, q_err, sf_tilt, fa_tilt in rows:
        print(f"    {label:35s}  q_err={q_err:.4f}  SF_tilt={sf_tilt:.1f}°  FA_tilt={fa_tilt:.1f}°")

    fa.reset_joints()


# ── TEST C — Dynamic gain sweep ───────────────────────────────────────────────

def test_c_dynamic_gain_sweep(fa, cc, pub):
    """
    Repeat each HARD_TARGET using goto_joints(dynamic=True) with four
    different gain configurations.  All other parameters are identical.

    This directly shows whether the impedance gains are the cause of the
    large q_err seen in test_goto_joints_dynamic.py.
    """
    _sep("TEST C — Dynamic streaming gain sweep")
    print("  Same targets as TEST B, executed via goto_joints(dynamic=True).")
    print("  If a gain set fixes q_err, that is the right set to use.\n")

    # Table header
    col_w = 38
    print(f"  {'Target':{col_w}}  {'Gain set':{col_w}}  q_err(rad)  SF_tilt(°)  FA_tilt(°)")
    print(f"  {'-'*col_w}  {'-'*col_w}  ----------  ----------  ----------")

    for label, q_target in HARD_TARGETS:
        for gain_label, k_gains, d_gains in GAIN_SETS:
            fa.reset_joints()
            q_start = np.array(fa.get_joints())

            _stream_dynamic(fa, pub, q_start, q_target,
                            duration_s=5.0,
                            k_gains=k_gains, d_gains=d_gains)

            q_actual = np.array(fa.get_joints())
            q_err    = np.linalg.norm(q_actual - q_target)
            sf_tilt, _, _ = _simplefk_orientation_error(cc, q_actual)
            fa_tilt, _, _ = _fa_orientation_error(fa)

            flag = "  <-- best?" if q_err < 0.05 else ""
            print(f"  {label:{col_w}}  {gain_label:{col_w}}  "
                  f"{q_err:10.4f}  {sf_tilt:10.2f}  {fa_tilt:10.2f}{flag}")

        print()

    fa.reset_joints()


# ── TEST D — IK self-consistency ──────────────────────────────────────────────

def test_d_ik_selfcheck(cc):
    """
    Offline only (no robot movement).  For each HARD_TARGET, verify:
      1. SimpleFranka.ForwardKin(q_target) gives R ≈ R_DESIRED
      2. The IK that produced q_target is self-consistent

    This rules out SimpleFranka FK bugs as the cause of the reported tilt.
    """
    _sep("TEST D — IK / FK self-consistency (offline, no robot movement)")
    print("  Checks SimpleFranka FK at each target config.")
    print("  If tilt > 5° here, the target was generated with a broken IK.\n")

    for label, q_target in HARD_TARGETS:
        sf_tilt, sf_pos, sf_R = _simplefk_orientation_error(cc, q_target)

        # Also compute what the joint limits allow
        Q_MIN = np.array([-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        Q_MAX = np.array([ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
        in_limits = np.all(q_target >= Q_MIN) and np.all(q_target <= Q_MAX)

        print(f"  Target: {label}")
        print(f"    q_target    = {np.round(q_target, 4)}")
        print(f"    within limits: {in_limits}")
        print(f"    SimpleFranka FK:")
        print(f"      EE pos = {np.round(sf_pos, 4)}")
        print(f"      EE R   =\n{np.round(sf_R, 4)}")
        print(f"      tilt from R_DESIRED = {sf_tilt:.4f}°")
        status = "OK — target is valid" if sf_tilt < 5.0 else "PROBLEM — IK produced a non-vertical target"
        print(f"      [{status}]")
        print()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Orientation drift diagnostics")
    parser.add_argument("--arm",   type=int, default=1, choices=[1, 2])
    parser.add_argument("--tests", nargs="+", default=["A", "B", "C", "D"],
                        choices=["A", "B", "C", "D"],
                        help="Which tests to run (default: all)")
    args = parser.parse_args()

    print("=" * 70)
    print(f"Orientation drift diagnostics  —  arm {args.arm}")
    print(f"Tests to run: {args.tests}")
    print("=" * 70)

    cc = SimpleFranka.SimpleFrankArm(arm_number=args.arm)

    # TEST D is offline — run it first, always
    if "D" in args.tests:
        test_d_ik_selfcheck(cc)

    needs_robot = any(t in args.tests for t in ["A", "B", "C"])
    if not needs_robot:
        return

    rospy.init_node("debug_orientation_drift", anonymous=True)
    fa  = FrankaArm(with_gripper=False, old_gripper=False,
                    robot_num=args.arm, init_node=False)
    pub = rospy.Publisher(ARM_SENSOR_TOPIC[args.arm],
                          SensorDataGroup, queue_size=1000)

    print(f"\nRobot connected. Running tests: {args.tests}\n")

    if "A" in args.tests:
        test_a_fk_consistency(fa, cc)

    if "B" in args.tests:
        test_b_static_goto_joints(fa, cc)

    if "C" in args.tests:
        test_c_dynamic_gain_sweep(fa, cc, pub)

    print("\nAll done.")


if __name__ == "__main__":
    main()
