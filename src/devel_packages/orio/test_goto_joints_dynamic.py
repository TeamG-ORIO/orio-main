#!/usr/bin/env python3
"""
Unit test for goto_joints with dynamic=True.

Generates num_traj random trajectories, each with num_waypoints random Cartesian
positions (within the workspace bounds from acc_repeat.py).  Each position is
solved with IK constrained to EE rotation = diag(1,-1,-1) (straight down).
Waypoints are executed sequentially using the dynamic streaming pattern:
  goto_joints(dynamic=True) opens the skill, ROS messages stream joint positions
  at 1 kHz, a termination message closes each segment, wait_for_skill() blocks.

Usage:
    python test_goto_joints_dynamic.py
    python test_goto_joints_dynamic.py --num-traj 3 --num-waypoints 4 --arm 1
    python test_goto_joints_dynamic.py --seg-duration 4.0 --seed 0
"""

import argparse
import sys
import time
import numpy as np

# ── local imports (must be on PYTHONPATH) ─────────────────────────────────────
sys.path.insert(0, "/home/ros_ws/src/devel_packages/prm")
import SimpleFranka
import RobotUtil as rt

# ── ROS / frankapy ────────────────────────────────────────────────────────────
import rospy
from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

# ── Constants ─────────────────────────────────────────────────────────────────

# "Straight down" EE orientation in the real robot frame (O_T_EE / libfranka).
# fa.get_pose() reports this when the gripper z-axis points into the table.
# Confirmed by FC.HOME_POSE.rotation and debug_orientation_drift TEST A.
R_DESIRED = np.array([
    [ 1,  0,  0],
    [ 0, -1,  0],
    [ 0,  0, -1],
], dtype=float)

# libfranka O_T_EE = O_T_link8 @ panda_hand_joint, where panda_hand_joint
# applies Rz(-pi/4).  SimpleFranka.ForwardKin stops at panda_link8, so:
#   R_real = R_SF @ Rz(-pi/4)
# For R_real = R_DESIRED we need:
#   R_SF_target = R_DESIRED @ Rz(+pi/4)
_c45, _s45 = np.cos(np.pi / 4), np.sin(np.pi / 4)
_Rz_p45 = np.array([[ _c45, -_s45, 0],
                     [ _s45,  _c45, 0],
                     [    0,     0, 1]], dtype=float)
# Target rotation for SimpleFranka IK / DLS projection so the real EE is vertical.
R_DESIRED_SF = R_DESIRED @ _Rz_p45

# Workspace bounds taken from acc_repeat.py
X_MIN, X_MAX = 0.2, 0.8
Y_MIN, Y_MAX = -0.3, 0.3
Z_MIN, Z_MAX = 0.15, 0.75

# Joint limits (from SimpleFranka / PRM generator — slightly tighter on joint 1)
Q_MIN = np.array([-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

STREAM_RATE_HZ = 1000.0   # Hz — matches franka_prm_single_arm.py
SENSOR_TOPIC   = "/franka_ros_interface/sensor"  # arm 1 default

ARM_SENSOR_TOPIC = {
    1: "/franka_ros_interface/sensor",
    2: "/franka_ros_interface_2/sensor",
}

# ── IK helpers ────────────────────────────────────────────────────────────────

def _project_to_vertical_down(cc, q, lam=0.01, max_iter=200, r_eps=1e-3):
    """DLS projection onto the gripper-vertical-down manifold in SimpleFranka space.

    Targets R_DESIRED_SF = R_DESIRED @ Rz(+pi/4) so that after the real robot's
    panda_hand_joint (Rz(-pi/4)), O_T_EE equals R_DESIRED (gripper straight down).
    Only orientation rows of J are used — position drifts freely.
    """
    q = np.array(q, dtype=float)
    for _ in range(max_iter):
        Tcurr, J = cc.ForwardKin(q)
        R_curr = np.array(Tcurr[-1][:3, :3])
        R_err  = R_DESIRED_SF @ R_curr.T
        axis, ang = rt.R2axisang(R_err)
        r_err = np.array(axis) * ang
        if np.linalg.norm(r_err) < r_eps:
            break
        if abs(ang) > 0.1:
            r_err = np.array(axis) * 0.1
        Jo  = J[3:6, :]
        A   = Jo @ Jo.T + (lam ** 2) * np.eye(3)
        dq  = Jo.T @ np.linalg.inv(A) @ r_err
        q   = np.clip(q + dq, Q_MIN, Q_MAX)
    return q


def compute_ik(cc, task_pos):
    """Solve IK for task_pos so the real EE points straight down.

    Uses SimpleFranka.IterInvKin with T_goal oriented as R_DESIRED_SF
    (R_DESIRED rotated +45° about Z), then DLS-projects onto that manifold.
    After the real robot's panda_hand_joint applies Rz(-pi/4), O_T_EE will
    equal R_DESIRED = diag(1,-1,-1).

    Returns (q, pos_err_m, ori_err_rad) where ori_err is vs R_DESIRED_SF
    in SimpleFranka space.
    """
    T_goal = np.eye(4)
    T_goal[:3, :3] = R_DESIRED_SF   # target in SimpleFranka / panda_link8 frame
    T_goal[:3,  3] = task_pos

    q_seed = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    q_ik, _ = cc.IterInvKin(q_seed, T_goal)

    # DLS project to enforce orientation exactly (same step the PRM generator uses)
    q_proj = _project_to_vertical_down(cc, q_ik)

    # Verify position and orientation in SimpleFranka space
    Tcurr, _ = cc.ForwardKin(q_proj)
    fk_pos  = Tcurr[-1][:3, 3]
    R_curr  = np.array(Tcurr[-1][:3, :3])
    R_err   = R_DESIRED_SF @ R_curr.T
    _, ang  = rt.R2axisang(R_err)

    pos_err = np.linalg.norm(fk_pos - np.array(task_pos))
    ori_err = abs(ang)
    return q_proj, pos_err, ori_err


# ── Trajectory interpolation ──────────────────────────────────────────────────

def _minjerk_segment(q0, qf, n_pts):
    """Min-jerk polynomial from q0 to qf over n_pts points (including qf)."""
    t = np.linspace(0.0, 1.0, n_pts + 1)[1:]   # exclude t=0 (= q0)
    w = 10*t**3 - 15*t**4 + 6*t**5
    return q0 + np.outer(w, qf - q0)            # (n_pts, 7)


def build_segment_traj(q_start, q_end, duration_s, rate_hz=STREAM_RATE_HZ):
    """Build a min-jerk joint trajectory from q_start to q_end.

    Returns an (N, 7) array where N = round(duration_s * rate_hz).
    A 2x dwell at q_end is appended so the arm settles before termination.
    """
    n_move  = max(2, round(duration_s * rate_hz))
    traj    = _minjerk_segment(np.asarray(q_start), np.asarray(q_end), n_move)
    n_dwell = 2 * n_move
    dwell   = np.tile(traj[-1], (n_dwell, 1))
    return np.vstack([traj, dwell])              # (3*n_move, 7)


# ── Streaming execution ───────────────────────────────────────────────────────

def stream_segment(fa, pub, q_start, q_end, seg_duration,
                   k_gains=None, d_gains=None, buffer_time=30.0,
                   rate_hz=STREAM_RATE_HZ):
    """Execute one joint-space segment using goto_joints(dynamic=True).

    Pattern mirrors franka_prm_single_arm._execute_plan:
      1. goto_joints(dynamic=True, buffer_time=buffer_time) — opens PassThrough skill
      2. Stream JointPositionSensorMessage at rate_hz
      3. Send ShouldTerminateSensorMessage
      4. wait_for_skill()
    """
    k_gains = k_gains or [600, 600, 600, 600, 600, 300, 200]
    d_gains = d_gains or [ 50,  50,  50,  50,  30,  25,  15]

    traj   = build_segment_traj(q_start, q_end, seg_duration, rate_hz)
    n_pts  = traj.shape[0]

    # Open the dynamic skill (non-blocking, sets up PassThrough generator)
    fa.goto_joints(traj[0], duration=seg_duration + buffer_time,
                   dynamic=True, buffer_time=buffer_time,
                   k_gains=k_gains, d_gains=d_gains)

    init_time  = rospy.Time.now().to_time()
    rate       = rospy.Rate(rate_hz)
    late_count = 0
    dt_expect  = 1.0 / rate_hz

    for i in range(1, n_pts):
        t_before = rospy.Time.now().to_time()
        msg = JointPositionSensorMessage(
            id=i,
            timestamp=t_before - init_time,
            joints=traj[i].tolist(),
        )
        pub.publish(make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                msg, SensorDataMessageType.JOINT_POSITION)
        ))
        rate.sleep()
        if rospy.Time.now().to_time() - t_before > dt_expect * 1.5:
            late_count += 1

    if late_count:
        print(f"  [warn] {late_count}/{n_pts-1} publishes exceeded 1.5x deadline")

    # Terminate the skill
    term = ShouldTerminateSensorMessage(
        timestamp=rospy.Time.now().to_time() - init_time,
        should_terminate=True,
    )
    pub.publish(make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term, SensorDataMessageType.SHOULD_TERMINATE)
    ))
    fa.wait_for_skill()


# ── Trajectory generation ─────────────────────────────────────────────────────

def generate_trajectory(cc, num_waypoints, rng, max_ik_attempts=20):
    """Return a list of num_waypoints joint configs, each satisfying R_DESIRED.

    Positions are drawn uniformly from the workspace bounds of acc_repeat.py.
    IK failures are retried with a fresh random position.

    Returns list of (q, task_pos) tuples, or raises RuntimeError if too many
    IK attempts fail.
    """
    waypoints = []
    for wp_idx in range(num_waypoints):
        for attempt in range(max_ik_attempts):
            x = rng.uniform(X_MIN, X_MAX)
            y = rng.uniform(Y_MIN, Y_MAX)
            z = rng.uniform(Z_MIN, Z_MAX)
            task_pos = np.array([x, y, z])

            try:
                q, pos_err, ori_err = compute_ik(cc, task_pos)
            except Exception as e:
                print(f"    IK exception at waypoint {wp_idx+1} attempt {attempt+1}: {e}")
                continue

            if pos_err > 0.05:
                print(f"    IK pos_err={pos_err:.4f} m at attempt {attempt+1}, retrying…")
                continue
            if ori_err > np.radians(5):
                print(f"    IK ori_err={np.degrees(ori_err):.1f}° at attempt {attempt+1}, retrying…")
                continue

            waypoints.append((q, task_pos, pos_err, ori_err))
            print(f"  Waypoint {wp_idx+1}: pos={np.round(task_pos,3)}  "
                  f"pos_err={pos_err:.4f}m  ori_err={np.degrees(ori_err):.2f}°")
            break
        else:
            raise RuntimeError(
                f"Failed to find valid IK for waypoint {wp_idx+1} "
                f"after {max_ik_attempts} attempts"
            )
    return waypoints


# ── Verification helpers ──────────────────────────────────────────────────────

def check_ee_orientation(fa, tol_deg=5.0):
    """Return (tilt_deg, passed) using libfranka's FK via fa.get_pose().

    SimpleFranka.ForwardKin is intentionally NOT used here: TEST A confirmed it
    is offset by a constant 45° about Z relative to libfranka (missing the
    panda_hand_joint fixed transform).  fa.get_pose() is the ground truth.
    """
    pose = fa.get_pose()
    R_curr = pose.rotation                    # 3×3, world ← franka_tool
    R_err  = R_DESIRED @ R_curr.T
    _, ang = rt.R2axisang(R_err)
    tilt_deg = np.degrees(abs(ang))
    return tilt_deg, tilt_deg <= tol_deg


def ee_pos_from_fa(fa):
    """Return EE position [x, y, z] in metres using libfranka FK."""
    return fa.get_pose().translation.copy()


# ── Main test ─────────────────────────────────────────────────────────────────

def run_test(arm_number=1, num_traj=5, num_waypoints=3,
             seg_duration=5.0, seed=42):
    """Run goto_joints(dynamic=True) test trajectories.

    Args:
        arm_number    : 1 or 2
        num_traj      : number of random trajectories to execute
        num_waypoints : random FK poses per trajectory
        seg_duration  : time in seconds for each waypoint-to-waypoint segment
        seed          : numpy random seed for reproducibility
    """
    rng = np.random.default_rng(seed)

    print("=" * 70)
    print(f"goto_joints(dynamic=True) test")
    print(f"  arm={arm_number}  num_traj={num_traj}  "
          f"num_waypoints={num_waypoints}  seg_duration={seg_duration}s  seed={seed}")
    print("=" * 70)

    # ── Robot setup ───────────────────────────────────────────────────────────
    cc = SimpleFranka.SimpleFrankArm(arm_number=arm_number)

    rospy.init_node("test_goto_joints_dynamic", anonymous=True)
    fa  = FrankaArm(with_gripper=False, old_gripper=False,
                    robot_num=arm_number, init_node=False)
    pub = rospy.Publisher(ARM_SENSOR_TOPIC[arm_number],
                          SensorDataGroup, queue_size=1000)

    print("Robot connected. Resetting to home pose…")
    fa.reset_joints()

    # ── Pre-generate all trajectories ─────────────────────────────────────────
    print("\nGenerating trajectories (IK + DLS projection)…")
    trajectories = []
    for traj_idx in range(num_traj):
        print(f"\n  Trajectory {traj_idx+1}/{num_traj}:")
        waypoints = generate_trajectory(cc, num_waypoints, rng)
        trajectories.append(waypoints)

    # ── Execute ───────────────────────────────────────────────────────────────
    results = []

    for traj_idx, waypoints in enumerate(trajectories):
        print(f"\n{'─'*70}")
        print(f"Trajectory {traj_idx+1}/{num_traj}  ({len(waypoints)} waypoints)")
        print(f"{'─'*70}")

        # Always return to home before each trajectory
        print("  Resetting to home…")
        fa.reset_joints()
        q_current = fa.get_joints()

        traj_result = {
            "traj_idx":  traj_idx,
            "waypoints": [],
        }

        for wp_idx, (q_target, task_pos, ik_pos_err, ik_ori_err) in enumerate(waypoints):
            print(f"\n  Segment {wp_idx+1}/{len(waypoints)}: "
                  f"pos={np.round(task_pos, 3)}  q={np.round(q_target, 3)}")

            t_start = time.time()

            if wp_idx == 0:
                # First waypoint: use blocking goto_joints to pre-position from
                # home.  PassThroughJointTrajectoryGenerator requires the robot
                # to already be near the seed config; streaming from home to a
                # distant target saturates the impedance controller.
                print(f"    [pre-position via static goto_joints]")
                fa.goto_joints(q_target.tolist(), duration=seg_duration)
                method = "static"
            else:
                # Subsequent waypoints: robot is already near q_current (the
                # previous target), so the dynamic streaming jump is small and
                # the impedance controller tracks correctly.
                stream_segment(fa, pub, q_current, q_target, seg_duration)
                method = "dynamic"

            elapsed = time.time() - t_start

            q_actual = np.array(fa.get_joints())
            q_err    = np.linalg.norm(q_actual - q_target)

            # Use libfranka FK (fa.get_pose()) for orientation — SimpleFranka FK
            # is offset 45° about Z (missing panda_hand_joint) and gives wrong tilt.
            tilt_deg, ori_ok = check_ee_orientation(fa, tol_deg=5.0)
            ee_pos_actual  = ee_pos_from_fa(fa)
            pos_err_actual = np.linalg.norm(ee_pos_actual - task_pos)

            status = "PASS" if ori_ok else "FAIL"
            print(f"    [{status}] method={method}  elapsed={elapsed:.2f}s  "
                  f"q_err={q_err:.4f}rad  "
                  f"pos_err={pos_err_actual:.4f}m  "
                  f"EE_tilt={tilt_deg:.2f}°")

            traj_result["waypoints"].append({
                "task_pos":          task_pos,
                "q_target":          q_target,
                "q_actual":          q_actual,
                "q_err_norm":        q_err,
                "pos_err_m":         pos_err_actual,
                "ee_tilt_deg":       tilt_deg,
                "ori_constraint_ok": ori_ok,
                "elapsed_s":         elapsed,
                "method":            method,
            })

            q_current = q_actual

        results.append(traj_result)

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")

    all_q_errs  = []
    all_pos_errs = []
    all_tilts   = []
    n_pass = 0
    n_total = 0

    for tr in results:
        for wp in tr["waypoints"]:
            all_q_errs.append(wp["q_err_norm"])
            all_pos_errs.append(wp["pos_err_m"])
            all_tilts.append(wp["ee_tilt_deg"])
            n_total += 1
            if wp["ori_constraint_ok"]:
                n_pass += 1

    print(f"  Orientation constraint (±5°): {n_pass}/{n_total} passed")
    print(f"  Joint tracking error   — mean={np.mean(all_q_errs):.4f} rad  "
          f"max={np.max(all_q_errs):.4f} rad")
    print(f"  EE position error      — mean={np.mean(all_pos_errs):.4f} m  "
          f"max={np.max(all_pos_errs):.4f} m")
    print(f"  EE tilt from vertical  — mean={np.mean(all_tilts):.2f}°  "
          f"max={np.max(all_tilts):.2f}°")
    print(f"{'='*70}")

    fa.reset_joints()
    return results


# ── CLI entry point ───────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test goto_joints(dynamic=True) with straight-down EE trajectories"
    )
    parser.add_argument("--arm",           type=int,   default=1,   help="Arm number (1 or 2)")
    parser.add_argument("--num-traj",      type=int,   default=5,   help="Number of trajectories")
    parser.add_argument("--num-waypoints", type=int,   default=3,   help="Random waypoints per trajectory")
    parser.add_argument("--seg-duration",  type=float, default=5.0, help="Seconds per waypoint segment")
    parser.add_argument("--seed",          type=int,   default=42,  help="Random seed")
    args = parser.parse_args()

    run_test(
        arm_number    = args.arm,
        num_traj      = args.num_traj,
        num_waypoints = args.num_waypoints,
        seg_duration  = args.seg_duration,
        seed          = args.seed,
    )
