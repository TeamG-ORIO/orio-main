#!/usr/bin/env python3
"""
Unit test for goto_joints_with_velocity.

Goal poses are specified in task space (x, y, z) and converted to joint angles
via IK (ikpy, identical to franka_prm_single_arm._compute_ik).  For each pose
pair the robot moves through a two-waypoint cubic Hermite spline:

  home  →  [optional pre-position via goto_joints]  →  waypoint_0
  waypoint_0  →  goto_joints_with_velocity(q1, qd1)  →  waypoint_1

The intermediate velocity qd0 is set to the finite-difference estimate of the
joint velocity at the transition point (so the spline is C1 continuous).
The final velocity qd1 is zero (arm comes to rest at the goal).

Usage:
    python test_goto_joints_with_velocity.py
    python test_goto_joints_with_velocity.py --arm 1 --num-poses 5 --speed 0.3
    python test_goto_joints_with_velocity.py --seed 7 --duration 4.0
    python test_goto_joints_with_velocity.py --arm 2 --speed 0.2 --no-reset
"""

import argparse
import sys
import time
import numpy as np

# ── local imports ──────────────────────────────────────────────────────────────
sys.path.insert(0, "/home/ros_ws/research/prm")
import ikpy.chain
import RobotUtil as rt

# ── ROS / frankapy ────────────────────────────────────────────────────────────
from frankapy import FrankaArm

# ── Constants ─────────────────────────────────────────────────────────────────

_URDF_FILE = "/home/ros_ws/src/devel_packages/orio/panda_arm_hand.urdf"

# "Straight down" EE orientation (O_T_EE in libfranka / fa.get_pose()).
R_DESIRED = np.array([
    [ 1,  0,  0],
    [ 0, -1,  0],
    [ 0,  0, -1],
], dtype=float)

# Workspace bounds (same as acc_repeat.py / test_goto_joints_dynamic.py)
X_MIN, X_MAX = 0.2, 0.8
Y_MIN, Y_MAX = -0.3, 0.3
Z_MIN, Z_MAX = 0.15, 0.75

# Joint limits (slightly tighter on joint 1, matching PRM generator)
Q_MIN = np.array([-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

ARM_SENSOR_TOPIC = {
    1: "/franka_ros_interface/sensor",
    2: "/franka_ros_interface_2/sensor",
}


# ── IK (mirrors franka_prm_single_arm._compute_ik) ────────────────────────────

def _build_ik_chain():
    """Load ikpy chain from URDF. Activates joints 1-7 (panda_joint1–7)."""
    chain = ikpy.chain.Chain.from_urdf_file(_URDF_FILE,
                                             base_elements=["panda_link0"])
    n = len(chain.links)
    chain.active_links_mask = [False] + [True] * 7 + [False] * (n - 8)
    return chain, n


def compute_ik(ik_chain, chain_len, task_pos, yaw=0.0):
    """Return 7-DOF joint angles for task_pos with the gripper pointing down.

    Orientation: straight down, optionally rotated by yaw about world-Z.
    Raises RuntimeError if the IK position error exceeds 1 cm.

    Args:
        ik_chain  : ikpy.chain.Chain
        chain_len : total number of links in the chain
        task_pos  : [x, y, z] in metres
        yaw       : EE yaw rotation in radians (default 0)

    Returns:
        q7 : np.ndarray of shape (7,)
    """
    cy, sy = np.cos(yaw), np.sin(yaw)
    R_yaw = np.array([[ cy, -sy, 0],
                      [ sy,  cy, 0],
                      [  0,   0, 1]])
    target_ori = R_yaw @ R_DESIRED

    initial = [0.0] * chain_len
    initial[4] = -1.5  # elbow-down seed (matches franka_prm_single_arm)

    angles = ik_chain.inverse_kinematics(
        target_position=task_pos,
        target_orientation=target_ori,
        orientation_mode="all",
        initial_position=initial,
    )
    q7 = np.array(angles[1:8])
    fk_pos = ik_chain.forward_kinematics(angles)[:3, 3]
    err = np.linalg.norm(fk_pos - np.array(task_pos))
    if err > 0.01:
        raise RuntimeError(
            f"IK position error {err:.4f} m > 1 cm  "
            f"(target={np.round(task_pos,4)}  fk={np.round(fk_pos,4)})"
        )
    return q7


# ── Velocity estimation ────────────────────────────────────────────────────────

def estimate_transition_velocity(q_prev, q_next, duration):
    """Central finite-difference estimate of joint velocity at a waypoint.

    Approximates the derivative of a path passing through q_prev → q_mid → q_next
    at the midpoint using a symmetric difference.  Provides a smooth C1 entry
    velocity into the next cubic Hermite segment.

    Args:
        q_prev    : joint config before the transition waypoint  (7,)
        q_next    : joint config after  the transition waypoint  (7,)
        duration  : time per segment in seconds

    Returns:
        qd : np.ndarray of shape (7,)  [rad/s]
    """
    return (np.array(q_next) - np.array(q_prev)) / (2.0 * duration)


# ── Verification ───────────────────────────────────────────────────────────────

def check_ee_orientation(fa, tol_deg=5.0):
    """Check EE tilt from straight-down using libfranka FK (fa.get_pose()).

    Returns:
        tilt_deg : float — angle from R_DESIRED in degrees
        passed   : bool  — True if within tol_deg
    """
    pose = fa.get_pose()
    R_curr = pose.rotation
    R_err  = R_DESIRED @ R_curr.T
    _, ang = rt.R2axisang(R_err)
    tilt_deg = np.degrees(abs(ang))
    return tilt_deg, tilt_deg <= tol_deg


def check_joint_error(fa, q_target):
    """Return L2 norm of joint error between actual and target (rad)."""
    q_actual = np.array(fa.get_joints())
    return float(np.linalg.norm(q_actual - np.array(q_target))), q_actual


def check_ee_position(fa, task_pos):
    """Return L2 error (m) between actual EE position and target task_pos."""
    ee_actual = fa.get_pose().translation.copy()
    return float(np.linalg.norm(ee_actual - np.array(task_pos))), ee_actual


# ── Test runner ────────────────────────────────────────────────────────────────

def run_test(arm_number=1, num_poses=5, speed=0.3, seed=42,
             reset_between=True):
    """Test goto_joints_with_velocity with task-space goals converted via IK.

    For every pair of consecutive task-space poses (p0, p1):
      1. IK solve both poses.
      2. Pre-position the robot at q0 using a blocking goto_joints call.
      3. Estimate the C1 transition velocity at q0 using finite differences
         with q_home and q1 as neighbours.
      4. Call goto_joints_with_velocity(q1, qd0, duration) to move from q0
         to q1 with a smooth spline entry velocity.
      5. Verify joint error, EE position error, and EE orientation tilt.

    Args:
        arm_number     : 1 or 2
        num_poses      : number of random Cartesian poses to generate
        speed          : joint-space speed (rad/s) used to derive segment duration
        seed           : numpy RNG seed for reproducibility
        reset_between  : if True, return to home between every pose pair
    """
    rng = np.random.default_rng(seed)

    print("=" * 70)
    print("goto_joints_with_velocity — task-space unit test")
    print(f"  arm={arm_number}  num_poses={num_poses}  speed={speed} rad/s  seed={seed}")
    print("=" * 70)

    # ── Robot + IK setup ──────────────────────────────────────────────────────
    ik_chain, chain_len = _build_ik_chain()

    # Let FrankaArm call rospy.init_node internally (with disable_signals=True).
    # Calling rospy.init_node ourselves first and then passing init_node=False
    # causes the interface status check to time out even when the robot is ready.
    fa = FrankaArm(rosnode_name="test_goto_joints_with_velocity",
                   with_gripper=False, old_gripper=False,
                   robot_num=arm_number, init_node=True)

    print("Robot connected. Resetting to home…")
    fa.reset_joints()
    q_home = np.array(fa.get_joints())

    # ── Generate poses & solve IK ─────────────────────────────────────────────
    print("\nGenerating task-space poses and solving IK…")
    poses = []
    max_attempts = 50
    while len(poses) < num_poses:
        x = rng.uniform(X_MIN, X_MAX)
        y = rng.uniform(Y_MIN, Y_MAX)
        z = rng.uniform(Z_MIN, Z_MAX)
        task_pos = np.array([x, y, z])
        attempt = 0
        while attempt < max_attempts:
            try:
                q = compute_ik(ik_chain, chain_len, task_pos)
                q = np.clip(q, Q_MIN, Q_MAX)
                poses.append((task_pos, q))
                print(f"  Pose {len(poses)}: pos={np.round(task_pos, 3)}  "
                      f"q={np.round(q, 3)}")
                break
            except RuntimeError as e:
                attempt += 1
                # Resample position on IK failure
                x = rng.uniform(X_MIN, X_MAX)
                y = rng.uniform(Y_MIN, Y_MAX)
                z = rng.uniform(Z_MIN, Z_MAX)
                task_pos = np.array([x, y, z])
        else:
            print(f"  [warn] Could not solve IK after {max_attempts} attempts, skipping")

    if len(poses) < 2:
        print("[ERROR] Need at least 2 valid poses to test. Exiting.")
        return []

    # ── Execute waypoint pairs ────────────────────────────────────────────────
    results = []
    n_pairs = len(poses) - 1

    for i in range(n_pairs):
        task_pos0, q0 = poses[i]
        task_pos1, q1 = poses[i + 1]

        # Arc-length based duration for each segment
        arc0 = float(np.linalg.norm(q0 - q_home))   # home → q0
        arc1 = float(np.linalg.norm(q1 - q0))        # q0   → q1

        dur0 = max(1.0, arc0 / speed)
        dur1 = max(1.0, arc1 / speed)

        # C1 transition velocity at q0: central difference using q_home and q1
        # (weighted by segment durations to handle unequal step sizes)
        qd_at_q0 = (q1 - q_home) / (dur0 + dur1)
        qd_at_q1 = np.zeros(7)  # come to rest at the goal

        print(f"\n{'─'*70}")
        print(f"Pair {i+1}/{n_pairs}")
        print(f"  Start : pos={np.round(task_pos0, 3)}  q={np.round(q0, 3)}")
        print(f"  Goal  : pos={np.round(task_pos1, 3)}  q={np.round(q1, 3)}")
        print(f"  dur0={dur0:.2f}s  dur1={dur1:.2f}s  "
              f"arc1={arc1:.4f} rad")
        print(f"  entry velocity qd={np.round(qd_at_q0, 3)} rad/s")

        if reset_between:
            print("  Resetting to home…")
            fa.reset_joints()

        # ── Pre-position at q0 (blocking static move) ─────────────────────────
        print(f"  Pre-positioning at q0 (static goto_joints, {dur0:.2f}s)…")
        t0 = time.time()
        fa.goto_joints(q0.tolist(), duration=dur0)
        elapsed_pre = time.time() - t0

        q_err_pre, q_actual_pre = check_joint_error(fa, q0)
        pos_err_pre, ee_pre     = check_ee_position(fa, task_pos0)
        tilt_pre, ori_ok_pre    = check_ee_orientation(fa)

        print(f"    Pre-position done in {elapsed_pre:.2f}s  "
              f"q_err={q_err_pre:.4f} rad  "
              f"pos_err={pos_err_pre:.4f} m  "
              f"EE_tilt={tilt_pre:.2f}°")

        # ── goto_joints_with_velocity ─────────────────────────────────────────
        print(f"  goto_joints_with_velocity → q1 ({dur1:.2f}s)…")
        t1 = time.time()
        fa.goto_joints_with_velocity(
            joints=q1.tolist(),
            joint_velocities=qd_at_q1.tolist(),
            duration=dur1,
        )
        elapsed_gwv = time.time() - t1

        q_err_goal, q_actual_goal = check_joint_error(fa, q1)
        pos_err_goal, ee_goal     = check_ee_position(fa, task_pos1)
        tilt_goal, ori_ok_goal    = check_ee_orientation(fa)

        status = "PASS" if ori_ok_goal else "FAIL"
        print(f"    [{status}] done in {elapsed_gwv:.2f}s  "
              f"q_err={q_err_goal:.4f} rad  "
              f"pos_err={pos_err_goal:.4f} m  "
              f"EE_tilt={tilt_goal:.2f}°")

        results.append({
            "pair_idx":       i,
            "task_pos0":      task_pos0,
            "task_pos1":      task_pos1,
            "q0":             q0,
            "q1":             q1,
            "qd_entry":       qd_at_q0,
            "dur0_s":         dur0,
            "dur1_s":         dur1,
            "q_err_pre_rad":  q_err_pre,
            "q_err_goal_rad": q_err_goal,
            "pos_err_goal_m": pos_err_goal,
            "ee_tilt_goal_deg": tilt_goal,
            "ori_ok":         ori_ok_goal,
            "elapsed_gwv_s":  elapsed_gwv,
        })

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print("SUMMARY — goto_joints_with_velocity test")
    print(f"{'='*70}")

    q_errs   = [r["q_err_goal_rad"]   for r in results]
    pos_errs = [r["pos_err_goal_m"]   for r in results]
    tilts    = [r["ee_tilt_goal_deg"] for r in results]
    n_pass   = sum(r["ori_ok"] for r in results)

    print(f"  Orientation (±5°): {n_pass}/{len(results)} passed")
    print(f"  Joint error  — mean={np.mean(q_errs):.4f} rad  max={np.max(q_errs):.4f} rad")
    print(f"  EE pos error — mean={np.mean(pos_errs):.4f} m   max={np.max(pos_errs):.4f} m")
    print(f"  EE tilt      — mean={np.mean(tilts):.2f}°   max={np.max(tilts):.2f}°")
    print(f"{'='*70}")

    fa.reset_joints()
    return results


# ── CLI entry point ────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test goto_joints_with_velocity with task-space poses converted via IK"
    )
    parser.add_argument("--arm",       type=int,   default=1,    help="Arm number (1 or 2)")
    parser.add_argument("--num-poses", type=int,   default=5,    help="Number of random task-space poses")
    parser.add_argument("--speed",     type=float, default=0.3,  help="Joint-space speed (rad/s) for timing")
    parser.add_argument("--seed",      type=int,   default=42,   help="Random seed")
    parser.add_argument("--no-reset",  action="store_true",      help="Skip home reset between pose pairs")
    args = parser.parse_args()

    run_test(
        arm_number    = args.arm,
        num_poses     = args.num_poses,
        speed         = args.speed,
        seed          = args.seed,
        reset_between = not args.no_reset,
    )
