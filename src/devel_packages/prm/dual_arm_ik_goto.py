#!/usr/bin/env python3
"""
Dual-arm IK + goto_joints.

Accepts a Cartesian pose (x, y, z, yaw) for each arm, solves IK, and
drives both arms to the resulting joint configuration simultaneously.

Usage:
    python dual_arm_ik_goto.py
    python dual_arm_ik_goto.py --pos1 0.4 -0.3 0.3 --yaw1 0.0 \
                                --pos2 0.4  0.3 0.3 --yaw2 0.0 \
                                --duration 5.0
"""

import argparse
import os
import numpy as np
import ikpy.chain

from frankapy import FrankaArm

_URDF_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "../orio/panda_arm_hand.urdf")


def build_ik_chain():
    """Load the IK chain once (URDF is shared by both arms)."""
    chain = ikpy.chain.Chain.from_urdf_file(_URDF_FILE,
                                             base_elements=["panda_link0"])
    n = len(chain.links)
    chain.active_links_mask = [False] + [True] * 7 + [False] * (n - 8)
    return chain, n


def compute_ik(chain, chain_len, task_pos, yaw=0.0):
    """Return 7-DOF joint angles for a Cartesian position + EE yaw.

    Args:
        chain     : ikpy.chain.Chain (shared, stateless)
        chain_len : total number of links in the chain
        task_pos  : [x, y, z] in metres in the arm base frame
        yaw       : EE rotation about world Z (radians, default 0)

    Returns:
        np.ndarray of shape (7,)
    """
    cy, sy = np.cos(yaw), np.sin(yaw)
    # Downward-pointing EE, rotated by yaw about Z
    target_ori = (np.array([[ cy, -sy, 0.0],
                             [ sy,  cy, 0.0],
                             [0.0, 0.0, 1.0]])
                  @ np.array([[1.0,  0.0,  0.0],
                               [0.0, -1.0,  0.0],
                               [0.0,  0.0, -1.0]]))

    initial_guess = [0.0] * chain_len
    initial_guess[4] = -1.5  # elbow hint

    angles = chain.inverse_kinematics(
        target_position=task_pos,
        target_orientation=target_ori,
        orientation_mode="all",
        initial_position=initial_guess,
    )
    q = angles[1:8]

    fk_pos = chain.forward_kinematics(angles)[:3, 3]
    err = np.linalg.norm(fk_pos - np.array(task_pos))
    if err > 0.01:
        print(f"  [IK] WARNING  pos_err={err:.4f} m  "
              f"target={np.round(task_pos, 4)}  fk={np.round(fk_pos, 4)}")
    else:
        print(f"  [IK] OK  joints={np.round(q, 3)}  pos_err={err:.4f} m")
    return q


def goto_joints_dual(fa1, fa2, q1, q2, duration=5.0):
    """Send both arms to their target joint configs simultaneously.

    Sends goto_joints non-blocking to arm1, then arm2, then waits for both.
    """
    print(f"\nMoving arm1 -> {np.round(q1, 3)}")
    print(f"Moving arm2 -> {np.round(q2, 3)}")

    fa1.goto_joints(q1, duration=duration, block=False)
    fa2.goto_joints(q2, duration=duration, block=False)

    fa1.wait_for_skill()
    fa2.wait_for_skill()
    print("Both arms reached target.")


def main():
    parser = argparse.ArgumentParser(description="Dual-arm IK + goto_joints")
    parser.add_argument("--pos1",     type=float, nargs=3,
                        default=[0.4, -0.3, 0.3],
                        metavar=("X", "Y", "Z"),
                        help="Goal Cartesian position for arm 1 (metres)")
    parser.add_argument("--yaw1",     type=float, default=0.0,
                        help="Goal EE yaw for arm 1 (radians)")
    parser.add_argument("--pos2",     type=float, nargs=3,
                        default=[0.4,  -0.3, 0.3],
                        metavar=("X", "Y", "Z"),
                        help="Goal Cartesian position for arm 2 (metres)")
    parser.add_argument("--yaw2",     type=float, default=0.0,
                        help="Goal EE yaw for arm 2 (radians)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Motion duration in seconds (default: 5.0)")
    args = parser.parse_args()

    # ── Connect to robots ─────────────────────────────────────────────────────
    print("Connecting to arm 1 ...")
    fa1 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1)
    print("Connecting to arm 2 ...")
    fa2 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2,
                    init_node=False)

    # ── IK ────────────────────────────────────────────────────────────────────
    chain, chain_len = build_ik_chain()

    print(f"\nArm 1  pos={args.pos1}  yaw={args.yaw1:.3f}")
    q1 = compute_ik(chain, chain_len, args.pos1, yaw=args.yaw1)

    print(f"\nArm 2  pos={args.pos2}  yaw={args.yaw2:.3f}")
    q2 = compute_ik(chain, chain_len, args.pos2, yaw=args.yaw2)

    # ── Execute ───────────────────────────────────────────────────────────────
    goto_joints_dual(fa1, fa2, q1, q2, duration=args.duration)


if __name__ == "__main__":
    main()
