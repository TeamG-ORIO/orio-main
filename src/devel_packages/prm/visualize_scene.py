#!/usr/bin/env python3
"""Visualize orio_dual_scene.xml in a MuJoCo passive viewer.

Task poses for each arm can be set below. Each pose is specified in the
arm's own base frame (i.e. relative to mj_left_link0 / mj_right_link0).

Set a pose to None to leave that arm at its default (zero) joint angles.
"""

import sys
import os
import argparse
import pickle
import time
import numpy as np
import mujoco
from mujoco import viewer

# ── Task poses (arm-local base frame) ────────────────────────────────────────
# The label arm (mj_left) base is at world pos [-0.538, 0.0, 0.9].
# The pick  arm (mj_right) base is at world pos [ 0.709, 0.0, 0.9].
# To convert a world-frame target: subtract the arm's base position.

LABEL_ARM_POSE = {
    "translation": [0.18, 0.001, 0.41],  # LABELLING_SAFE in arm frame
    "rotation": [
        [0.7071,  -0.7071,  0],
        [-0.7071, -0.7071,  0],
        [0,  0, -1],
    ],
}

PICK_ARM_POSE = {
    "translation": [0.18, 0.001, 0.41],  # LABELLING_SAFE in arm frame
    "rotation": [
        [0.7071,  -0.7071,  0],
        [-0.7071, -0.7071,  0],
        [0,  0, -1],
    ],
}

# ── PD gains (same as franka_prm_dual_arm_gtj.py) ────────────────────────────
KP = np.array([120, 120, 100, 90, 60, 40, 30], dtype=float)
KD = np.array([  8,   8,   6,  5,  4,  3,  2], dtype=float)

# ─────────────────────────────────────────────────────────────────────────────

MODEL_XML = "mujoco_files/orio_dual_scene.xml"

# World-frame base positions and yaw for each arm (from orio_mj_dual.xml)
# arm 1 (mj_left):  pos=[-0.538, 0.0, 0.9], quat=1 0 0 0 (no rotation)
# arm 2 (mj_right): pos=[ 0.709, 0.0, 0.9], quat=0 0 0 1 (180 deg yaw)
_ARM_BASE_POS = {
    1: np.array([-0.538, 0.0, 0.9]),
    2: np.array([ 0.709, 0.0, 0.9]),
}
_ARM_BASE_ROT = {
    1: np.eye(3),
    2: np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]),  # 180 deg about Z
}

sys.path.insert(0, os.path.dirname(__file__))
import SimpleFranka
import RobotUtil as rt


def compute_connected_components(edges):
    """Return a list of component ids (one per node) via BFS on the adjacency list."""
    n = len(edges)
    component = [-1] * n
    comp_id = 0
    for start in range(n):
        if component[start] != -1:
            continue
        queue = [start]
        component[start] = comp_id
        while queue:
            node = queue.pop()
            for nb in edges[node]:
                if component[nb] == -1:
                    component[nb] = comp_id
                    queue.append(nb)
        comp_id += 1
    return component, comp_id  # component ids per node, total number of components


def load_prm_ee_positions(prm_file, arm_number):
    """Load a PRM file and return world-frame EE positions and per-node component ids."""
    with open(prm_file, 'rb') as f:
        prm_vertices = pickle.load(f)
        prm_edges    = pickle.load(f)
        pickle.load(f)  # obs_points
        pickle.load(f)  # obs_axes

    robot    = SimpleFranka.SimpleFrankArm(arm_number=arm_number)
    base_pos = _ARM_BASE_POS[arm_number]
    base_rot = _ARM_BASE_ROT[arm_number]

    ee_positions = []
    for q in prm_vertices:
        Tcurr, _ = robot.ForwardKin(q)
        p_local = np.array(Tcurr[-1])[:3, 3]
        p_world = base_rot @ p_local + base_pos
        ee_positions.append(p_world)

    component_ids, num_components = compute_connected_components(prm_edges)
    return np.array(ee_positions), component_ids, num_components, prm_vertices, robot


def plot_ee_orientation_deviation(prm_vertices, robot, arm_number):
    """Histogram orientation deviation of each PRM node from the straight-down constraint.

    Uses the same metric as PRMGenerator_DLS_JPI: axis-angle magnitude of
    R_DESIRED @ R_curr^T, where R_DESIRED is the gripper-points-down target.
    """
    import matplotlib.pyplot as plt

    # Same R_DESIRED as PRMGenerator_DLS_JPI.py
    R_DESIRED = np.array([
        [ 1,  0,  0],
        [ 0, -1,  0],
        [ 0,  0, -1],
    ], dtype=float)

    deviations_deg = []
    for q in prm_vertices:
        Tcurr, _ = robot.ForwardKin(q)
        R_curr = np.array(Tcurr[-1])[:3, :3]
        R_err = R_DESIRED @ R_curr.T
        _, ang = rt.R2axisang(R_err)       # same call as the generator
        deviations_deg.append(np.degrees(abs(ang)))

    deviations_deg = np.array(deviations_deg)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(deviations_deg, bins=40, color='steelblue', edgecolor='white', linewidth=0.5)
    ax.axvline(np.degrees(0.05), color='tomato', linestyle='--',
               label='generator cutoff (0.05 rad ≈ 2.9°)')
    ax.set_xlabel("Orientation deviation from straight-down [deg] (axis-angle magnitude)")
    ax.set_ylabel("Number of PRM nodes")
    ax.set_title(f"EE orientation deviation — arm {arm_number} "
                 f"({len(deviations_deg)} nodes)\n"
                 f"median={np.median(deviations_deg):.2f}°  "
                 f"90th pct={np.percentile(deviations_deg, 90):.2f}°  "
                 f"max={deviations_deg.max():.2f}°")
    ax.legend()
    plt.tight_layout()
    plt.savefig(f"prm_orientation_deviation_arm{arm_number}.png", dpi=150)
    print(f"Orientation deviation plot saved to prm_orientation_deviation_arm{arm_number}.png")
    plt.show()


def add_prm_spheres(scn, positions, rgba, radius=0.012):
    """Inject spheres into a MuJoCo mjvScene for each PRM node position."""
    for pos in positions:
        if scn.ngeom >= scn.maxgeom:
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0.0, 0.0]),   # size (only [0] used for sphere)
            pos.astype(np.float64),
            np.eye(3).flatten().astype(np.float64),
            rgba.astype(np.float32),
        )
        scn.ngeom += 1


# Distinct colors for up to N components; extras cycle back through the list.
_COMPONENT_COLORS = [
    [1.0, 0.2, 0.2, 0.9],  # red
    [0.2, 0.4, 1.0, 0.9],  # blue
    [0.2, 0.9, 0.2, 0.9],  # green
    [1.0, 0.8, 0.0, 0.9],  # yellow
    [0.9, 0.3, 1.0, 0.9],  # magenta
    [0.0, 0.9, 0.9, 0.9],  # cyan
    [1.0, 0.5, 0.0, 0.9],  # orange
    [0.5, 0.0, 0.5, 0.9],  # purple
]


def add_prm_spheres_by_component(scn, positions, component_ids, radius=0.012):
    """Draw each PRM node with a color determined by its connected component."""
    colors = [np.array(c, dtype=np.float32) for c in _COMPONENT_COLORS]
    for pos, cid in zip(positions, component_ids):
        if scn.ngeom >= scn.maxgeom:
            break
        rgba = colors[cid % len(colors)]
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([radius, 0.0, 0.0]),
            pos.astype(np.float64),
            np.eye(3).flatten().astype(np.float64),
            rgba,
        )
        scn.ngeom += 1


def pose_to_homogeneous(pose):
    T = np.eye(4)
    T[:3, :3] = np.array(pose["rotation"], dtype=float)
    T[:3,  3] = np.array(pose["translation"], dtype=float)
    return T


def run_ik(arm_number, pose, initial_guess=None):
    robot = SimpleFranka.SimpleFrankArm(arm_number=arm_number)
    T_goal = pose_to_homogeneous(pose)
    if initial_guess is None:
        initial_guess = [0.0, -0.3, 0.0, -1.5, 0.0, 1.8, 0.0]
    q, err = robot.IterInvKin(initial_guess, T_goal)
    pos_err = np.linalg.norm(err[:3])
    rot_err = np.linalg.norm(err[3:])
    print(f"  Arm {arm_number} IK  pos_err={pos_err:.4f} m  rot_err={rot_err:.4f} rad")
    if pos_err > 5e-3 or rot_err > 5e-3:
        print(f"  WARNING: IK may not have converged for arm {arm_number}")
    return np.array(q)


LEFT_JOINT_NAMES  = [f"mj_left_joint{i}"  for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"mj_right_joint{i}" for i in range(1, 8)]

# ── CLI args ─────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Visualize orio dual scene.")
parser.add_argument("--prm-file", type=str, default=None,
                    help="Path to a PRM .p file. If given, end-effector positions of "
                         "all nodes are shown as spheres in the scene.")
parser.add_argument("--arm", type=int, choices=[1, 2], default=1,
                    help="Which arm the PRM was built for (default: 1)")
args = parser.parse_args()

prm_ee_pos   = None
prm_comp_ids = None
if args.prm_file is not None:
    print(f"Loading PRM from '{args.prm_file}' for arm {args.arm}...")
    prm_ee_pos, prm_comp_ids, num_components, prm_vertices, prm_robot = \
        load_prm_ee_positions(args.prm_file, args.arm)
    print(f"  {len(prm_ee_pos)} nodes loaded, {num_components} connected component(s).")

    from collections import Counter
    for cid, sz in sorted(Counter(prm_comp_ids).items(), key=lambda x: -x[1]):
        print(f"    component {cid}: {sz} nodes")

    plot_ee_orientation_deviation(prm_vertices, prm_robot, args.arm)

# ── Load model ────────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data  = mujoco.MjData(model)

# ── Resolve joint indices ─────────────────────────────────────────────────────
l_qpos = [model.joint(n).qposadr[0] for n in LEFT_JOINT_NAMES]
l_qvel = [model.joint(n).dofadr[0]  for n in LEFT_JOINT_NAMES]
l_ctrl = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"mj_left_act_trq{i}")  for i in range(1, 8)]

r_qpos = [model.joint(n).qposadr[0] for n in RIGHT_JOINT_NAMES]
r_qvel = [model.joint(n).dofadr[0]  for n in RIGHT_JOINT_NAMES]
r_ctrl = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"mj_right_act_trq{i}") for i in range(1, 8)]

# ── Run IK ───────────────────────────────────────────────────────────────────
q_left_goal  = np.zeros(7)
q_right_goal = np.zeros(7)

if LABEL_ARM_POSE is not None:
    print("Running IK for label arm (mj_left)...")
    q_left_goal = run_ik(arm_number=1, pose=LABEL_ARM_POSE)
    print(f"  Joint angles: {np.round(q_left_goal, 4).tolist()}")

if PICK_ARM_POSE is not None:
    print("Running IK for pick arm (mj_right)...")
    q_right_goal = run_ik(arm_number=2, pose=PICK_ARM_POSE)
    print(f"  Joint angles: {np.round(q_right_goal, 4).tolist()}")

# Start both arms at zero
data.qpos[l_qpos] = 0.0
data.qpos[r_qpos] = 0.0
data.qvel[:]      = 0.0
mujoco.mj_forward(model, data)

# ── Launch viewer with PD + gravity compensation controller ───────────────────
render_dt = 1.0 / 60.0

with viewer.launch_passive(model, data) as v:
    t_wall = time.time()
    while v.is_running():
        # PD + gravity compensation for left arm
        q_l  = data.qpos[l_qpos]
        qd_l = data.qvel[l_qvel]
        tau_l = KP * (q_left_goal - q_l) + KD * (0.0 - qd_l)
        data.ctrl[l_ctrl] = tau_l + data.qfrc_bias[l_qvel]

        # PD + gravity compensation for right arm
        q_r  = data.qpos[r_qpos]
        qd_r = data.qvel[r_qvel]
        tau_r = KP * (q_right_goal - q_r) + KD * (0.0 - qd_r)
        data.ctrl[r_ctrl] = tau_r + data.qfrc_bias[r_qvel]

        mujoco.mj_step(model, data)

        now = time.time()
        if now - t_wall >= render_dt:
            if prm_ee_pos is not None:
                with v.lock():
                    v.user_scn.ngeom = 0
                    add_prm_spheres_by_component(
                        v.user_scn, prm_ee_pos, prm_comp_ids)
            v.sync()
            t_wall = now
