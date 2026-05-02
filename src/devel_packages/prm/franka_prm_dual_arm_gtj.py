#!/usr/bin/env python3
"""
PRM-based point-to-point motion on the real Franka robot.
Uses the pre-built PRM (prm_files/myPRM3.p) and frankapy for execution.

Usage:
    python franka_prm_runner.py                       # real robot only
    python franka_prm_runner.py --sim                 # real robot + MuJoCo visualization
    python franka_prm_runner.py --sim-only            # MuJoCo only (no real robot)
    python franka_prm_runner.py --sim --duration 3.0
"""

import argparse
import numpy as np
import pickle
import heapq
import random
import sys

import ikpy.chain
import RobotUtil as rt
import SimpleFranka  # offline kinematics/collision only

URDF_FILE = "../orio/panda_arm_hand.urdf"

def compute_ik(task_pos):
    """Compute 7-DOF joint angles for a Cartesian task position using ikpy.
    Matches the method used in pick-place-label_old.py:
      - fixed downward end-effector orientation
      - initial_guess[4] = -1.5
    """
    ik_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE, base_elements=["panda_link0"])
    chain_length = len(ik_chain.links)
    ik_chain.active_links_mask = [False] + [True]*7 + [False]*(chain_length - 8)

    target_ori = np.array([[1.0, 0.0, 0.0],
                            [0.0,-1.0, 0.0],
                            [0.0, 0.0,-1.0]])
    initial_guess = [0.0] * chain_length
    initial_guess[4] = -1.5

    angles = ik_chain.inverse_kinematics(
        target_position=task_pos,
        target_orientation=target_ori,
        orientation_mode="all",
        initial_position=initial_guess,
    )
    return angles[1:8]

PRM_FILE_ARM1 = "prm_files_old/myPRM_arm1_free.p"
PRM_FILE_ARM2 = "prm_files_old/myPRM_arm2_free.p"
MODEL_XML     = "mujoco_files/orio_dual_scene.xml"

LEFT_JOINT_NAMES  = [f"mj_left_joint{i}"  for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"mj_right_joint{i}" for i in range(1, 8)]

KP = np.array([120, 120, 100, 90, 60, 40, 30], dtype=float)
KD = np.array([  8,   8,   6,  5,  4,  3,  2], dtype=float)


# ── PRM Query (same logic as PRMQueryWithAStar.py) ────────────────────────────

def prm_query(q_init, q_goal, mybot, prm_file):
    with open(prm_file, 'rb') as f:
        prmVertices = pickle.load(f)
        prmEdges    = pickle.load(f)
        pointsObs   = pickle.load(f)
        axesObs     = pickle.load(f)

    num_nodes, num_edges, num_components = rt.AnalyzeGraph(prmVertices, prmEdges)
    print(f"PRM: {num_nodes} nodes, {num_edges} edges, {num_components} components")

    def find_neighbors(q):
        dists = sorted(range(len(prmVertices)),
                       key=lambda i: np.linalg.norm(np.array(prmVertices[i]) - np.array(q)))
        neighbors = []
        for i in dists:
            if not mybot.DetectCollisionEdge(prmVertices[i], q, pointsObs, axesObs):
                neighbors.append(i)
            if len(neighbors) >= 10:
                break
        return neighbors

    if mybot.DetectCollision(q_init, pointsObs, axesObs):
        print("ERROR: start configuration is in collision")
        return None
    if mybot.DetectCollision(q_goal, pointsObs, axesObs):
        print("ERROR: goal configuration is in collision")
        return None

    neigh_init = find_neighbors(q_init)
    neigh_goal = find_neighbors(q_goal)
    print(f"Init neighbors: {len(neigh_init)}, Goal neighbors: {len(neigh_goal)}")
    if not neigh_init or not neigh_goal:
        print("Could not connect start/goal to PRM")
        return None

    heuristic = [np.linalg.norm(np.array(v) - np.array(q_goal)) for v in prmVertices]
    g_cost    = [float('inf')] * len(prmVertices)
    parent    = [None] * len(prmVertices)

    open_set = []
    for n in neigh_init:
        g = np.linalg.norm(np.array(prmVertices[n]) - np.array(q_init))
        g_cost[n] = g
        heapq.heappush(open_set, (g + heuristic[n], n))

    closed_set = set()
    goal_node  = None
    while open_set:
        _, curr = heapq.heappop(open_set)
        if curr in closed_set:
            continue
        closed_set.add(curr)
        if curr in neigh_goal:
            goal_node = curr
            break
        for nb in prmEdges[curr]:
            if nb in closed_set:
                continue
            edge_cost = np.linalg.norm(np.array(prmVertices[nb]) - np.array(prmVertices[curr]))
            tg = g_cost[curr] + edge_cost
            if tg < g_cost[nb]:
                g_cost[nb] = tg
                parent[nb]  = curr
                heapq.heappush(open_set, (tg + heuristic[nb], nb))

    if goal_node is None:
        print("A* failed to find path")
        return None

    path = [goal_node]
    while parent[path[0]] is not None:
        path.insert(0, parent[path[0]])

    plan = ([np.array(q_init)]
            + [np.array(prmVertices[i]) for i in path]
            + [np.array(q_goal)])

    for _ in range(200):
        if len(plan) <= 2:
            break
        i = random.randint(0, len(plan) - 3)
        j = random.randint(i + 2, len(plan) - 1)
        if not mybot.DetectCollisionEdge(plan[i], plan[j], pointsObs, axesObs):
            plan = plan[:i+1] + plan[j:]

    print(f"Plan: {len(plan)} waypoints after shortcutting")
    return plan


# ── MuJoCo helpers ────────────────────────────────────────────────────────────

def _setup_mujoco(plan1, plan2):
    """Load MuJoCo model and open passive viewer, initialised to plan[0] for both arms."""
    import mujoco as mj
    from mujoco import viewer as mj_viewer

    model = mj.MjModel.from_xml_path(MODEL_XML)
    data  = mj.MjData(model)

    # Resolve qpos/qvel/ctrl indices by name for each arm
    l_qpos = [model.joint(name).qposadr[0] for name in LEFT_JOINT_NAMES]
    l_qvel = [model.joint(name).dofadr[0]  for name in LEFT_JOINT_NAMES]
    l_ctrl = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, f"mj_left_act_trq{i}")  for i in range(1, 8)]

    r_qpos = [model.joint(name).qposadr[0] for name in RIGHT_JOINT_NAMES]
    r_qvel = [model.joint(name).dofadr[0]  for name in RIGHT_JOINT_NAMES]
    r_ctrl = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, f"mj_right_act_trq{i}") for i in range(1, 8)]

    data.qpos[l_qpos] = plan1[0].copy()
    data.qpos[r_qpos] = plan2[0].copy()
    data.qvel[:]      = 0
    mj.mj_forward(model, data)

    v = mj_viewer.launch_passive(model, data)
    v.cam.distance  = 2.5
    v.cam.azimuth   = 135
    v.cam.elevation = -25
    v.cam.lookat[:] = [0.3, 0.0, 0.3]
    return model, data, v, (l_qpos, l_qvel, l_ctrl), (r_qpos, r_qvel, r_ctrl)


def _run_mujoco_segment(model, data, v,
                        q1_start, q1_end,
                        q2_start, q2_end,
                        waypoint_duration,
                        left_idx, right_idx):
    """Step MuJoCo through one waypoint segment, controlling both arms simultaneously."""
    import mujoco as mj
    import time

    l_qpos, l_qvel, l_ctrl = left_idx
    r_qpos, r_qvel, r_ctrl = right_idx

    dt           = model.opt.timestep
    render_dt    = 1.0 / 60.0
    t            = 0.0
    t_wall_start = time.time()

    while t < waypoint_duration and v.is_running():
        t_wall_target = time.time() - t_wall_start
        while t < min(t_wall_target, waypoint_duration):
            # left arm
            q1_des, qd1_des = rt.interp_min_jerk(q1_start, q1_end, t, waypoint_duration)
            q1  = data.qpos[l_qpos].copy()
            qd1 = data.qvel[l_qvel].copy()
            tau1 = KP * (q1_des - q1) + KD * (qd1_des - qd1)
            data.ctrl[l_ctrl] = tau1 + data.qfrc_bias[l_qvel]
            # right arm
            q2_des, qd2_des = rt.interp_min_jerk(q2_start, q2_end, t, waypoint_duration)
            q2  = data.qpos[r_qpos].copy()
            qd2 = data.qvel[r_qvel].copy()
            tau2 = KP * (q2_des - q2) + KD * (qd2_des - qd2)
            data.ctrl[r_ctrl] = tau2 + data.qfrc_bias[r_qvel]

            mj.mj_step(model, data)
            t += dt
        v.sync()
        time.sleep(max(0.0, render_dt - (time.time() - t_wall_start - t_wall_target)))


# ── Execution modes ───────────────────────────────────────────────────────────

def execute_real_only(fa1, fa2, plan1, plan2, waypoint_duration):
    """Execute plans on both real robots (non-blocking per segment, synced after)."""
    n = max(len(plan1), len(plan2))
    print(f"Executing up to {n} waypoints...")
    for i in range(n - 1):
        if i < len(plan1) - 1:
            fa1.goto_joints(plan1[i + 1], duration=waypoint_duration, block=False)
        if i < len(plan2) - 1:
            fa2.goto_joints(plan2[i + 1], duration=waypoint_duration, block=False)
        if i < len(plan1) - 1:
            fa1.wait_for_skill()
        if i < len(plan2) - 1:
            fa2.wait_for_skill()
    print("Done.")


def execute_with_sim(fa1, fa2, plan1, plan2, waypoint_duration):
    """Execute both plans on real robots with MuJoCo visualization."""
    model, data, v, left_idx, right_idx = _setup_mujoco(plan1, plan2)
    n = max(len(plan1), len(plan2))
    print(f"Executing up to {n-1} segments (real robots + sim)...")
    for i in range(n - 1):
        q1_start = plan1[min(i,   len(plan1)-1)]
        q1_end   = plan1[min(i+1, len(plan1)-1)]
        q2_start = plan2[min(i,   len(plan2)-1)]
        q2_end   = plan2[min(i+1, len(plan2)-1)]
        if i < len(plan1) - 1:
            fa1.goto_joints(q1_end, duration=waypoint_duration, block=False)
        if i < len(plan2) - 1:
            fa2.goto_joints(q2_end, duration=waypoint_duration, block=False)
        _run_mujoco_segment(model, data, v,
                            q1_start, q1_end, q2_start, q2_end,
                            waypoint_duration, left_idx, right_idx)
        if i < len(plan1) - 1:
            fa1.wait_for_skill()
        if i < len(plan2) - 1:
            fa2.wait_for_skill()
    print("Done.")
    v.close()


def execute_sim_only(plan1, plan2, waypoint_duration):
    """Run MuJoCo visualization only — no real robot connection."""
    model, data, v, left_idx, right_idx = _setup_mujoco(plan1, plan2)
    n = max(len(plan1), len(plan2))
    print(f"Sim-only: up to {n-1} segments...")
    for i in range(n - 1):
        q1_start = plan1[min(i,   len(plan1)-1)]
        q1_end   = plan1[min(i+1, len(plan1)-1)]
        q2_start = plan2[min(i,   len(plan2)-1)]
        q2_end   = plan2[min(i+1, len(plan2)-1)]
        _run_mujoco_segment(model, data, v,
                            q1_start, q1_end, q2_start, q2_end,
                            waypoint_duration, left_idx, right_idx)
    print("Done.")
    v.close()


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="PRM-based point-to-point motion on the real Franka robot."
    )
    parser.add_argument("--sim", action="store_true",
                        help="Run MuJoCo visualization alongside the real robot")
    parser.add_argument("--sim-only", action="store_true",
                        help="Run MuJoCo visualization only (no real robot)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Seconds per waypoint (default: 5.0)")
    args = parser.parse_args()

    random.seed(13)

    mybot = SimpleFranka.SimpleFrankArm()  # offline collision checker — no robot connection

    print("Franka arm initialized in sim for collision checking")

    if args.sim_only:
        # No robot: use a hardcoded start config
        q_init_1 = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])
        q_init_2 = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])
    else:
        from frankapy import FrankaArm
        fa1     = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1)
        print("Connected to real Franka robot")
        fa2    = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)  # label_arm
        q_init_1 = fa1.get_joints()
        q_init_2 = fa2.get_joints()

    # ── Set your goal task position here (x, y, z in metres, arm base frame) ──
    task_pos_1 = [0.3, -0.5, 0.3]
    task_pos_2 = [0.5, 0.0, 0.3]

    q_goal_1 = compute_ik(task_pos_1)
    q_goal_2 = compute_ik(task_pos_2)
    # q_goal_2 = np.array([-0.3957843,  -1.61734739,  1.49835695, -2.58231489,  1.51090166,  1.57524887,  0.9455237 ])

    print(f"IK goal joints: {np.round(q_goal_1, 3)} and {np.round(q_goal_2, 3)}")
    # ──────────────────────────────────────────────────────────────────────────

    print(f"Arm1 start: {np.round(q_init_1, 3)}  goal: {np.round(q_goal_1, 3)}")
    print(f"Arm2 start: {np.round(q_init_2, 3)}  goal: {np.round(q_goal_2, 3)}")

    plan1 = prm_query(q_init_1, q_goal_1, mybot, PRM_FILE_ARM1)
    if plan1 is None:
        sys.exit(1)
    plan2 = prm_query(q_init_2, q_goal_2, mybot, PRM_FILE_ARM2)
    if plan2 is None:
        sys.exit(1)

    if args.sim_only:
        execute_sim_only(plan1, plan2, waypoint_duration=args.duration)
    elif args.sim:
        execute_with_sim(fa1, fa2, plan1, plan2, waypoint_duration=args.duration)
        fa1.reset_joints(block=False)
        fa2.reset_joints(block=False)
    else:
        execute_real_only(fa1, fa2, plan1, plan2, waypoint_duration=args.duration)
        fa1.reset_joints(block=False)
        fa2.reset_joints(block=False)
    
