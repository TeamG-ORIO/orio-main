#!/usr/bin/env python3
"""
PRM-based point-to-point motion on a single Franka arm.

Usage:
    from franka_prm_single_arm import SingleArmExecutor

    executor = SingleArmExecutor(arm_number=1)

    # Run directly (blocking):
    q_init = executor.fa.get_joints()
    executor.execute_real_only(q_init, taskPose_goal, label_zone=1)

    # Run as a thread (non-blocking):
    t = threading.Thread(target=executor.execute_real_only,
                         args=(q_init, taskPose_goal, 1))
    t.start()

Script usage (test / demo):
    python franka_prm_single_arm.py                  # real robot only
    python franka_prm_single_arm.py --sim            # real robot + MuJoCo viz
    python franka_prm_single_arm.py --sim-only       # MuJoCo only
"""

import argparse
import logging
import os
import time
import numpy as np
import pickle
import heapq
import random
import threading
from datetime import datetime
from scipy.spatial import KDTree
from scipy.interpolate import CubicSpline

import ikpy.chain
import RobotUtil as rt
import SimpleFranka  # offline kinematics/collision only

from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
import rospy

# ── Logging ───────────────────────────────────────────────────────────────────

_logger_configured = False

def _setup_logger(log_to_file=True):
    global _logger_configured
    if _logger_configured:
        return
    _logger_configured = True

    fmt       = "%(asctime)s [%(levelname)-8s] %(name)s: %(message)s"
    datefmt   = "%Y-%m-%dT%H:%M:%S"
    formatter = logging.Formatter(fmt, datefmt=datefmt)

    logger = logging.getLogger("franka_prm")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    logger.handlers.clear()

    sh = logging.StreamHandler()
    sh.setFormatter(formatter)
    logger.addHandler(sh)

    if log_to_file:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(script_dir, "logs")
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(log_dir, f"prm_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        fh = logging.FileHandler(fname)
        fh.setFormatter(formatter)
        logger.addHandler(fh)
        print(f"[logger] writing to {fname}")

log = logging.getLogger("franka_prm")
np.set_printoptions(suppress=True, precision=4)

# ── Constants ─────────────────────────────────────────────────────────────────

_URDF_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../orio/panda_arm_hand.urdf")

_ARM_SENSOR_TOPIC = {
    1: "/franka_ros_interface/sensor",
    2: "/franka_ros_interface_2/sensor",
}


# ── Collision debug visualizer ─────────────────────────────────────────────────

def _draw_box_edges(ax, points, color, lw=1.2, alpha=0.9):
    """Draw wireframe box. points[0] = center, points[1:9] = corners."""
    corners = np.array(points[1:])  # (8, 3)
    center  = np.array(points[0])
    half_vecs = corners - center
    _, _, Vt = np.linalg.svd(half_vecs)
    axis_dirs = Vt[:3]
    projs = half_vecs @ axis_dirs.T
    axis_extents = projs.max(axis=0) - projs.min(axis=0)
    tol = 1e-4
    for i in range(8):
        for j in range(i + 1, 8):
            diff = corners[i] - corners[j]
            diff_proj = np.abs(axis_dirs @ diff)
            nonzero = diff_proj > tol
            if nonzero.sum() == 1:
                k = np.argmax(nonzero)
                if abs(diff_proj[k] - axis_extents[k]) < tol * 10 + 1e-4:
                    ax.plot([corners[i, 0], corners[j, 0]],
                            [corners[i, 1], corners[j, 1]],
                            [corners[i, 2], corners[j, 2]],
                            color=color, linewidth=lw, alpha=alpha)


def plot_collision_debug(arm_number, q, obs_points, obs_axes, label="config"):
    """Pop up a 3-D plot showing which robot blocks and obstacles are in collision.

    Colours:
      blue        — arm link collision block (no collision)
      orange/red  — arm link collision block IN collision
      grey        — scene obstacle (no collision with this arm config)
      red         — scene obstacle IN collision with this arm config
    """
    import matplotlib.pyplot as plt

    cc = SimpleFranka.SimpleFrankArm(arm_number)
    cc.CompCollisionBlockPoints(q)

    # Determine which (robot_block, obstacle) pairs collide
    colliding_robot_blocks = set()
    colliding_obstacles    = set()
    for i in range(len(cc.Cpoints)):
        for j in range(len(obs_points)):
            if rt.CheckBoxBoxCollision(cc.Cpoints[i], cc.Caxes[i], obs_points[j], obs_axes[j]):
                colliding_robot_blocks.add(i)
                colliding_obstacles.add(j)

    fig = plt.figure(figsize=(11, 8))
    ax  = fig.add_subplot(111, projection='3d')

    # Arm skeleton
    for i in range(len(cc.Tcurr)):
        p = cc.Tcurr[i]
        ax.scatter(p[0, 3], p[1, 3], p[2, 3], c='k', s=20, zorder=5)
        if i == 0:
            ax.plot([0, p[0, 3]], [0, p[1, 3]], [0, p[2, 3]], c='k', lw=1.5)
        else:
            pp = cc.Tcurr[i - 1]
            ax.plot([pp[0, 3], p[0, 3]], [pp[1, 3], p[1, 3]], [pp[2, 3], p[2, 3]], c='k', lw=1.5)

    # Robot collision blocks
    for i, pts in enumerate(cc.Cpoints):
        if i in colliding_robot_blocks:
            color, lw = 'orangered', 2.0
        else:
            color, lw = 'steelblue', 1.0
        _draw_box_edges(ax, pts, color=color, lw=lw)

    # Obstacle blocks
    for j, pts in enumerate(obs_points):
        if j in colliding_obstacles:
            color, lw = 'red', 2.0
        else:
            color, lw = 'dimgrey', 1.0
        _draw_box_edges(ax, pts, color=color, lw=lw)

    from matplotlib.lines import Line2D
    legend = [
        Line2D([0], [0], color='steelblue',  lw=2, label='Arm block (clear)'),
        Line2D([0], [0], color='orangered',  lw=2, label='Arm block (IN COLLISION)'),
        Line2D([0], [0], color='dimgrey',    lw=2, label='Obstacle (clear)'),
        Line2D([0], [0], color='red',        lw=2, label='Obstacle (IN COLLISION)'),
        Line2D([0], [0], color='black',      lw=2, label='Arm skeleton'),
    ]
    ax.legend(handles=legend, loc='upper left', fontsize=8)
    ax.set_title(f"Arm {arm_number} collision debug — {label}\n"
                 f"q={np.round(q, 3)}\n"
                 f"{len(colliding_robot_blocks)} arm block(s) and "
                 f"{len(colliding_obstacles)} obstacle(s) in collision",
                 fontsize=9)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')

    all_pts = [np.array(p[1:]) for p in cc.Cpoints] + [np.array(p[1:]) for p in obs_points]
    all_pts = np.concatenate(all_pts)
    lo, hi  = all_pts.min(axis=0), all_pts.max(axis=0)
    mid     = (lo + hi) / 2
    span    = (hi - lo).max() / 2 * 1.2
    ax.set_xlim(mid[0] - span, mid[0] + span)
    ax.set_ylim(mid[1] - span, mid[1] + span)
    ax.set_zlim(0, mid[2] + span)

    plt.tight_layout()
    plt.show(block=True)


# ── Single-Arm Executor ────────────────────────────────────────────────────────

class SingleArmExecutor:
    """Owns a single FrankaArm handle, its collision checker, and ROS publisher.

    Instantiate with an arm number; all execution methods handle IK, PRM
    planning, and trajectory streaming for that arm.  Every public method
    can be run directly or wrapped in a threading.Thread for async use:

        t = threading.Thread(target=executor.execute_real_only,
                             args=(init_pose, goal_pose, label_zone))
        t.start()
    """

    N_RAMP         = 50    # points for ramp-up and ramp-down phases
    N_MID          = 20    # points per mid-trajectory segment
    STREAM_RATE_HZ = 1000.0  # ROS publish rate (Hz)
    FILLET_RADIUS  = 0.5  # joint-space fillet blend radius in radians (spline mode)

    def __init__(self, arm_number, traj_type='ramp', duration=5.0,
                 log_fk=False, init_node=True, sim_only=False, totg_scale=0.25,
                 spline_speed=None):
        """
        arm_number   : 1 (left) or 2 (right)
        traj_type    : 'ramp'    – quadratic ramp-up/down with linear mid segments
                       'minjerk' – 5th-order min-jerk polynomial
                       'spline'  – joint-space fillets + cubic spline, C2 continuous
                       'totg'    – time-optimal bang-bang acceleration profile respecting
                                   Franka per-joint velocity and acceleration limits
        duration     : total motion time in seconds (used by 'minjerk' and 'spline'
                       when spline_speed is None)
        log_fk       : if True, FK poses are buffered and saved to CSV after execution
        init_node    : pass False if rospy.init_node() has already been called elsewhere
        sim_only     : if True, skip real robot connection (collision checker still loads)
        totg_scale   : scale factor applied to both V_MAX and A_MAX in 'totg' mode
                       (e.g. 0.5 = half speed/acceleration, 1.0 = full limits)
        spline_speed : average path speed in rad/s of joint-space arc length for
                       'spline' mode.  When set, duration = total_arc / spline_speed
                       is computed automatically from each path, overriding 'duration'.
                       When None, 'duration' is used as before.
        """
        assert arm_number in (1, 2), f"arm_number must be 1 or 2, got {arm_number}"
        assert traj_type in ('ramp', 'minjerk', 'spline', 'totg'), f"Unknown traj_type '{traj_type}'"
        assert 0.0 < totg_scale <= 1.0, f"totg_scale must be in (0, 1], got {totg_scale}"
        assert spline_speed is None or spline_speed > 0.0, \
            f"spline_speed must be positive, got {spline_speed}"

        self.arm_number   = arm_number
        self.traj_type    = traj_type
        self.duration     = duration
        self.log_fk       = log_fk
        self.totg_scale   = totg_scale
        self.spline_speed = spline_speed

        _setup_logger()

        self.collision_checker = SimpleFranka.SimpleFrankArm(arm_number)
        log.info("Arm%d: offline collision checker initialized", arm_number)

        self.ik_chain = ikpy.chain.Chain.from_urdf_file(_URDF_FILE, base_elements=["panda_link0"])
        _cl = len(self.ik_chain.links)
        self.ik_chain.active_links_mask = [False] + [True]*7 + [False]*(_cl - 8)
        self._ik_chain_len = _cl

        if sim_only:
            self.fa  = None
            self.pub = None
            log.info("Arm%d: sim-only mode — skipping real robot connection", arm_number)
        else:
            self.fa = FrankaArm(with_gripper=False, old_gripper=False,
                                robot_num=arm_number, init_node=init_node)
            log.info("Arm%d: connected to real robot", arm_number)
            self.pub = rospy.Publisher(_ARM_SENSOR_TOPIC[arm_number],
                                       SensorDataGroup, queue_size=1000)

    # ── FK helper ─────────────────────────────────────────────────────────────

    def _fk_mat(self, q7):
        """Return the 4x4 EE transform for a 7-DOF joint vector using ikpy."""
        full = [0.0] + list(q7) + [0.0] * (self._ik_chain_len - 8)
        return self.ik_chain.forward_kinematics(full)

    # ── IK ────────────────────────────────────────────────────────────────────

    def _compute_ik(self, task_pos, yaw=0.0):
        """Compute 7-DOF joint angles for a Cartesian task position and yaw using ikpy.

        Args:
            task_pos : [x, y, z] in metres in the arm base frame
            yaw      : end-effector rotation about the world Z axis (radians, default 0)
        """
        ik_chain = self.ik_chain
        chain_length = self._ik_chain_len

        cy, sy = np.cos(yaw), np.sin(yaw)
        # Base orientation: pointing straight down, rotated by yaw about Z
        target_ori = np.array([[ cy, -sy, 0.0],
                                [ sy,  cy, 0.0],
                                [ 0.0, 0.0, 1.0]]) @ np.array([[1.0, 0.0, 0.0],
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
        result = angles[1:8]
        fk_pos = ik_chain.forward_kinematics(angles)[:3, 3]
        err = np.linalg.norm(fk_pos - np.array(task_pos))
        if err > 0.01:
            log.warning("IK position error %.4f m  (target=%s  fk=%s)", err,
                        np.round(task_pos, 4), np.round(fk_pos, 4))
        else:
            log.info("IK solved  joints=%s  pos_err=%.4f m", np.round(result, 3), err)
        return result

    # ── PRM ───────────────────────────────────────────────────────────────────

    def _get_prm_file(self, label_zone):
        """Return the PRM file path for this arm and label zone.

        label_zone=None  →  prm_files/myPRM_arm{n}_none.p
        label_zone=k     →  prm_files/myPRM_arm{n}_labelZone{k}.p
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if label_zone is None:
            fname = f"myPRM_arm{self.arm_number}_none.p"
        elif (label_zone == 1 or label_zone == 2):
            fname = f"myPRM_arm{self.arm_number}_labelZone{label_zone}.p"
        else:
            fname = f"myPRM_arm{self.arm_number}_free.p"
        return os.path.join(script_dir, "prm_files_10k", fname)

    def check_collision(self, q7, label_zone=None):
        """Return True if the 7-DOF config q7 is in collision with scene obstacles.

        Obstacles are loaded from the PRM file for this arm / label_zone and
        cached after the first call so subsequent checks are fast.

        Args:
            q7         : 7-element joint angle array.
            label_zone : None | 1 | 2  — selects the same PRM file that plan_path
                         would use for this zone (default None).
        """
        cache_key = label_zone
        if not hasattr(self, '_obs_cache'):
            self._obs_cache = {}
        if cache_key not in self._obs_cache:
            prm_file = self._get_prm_file(label_zone)
            with open(prm_file, 'rb') as f:
                pickle.load(f)  # prm_vertices
                pickle.load(f)  # prm_edges
                obs_points = pickle.load(f)
                obs_axes   = pickle.load(f)
            self._obs_cache[cache_key] = (obs_points, obs_axes)
            log.info("Arm%d: cached %d obstacles from '%s'",
                     self.arm_number, len(obs_points), prm_file)
        obs_points, obs_axes = self._obs_cache[cache_key]
        return self.collision_checker.DetectCollision(q7, obs_points, obs_axes)

    def _log_collision_detail(self, label, q, obs_points, obs_axes):
        """Log which robot link block collides with which obstacle, and show debug plot."""
        cc = self.collision_checker
        cc.CompCollisionBlockPoints(q)
        for i in range(len(cc.Cpoints)):
            for j in range(len(obs_points)):
                if rt.CheckBoxBoxCollision(cc.Cpoints[i], cc.Caxes[i], obs_points[j], obs_axes[j]):
                    joint_frame = cc.Cidx[i] if i < len(cc.Cidx) else '?'
                    link_center = np.round(cc.Cpoints[i][0], 3)
                    obs_center  = np.round(obs_points[j][0], 3)
                    log.error(
                        "  [%s] robot block %d (joint_frame=%s, center=%s) collides with obstacle %d (center=%s)",
                        label, i, joint_frame, link_center, j, obs_center
                    )
        if self.log_fk:
            plot_collision_debug(self.arm_number, q, obs_points, obs_axes, label=label)

    def _prm_query(self, q_init, q_goal, prm_file):
        """Plan a path from q_init to q_goal using a pre-built PRM.

        Returns a list of waypoints (numpy arrays), or None if planning fails.
        """
        with open(prm_file, 'rb') as f:
            prm_vertices = pickle.load(f)
            prm_edges    = pickle.load(f)
            obs_points   = pickle.load(f)
            obs_axes     = pickle.load(f)

        num_nodes, num_edges, num_components = rt.AnalyzeGraph(prm_vertices, prm_edges)
        log.info("PRM loaded from '%s': %d nodes, %d edges, %d components",
                 prm_file, num_nodes, num_edges, num_components)
        log.info("  obstacles in PRM: %d", len(obs_points))

        cc = self.collision_checker

        # Log FK end-effector positions for start and goal
        # Tcurr_init, _ = cc.ForwardKin(q_init)
        ee_init = np.round(self._fk_mat(q_init)[:3, 3], 4)
        # Tcurr_goal, _ = cc.ForwardKin(q_goal)
        ee_goal = np.round(self._fk_mat(q_goal)[:3, 3], 4)
        log.info("  Arm%d FK  q_init -> EE pos %s", self.arm_number, ee_init)
        log.info("  Arm%d FK  q_goal -> EE pos %s", self.arm_number, ee_goal)

        # Pre-convert vertices to a numpy array for fast vectorized ops
        verts = np.array(prm_vertices)  # (N, 7)
        kd_tree = KDTree(verts)

        def find_neighbors(q, label):
            q_arr = np.asarray(q)
            # Query more candidates than needed to account for edge collisions
            k = min(200, len(prm_vertices))
            _, candidate_indices = kd_tree.query(q_arr, k=k)
            neighbors = []
            edge_blocked = 0
            for i in candidate_indices:
                if not cc.DetectCollisionEdge(prm_vertices[i], q, obs_points, obs_axes):
                    neighbors.append(int(i))
                else:
                    edge_blocked += 1
                if len(neighbors) >= 5:
                    break
            log.info(" Arm%d find_neighbors(%s): %d found, %d edge-blocked among top-%d candidates",
                     self.arm_number, label, len(neighbors), edge_blocked, k)
            return neighbors

        if cc.DetectCollision(q_init, obs_points, obs_axes):
            log.error("Start configuration is in collision: %s", np.round(q_init, 3))
            self._log_collision_detail("q_init", q_init, obs_points, obs_axes)
            return None
        if cc.DetectCollision(q_goal, obs_points, obs_axes):
            log.error("Goal configuration is in collision: %s", np.round(q_goal, 3))
            self._log_collision_detail("q_goal", q_goal, obs_points, obs_axes)
            return None

        init_neighbors = find_neighbors(q_init, "init")
        goal_neighbors = find_neighbors(q_goal, "goal")
        log.info("Neighbors found — init: %d, goal: %d", len(init_neighbors), len(goal_neighbors))
        if not init_neighbors or not goal_neighbors:
            log.error("Could not connect start/goal to PRM (init=%d, goal=%d)",
                      len(init_neighbors), len(goal_neighbors))
            return None

        # Vectorized heuristic: distance from every vertex to q_goal
        q_goal_arr = np.asarray(q_goal)
        q_init_arr = np.asarray(q_init)
        heuristic = np.linalg.norm(verts - q_goal_arr, axis=1)

        g_cost = np.full(len(prm_vertices), np.inf)
        parent = [None] * len(prm_vertices)

        open_set = []
        for n in init_neighbors:
            g = float(np.linalg.norm(verts[n] - q_init_arr))
            g_cost[n] = g
            heapq.heappush(open_set, (g + heuristic[n], n))

        goal_set  = set(goal_neighbors)
        closed_set = set()
        goal_node  = None
        while open_set:
            _, curr = heapq.heappop(open_set)
            if curr in closed_set:
                continue
            closed_set.add(curr)
            if curr in goal_set:
                goal_node = curr
                break
            curr_g = g_cost[curr]
            curr_v = verts[curr]
            for nb in prm_edges[curr]:
                if nb in closed_set:
                    continue
                tg = curr_g + float(np.linalg.norm(verts[nb] - curr_v))
                if tg < g_cost[nb]:
                    g_cost[nb] = tg
                    parent[nb] = curr
                    heapq.heappush(open_set, (tg + heuristic[nb], nb))

        if goal_node is None:
            reached_goal_neighbors = closed_set & goal_set
            log.error("A* failed to find a path (expanded %d nodes, goal_neighbors=%s, reached=%d)",
                      len(closed_set), goal_neighbors, len(reached_goal_neighbors))
            log.error("  init_neighbors=%s", init_neighbors)
            return None

        path = []
        node = goal_node
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()

        plan = ([np.array(q_init)]
                + [np.array(prm_vertices[i]) for i in path]
                + [np.array(q_goal)])

        # Stage 1: Node-based shortcutting — try to skip existing waypoints
        for _ in range(200):
            if len(plan) <= 2:
                break
            i = random.randint(0, len(plan) - 3)
            j = random.randint(i + 2, len(plan) - 1)
            if not cc.DetectCollisionEdge(plan[i], plan[j], obs_points, obs_axes):
                plan = plan[:i+1] + plan[j:]
        # log.info("After stage-1 shortcutting: %d waypoints", len(plan))

        # # Stage 2: Continuous shortcutting — sample arbitrary points along the path
        # # and try to connect them directly, bypassing intermediate waypoints
        # def _path_length(p):
        #     return sum(np.linalg.norm(p[k+1] - p[k]) for k in range(len(p) - 1))

        # for _ in range(200):
        #     if len(plan) <= 2:
        #         break
        #     # Pick two random parameter values t_a < t_b in [0, 1]
        #     t_a, t_b = sorted(random.uniform(0, 1) for _ in range(2))
        #     if t_b - t_a < 1e-6:
        #         continue

        #     # Interpolate a joint config at parameter t along the path
        #     def _interp_at(p, t):
        #         total = _path_length(p)
        #         if total == 0:
        #             return p[0].copy()
        #         target = t * total
        #         dist = 0.0
        #         for k in range(len(p) - 1):
        #             seg = float(np.linalg.norm(p[k+1] - p[k]))
        #             if dist + seg >= target or k == len(p) - 2:
        #                 s = (target - dist) / seg if seg > 1e-9 else 0.0
        #                 return p[k] + s * (p[k+1] - p[k])
        #             dist += seg
        #         return p[-1].copy()

        #     q_a = _interp_at(plan, t_a)
        #     q_b = _interp_at(plan, t_b)

        #     if cc.DetectCollisionEdge(q_a, q_b, obs_points, obs_axes):
        #         continue

        #     # Find which waypoint indices bracket t_a and t_b
        #     total = _path_length(plan)
        #     if total == 0:
        #         break
        #     cumulative = [0.0]
        #     for k in range(len(plan) - 1):
        #         cumulative.append(cumulative[-1] + float(np.linalg.norm(plan[k+1] - plan[k])))
        #     norm_cum = [c / total for c in cumulative]

        #     idx_a = next(k for k in range(len(norm_cum) - 1) if norm_cum[k] <= t_a <= norm_cum[k+1])
        #     idx_b = next(k for k in range(len(norm_cum) - 1) if norm_cum[k] <= t_b <= norm_cum[k+1])

        #     if idx_b <= idx_a:
        #         continue

        #     # Replace the segment [idx_a+1 .. idx_b] with the two sampled points
        #     plan = plan[:idx_a+1] + [q_a, q_b] + plan[idx_b+1:]

        log.info("Plan ready: %d waypoints after shortcutting", len(plan))
        return plan

    def _plan_from_poses(self, q_init, taskPose_goal, label_zone, goal_yaw=0.0):
        """IK (goal only) + PRM planning pipeline. Returns waypoint list or None on failure.

        Args:
            q_init        : 7-element joint array — current robot state, no IK needed
            taskPose_goal : [x, y, z] goal Cartesian position in arm base frame
            label_zone    : PRM label zone index, or None for free-space PRM
            goal_yaw      : end-effector yaw at the goal (radians, default 0)
        """
        t0 = time.time()
        q_goal = self._compute_ik(taskPose_goal, yaw=goal_yaw)
        log.info("Arm%d  q_init=%s  q_goal=%s  [IK: %.3fs]",
                 self.arm_number, np.round(q_init, 3), np.round(q_goal, 3), time.time() - t0)
        prm_file = self._get_prm_file(label_zone)
        log.info("Arm%d: using PRM file '%s'", self.arm_number, prm_file)
        t1 = time.time()
        result = self._prm_query(q_init, q_goal, prm_file)
        log.info("Arm%d: PRM query took %.3fs", self.arm_number, time.time() - t1)
        return result
    
    def _plan_from_joints(self, q_init, q_goal, label_zone, goal_yaw=0.0):
        """PRM planning pipeline. Returns waypoint list or None on failure.

        Args:
            q_init        : 7-element joint array - current robot state, no IK needed
            q_goal        : 7-element joint array - Goal state
            label_zone    : PRM label zone index (1 or 2), or None for both full PRM, or 3 for both free
            goal_yaw      : end-effector yaw at the goal (radians, default 0)
        """
        log.info("Arm%d  q_init=%s  q_goal=%s",
                 self.arm_number, np.round(q_init, 3), np.round(q_goal, 3))
        prm_file = self._get_prm_file(label_zone)
        log.info("Arm%d: using PRM file '%s'", self.arm_number, prm_file)
        return self._prm_query(q_init, q_goal, prm_file)

    # ── Trajectory building ───────────────────────────────────────────────────

    def _est_duration(self, plan):
        n   = len(plan)
        pts = self.N_RAMP + max(0, n - 3) * self.N_MID + self.N_RAMP
        return pts / self.STREAM_RATE_HZ

    def _build_interpolated_traj(self, joints_traj):
        if self.traj_type == 'minjerk':
            return self._build_minjerk_traj(joints_traj)
        if self.traj_type == 'spline':
            return self._build_spline_traj(joints_traj)
        if self.traj_type == 'totg':
            return self._build_totg_traj(joints_traj)
        return self._build_ramp_traj(joints_traj)

    def _build_ramp_traj(self, joints_traj):
        """Quadratic ramp-up → linear mid segments → quadratic ramp-down."""
        N_RAMP = self.N_RAMP
        N_MID  = self.N_MID

        t_mid       = np.linspace(1/N_MID,  1, N_MID)
        t_ramp      = np.linspace(1/N_RAMP, 1, N_RAMP)
        t_ramp_up   = t_ramp**2
        t_ramp_down = 1 - (1-t_ramp)**2

        interp = [joints_traj[0, :]]

        for t_i in range(len(t_ramp_up)):
            dt = t_ramp_up[t_i]
            interp.append(joints_traj[1, :]*dt + joints_traj[0, :]*(1-dt))

        for i in range(2, joints_traj.shape[0]-1):
            for t_i in range(len(t_mid)):
                dt = t_mid[t_i]
                interp.append(joints_traj[i, :]*dt + joints_traj[i-1, :]*(1-dt))

        for t_i in range(len(t_ramp_down)):
            dt = t_ramp_down[t_i]
            interp.append(joints_traj[-1, :]*dt + joints_traj[-2, :]*(1-dt))

        return np.array(interp)

    def _build_minjerk_traj(self, joints_traj):
        """Min-jerk polynomial through each consecutive pair of PRM waypoints."""
        N = joints_traj.shape[0]
        total_pts = max(2, round(self.duration * self.STREAM_RATE_HZ))

        distances = np.array([
            np.linalg.norm(joints_traj[i+1] - joints_traj[i])
            for i in range(N - 1)
        ])
        total_dist = distances.sum()

        interp = [joints_traj[0, :]]
        for i in range(N - 1):
            q0 = joints_traj[i, :]
            qf = joints_traj[i+1, :]
            if total_dist > 0:
                n = max(2, round(total_pts * distances[i] / total_dist))
            else:
                n = max(2, total_pts // (N - 1))
            t = np.linspace(0.0, 1.0, n + 1)[1:]
            w = 10*t**3 - 15*t**4 + 6*t**5
            interp.append(q0 + np.outer(w, qf - q0))

        return np.vstack(interp)

    def _apply_fillets(self, plan):
        """Replace sharp corners at intermediate waypoints with circular fillets.

        At each interior waypoint q_i, the incoming segment (q_{i-1} -> q_i) and
        outgoing segment (q_i -> q_{i+1}) are trimmed by fillet_radius in joint-space
        distance. The fillet entry point `a` and exit point `b` are inserted in place
        of q_i, and n_fillet points are sampled along the circular arc between them.

        Waypoints where the adjacent segments are too short to accommodate the radius
        are left as-is (sharp corner preserved with a warning).

        Returns a list of numpy arrays representing the filleted path.
        """
        r = self.FILLET_RADIUS
        n_fillet = 8  # arc sample points per fillet

        if len(plan) <= 2:
            return [np.asarray(p) for p in plan]

        result = [np.asarray(plan[0])]

        for i in range(1, len(plan) - 1):
            q_prev = np.asarray(plan[i - 1])
            q_curr = np.asarray(plan[i])
            q_next = np.asarray(plan[i + 1])

            d_in  = float(np.linalg.norm(q_curr - q_prev))
            d_out = float(np.linalg.norm(q_next - q_curr))

            # Clamp radius so it never consumes more than half of either segment
            r_eff = min(r, d_in * 0.5, d_out * 0.5)
            if r_eff < 1e-6:
                log.warning("Fillet at waypoint %d skipped (segments too short: %.4f, %.4f)",
                            i, d_in, d_out)
                result.append(q_curr)
                continue

            t_in  = (q_curr - q_prev) / d_in   # unit tangent into corner
            t_out = (q_next - q_curr) / d_out   # unit tangent out of corner

            a = q_curr - r_eff * t_in   # fillet entry
            b = q_curr + r_eff * t_out  # fillet exit

            # Sample the arc: interpolate from a to b via the corner q_curr using
            # a quadratic Bezier (a, q_curr, b) which gives a smooth blend
            fillet_pts = []
            for s in np.linspace(0.0, 1.0, n_fillet + 2)[1:-1]:
                # Quadratic Bezier: (1-s)^2 * a + 2(1-s)s * q_curr + s^2 * b
                pt = (1 - s)**2 * a + 2 * (1 - s) * s * q_curr + s**2 * b
                fillet_pts.append(pt)

            result.append(a)
            result.extend(fillet_pts)
            result.append(b)

        result.append(np.asarray(plan[-1]))
        log.info("Fillets applied: %d waypoints -> %d points (r=%.4f rad)",
                 len(plan), len(result), r)
        return result

    def _build_spline_traj(self, joints_traj):
        """Fillet corners then fit a C2 cubic spline, re-sampled with a
        trapezoidal (ramp-up / cruise / ramp-down) speed profile.

        Steps:
          1. Apply joint-space fillets at every intermediate PRM waypoint.
          2. Parameterize the filleted path by cumulative arc length.
          3. Fit a CubicSpline (clamped zero-velocity endpoints).
          4. Re-sample using a trapezoidal velocity profile in time so the
             robot accelerates smoothly, cruises, then decelerates — rather
             than moving at constant speed along the path.
        """
        plan = [joints_traj[i] for i in range(joints_traj.shape[0])]
        filleted = self._apply_fillets(plan)

        # Build cumulative arc-length parameter
        pts = np.array(filleted)  # (M, 7)
        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(dists)])
        total_arc = arc[-1]
        if total_arc < 1e-9:
            return pts  # degenerate: start == goal

        # Normalize arc to [0, 1] for numerical stability
        s = arc / total_arc

        # Remove duplicate s values (can occur when consecutive fillet points are
        # numerically identical), which would cause CubicSpline to raise ValueError.
        unique_mask = np.concatenate([[True], np.diff(s) > 0])
        s = s[unique_mask]
        pts = pts[unique_mask]

        # Fit cubic spline with clamped (zero velocity) endpoints for smooth start/stop
        cs = CubicSpline(s, pts, bc_type=((1, np.zeros(pts.shape[1])),
                                          (1, np.zeros(pts.shape[1]))))

        if self.spline_speed is not None:
            duration = total_arc / self.spline_speed
            log.info("Spline duration from path length: %.4f rad / %.4f rad/s = %.3fs",
                     total_arc, self.spline_speed, duration)
        else:
            duration = self.duration

        total_pts = max(2, round(duration * self.STREAM_RATE_HZ))
        t_uniform = np.linspace(0.0, duration, total_pts)

        # Trapezoidal velocity profile: ramp up for t_ramp, cruise, ramp down for t_ramp.
        # Uses 20% of duration for each ramp (capped so 2*t_ramp <= duration).
        t_ramp = min(0.01 * duration, duration / 2.0)
        t_cruise = duration - 2.0 * t_ramp
        # Peak speed such that total arc travelled = 1.0 (normalised)
        v_peak = 1.0 / (t_cruise + t_ramp)

        def _s_of_t(t):
            """Arc-length fraction s in [0,1] for scalar time t."""
            if t <= t_ramp:
                # Ramp up: s = 0.5 * a * t^2,  a = v_peak / t_ramp
                return 0.5 * v_peak / t_ramp * t ** 2
            elif t <= t_ramp + t_cruise:
                # Cruise: s = area of ramp triangle + v_peak*(t - t_ramp)
                return 0.5 * v_peak * t_ramp + v_peak * (t - t_ramp)
            else:
                # Ramp down (mirror of ramp up from the end)
                t_rem = duration - t
                return 1.0 - 0.5 * v_peak / t_ramp * t_rem ** 2

        s_trap = np.array([_s_of_t(t) for t in t_uniform])
        s_trap = np.clip(s_trap, 0.0, 1.0)
        traj = cs(s_trap)

        log.info("Spline traj (trapezoidal): %d filleted pts -> %d stream pts "
                 "(%.1fs, t_ramp=%.2fs @ %.0fHz)",
                 len(filleted), total_pts, duration, t_ramp, self.STREAM_RATE_HZ)
        return traj

    def _build_totg_traj(self, joints_traj):
        """Time-Optimal Trajectory Generation respecting Franka Panda per-joint limits.

        Pipeline:
          1. Apply joint-space circular fillets (_apply_fillets) at every intermediate
             PRM waypoint to avoid hard velocity stops in the middle of the path.
          2. Parameterise the filleted path by cumulative arc length and fit a cubic
             spline to get a smooth continuous path q(s).
          3. For each joint j, compute the time-optimal bang-bang speed profile along
             the arc-length axis subject to:
               |dq_j/dt| <= V_MAX[j]   and   |d²q_j/dt²| <= A_MAX[j]
             The per-joint constraint translates to a limit on ds/dt:
               |ds/dt| <= V_MAX[j] / |dq_j/ds|   (velocity)
               (acceleration limit handled via the bang-bang ramp)
          4. The global speed limit at every arc-length position is the minimum over
             all joint velocity limits: v_lim(s) = min_j( V_MAX[j] / |dq_j/ds(s)| ).
          5. A bang-bang acceleration profile is solved for the resulting 1-D problem
             (scalar speed along arc length), then the path is re-sampled at
             STREAM_RATE_HZ.

        Franka Emika Panda joint limits (from Franka documentation):
          Joint  |  v_max (rad/s)  |  a_max (rad/s²)
          -------|-----------------|----------------
            1    |     2.175       |      15.0
            2    |     2.175       |       7.5
            3    |     2.175       |      10.0
            4    |     2.175       |      12.5
            5    |     2.610       |      15.0
            6    |     2.610       |      20.0
            7    |     2.610       |      20.0

        The trajectory starts and ends at zero velocity (safe for dynamic streaming).
        """
        # Franka Panda per-joint limits, scaled by totg_scale
        V_MAX = self.totg_scale * np.array([2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610])
        A_MAX = self.totg_scale * np.array([15.0,   7.5,  10.0,  12.5,  15.0,  20.0,  20.0])

        dt = 1.0 / self.STREAM_RATE_HZ

        # ── Step 1: fillet corners ─────────────────────────────────────────────
        plan = [joints_traj[i] for i in range(joints_traj.shape[0])]
        filleted = self._apply_fillets(plan)
        pts = np.array(filleted)          # (M, 7)

        # ── Step 2: arc-length parameterisation + cubic spline ─────────────────
        diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        arc   = np.concatenate([[0.0], np.cumsum(diffs)])
        S     = arc[-1]
        if S < 1e-9:
            log.warning("TOTG: start == goal, returning single-point trajectory")
            return pts[:1]

        s_norm = arc / S
        unique_mask = np.concatenate([[True], np.diff(s_norm) > 0])
        s_norm = s_norm[unique_mask]
        pts    = pts[unique_mask]

        cs = CubicSpline(s_norm, pts,
                         bc_type=((1, np.zeros(pts.shape[1])),
                                  (1, np.zeros(pts.shape[1]))))

        # ── Step 3: velocity limit field along the path ────────────────────────
        # Sample dq/ds at many arc-length points; derive per-joint speed limit.
        N_SAMPLES = max(500, int(S / 0.001))   # ~1 mm spacing, at least 500
        s_sample  = np.linspace(0.0, 1.0, N_SAMPLES)
        dqds      = cs(s_sample, 1)             # (N_SAMPLES, 7)  — derivative wrt s_norm

        # Convert derivative wrt normalised s to wrt physical arc length s
        dqds_phys = dqds / S                    # dq/ds_phys = dq/ds_norm * 1/S

        # Speed limit (scalar ds_phys/dt) from velocity constraints
        abs_dqds = np.abs(dqds_phys)            # (N_SAMPLES, 7)
        with np.errstate(divide='ignore'):
            v_lim_per_joint = np.where(abs_dqds > 1e-9,
                                       V_MAX[np.newaxis, :] / abs_dqds,
                                       np.inf)
        v_lim = v_lim_per_joint.min(axis=1)     # (N_SAMPLES,)  global speed limit

        # ── Step 4: bang-bang profile in arc-length space ─────────────────────
        # Acceleration limit: use the most conservative per-joint constraint.
        # a_path = min_j( A_MAX[j] / |dq_j/ds_phys| ), floored to avoid inf.
        with np.errstate(divide='ignore'):
            a_lim_per_joint = np.where(abs_dqds > 1e-9,
                                       A_MAX[np.newaxis, :] / abs_dqds,
                                       np.inf)
        a_lim = a_lim_per_joint.min(axis=1)     # (N_SAMPLES,)

        # Use a single conservative scalar for the bang-bang planner:
        # take the 5th-percentile of a_lim and v_lim to avoid edge effects.
        finite_a = a_lim[np.isfinite(a_lim)]
        finite_v = v_lim[np.isfinite(v_lim)]
        a_path = float(np.percentile(finite_a, 5)) if len(finite_a) else 1.0
        v_path = float(np.percentile(finite_v, 5)) if len(finite_v) else 1.0
        a_path = max(a_path, 0.01)
        v_path = max(v_path, 0.001)

        # Solve bang-bang (0 → 0 velocity) for total arc length S
        d_accel = v_path ** 2 / a_path          # distance to reach v_path and back
        if S >= d_accel:
            t_ramp   = v_path / a_path
            t_cruise = (S - d_accel) / v_path
            T_total  = 2.0 * t_ramp + t_cruise
        else:
            v_peak  = np.sqrt(a_path * S)
            t_ramp  = v_peak / a_path
            t_cruise = 0.0
            T_total  = 2.0 * t_ramp

        # ── Step 5: re-sample path at uniform time steps ──────────────────────
        n_pts  = max(2, int(np.ceil(T_total * self.STREAM_RATE_HZ)))
        t_vals = np.linspace(0.0, T_total, n_pts)

        # Map time → arc-length position s_phys(t)
        s_phys = np.empty(n_pts)
        v_peak_actual = v_path if S >= d_accel else np.sqrt(a_path * S)
        for k, t in enumerate(t_vals):
            if t <= t_ramp:
                s_phys[k] = 0.5 * a_path * t ** 2
            elif t <= t_ramp + t_cruise:
                s_phys[k] = 0.5 * a_path * t_ramp ** 2 + v_peak_actual * (t - t_ramp)
            else:
                tau = t - (t_ramp + t_cruise)
                s_phys[k] = (0.5 * a_path * t_ramp ** 2
                             + v_peak_actual * t_cruise
                             + v_peak_actual * tau
                             - 0.5 * a_path * tau ** 2)

        s_phys = np.clip(s_phys, 0.0, S)
        s_eval = s_phys / S                      # back to normalised [0, 1]
        traj   = cs(s_eval)                      # (n_pts, 7)

        log.info(
            "TOTG traj: %d waypoints -> %d filleted pts -> %d stream pts "
            "(est. %.2fs @ %.0fHz, v_path=%.3f rad/s, a_path=%.3f rad/s²)",
            joints_traj.shape[0], len(filleted), n_pts,
            T_total, self.STREAM_RATE_HZ, v_path, a_path
        )
        return traj

    # ── Low-level plan execution ───────────────────────────────────────────────

    def _execute_plan(self, joints_traj):
        """Stream an interpolated joint trajectory to this arm via ROS.

        joints_traj shape: (N x 7).  Safe to call from any thread.
        """
        name = f"arm{self.arm_number}"
        try:
            interpolated_traj = self._build_interpolated_traj(joints_traj)
            
            # Dwell at q_goal for an extra 20% of the trajectory length so the
            # robot fully settles before termination.
            dwell_pts = int(0.5 * (interpolated_traj.shape[0]))
            log.info("[%s] interpolated_traj[-1] (dwell target) = %s", name, np.round(interpolated_traj[-1], 4))
            log.info("[%s] joints_traj[-1]       (q_goal)       = %s", name, np.round(joints_traj[-1], 4))
            log.info("[%s] dwell delta from q_goal: %s", name, np.round(interpolated_traj[-1] - joints_traj[-1], 4))
            dwell = np.tile(interpolated_traj[-1], (dwell_pts, 1))
            interpolated_traj = np.vstack([interpolated_traj, dwell])
            n_pts = interpolated_traj.shape[0]
            deltas = np.linalg.norm(np.diff(interpolated_traj, axis=0), axis=1)
            log.info("[%s] Trajectory: %d pts  delta min/mean/max = %.4f/%.4f/%.4f rad",
                     name, n_pts, deltas.min(), deltas.mean(), deltas.max())

            rate = rospy.Rate(self.STREAM_RATE_HZ)
            # Use higher k_gains on wrist joints (5-7) to reduce steady-state
            # tracking error during dynamic streaming. Default gains (250/150/50)
            # are too soft for the wrist under gravity load.
            # self.fa.goto_joints(interpolated_traj[1], duration=5, dynamic=True, buffer_time=20,
            #                     k_gains=[600, 600, 600, 600, 600, 300, 200],
            #                     d_gains=[50,  50,  50,  50,  30,  25,  15])
            self.fa.goto_joints(interpolated_traj[1], duration=5, dynamic=True, buffer_time=20)
            init_time = rospy.Time.now().to_time()
            log.info("[%s] Streaming started", name)

            dt_expected = 1.0 / self.STREAM_RATE_HZ
            late_count = 0
            # Cache publish as a local to avoid per-iteration attribute lookup
            publish = self.pub.publish
            # Each entry: (timestamp, actual_joints)
            actual_buffer = []
            for i in range(2, n_pts):
                t_before = rospy.Time.now().to_time()
                traj_gen_proto_msg = JointPositionSensorMessage(
                    id=i,
                    timestamp=t_before - init_time,
                    joints=interpolated_traj[i]
                )
                publish(make_sensor_group_msg(
                    trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                        traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
                ))
                if self.log_fk:
                    actual_buffer.append((t_before - init_time, i, self.fa.get_joints()))
                rate.sleep()
                if rospy.Time.now().to_time() - t_before > dt_expected * 1.5:
                    late_count += 1

            if late_count:
                log.warning("[%s] %d/%d publishes exceeded 1.5x deadline", name, late_count, n_pts - 2)

            term_proto_msg = ShouldTerminateSensorMessage(
                timestamp=rospy.Time.now().to_time() - init_time,
                should_terminate=True
            )
            ros_msg = make_sensor_group_msg(
                termination_handler_sensor_msg=sensor_proto2ros_msg(
                    term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
            self.pub.publish(ros_msg)
            log.info("[%s] Termination message sent", name)

            if actual_buffer:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                log_dir = os.path.join(script_dir, "logs")
                csv_path = os.path.join(log_dir,
                    f"traj_{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
                pos_errs = []
                tilt_degs = []
                cc = self.collision_checker
                with open(csv_path, 'w') as f:
                    f.write("t,"
                            "des_px,des_py,des_pz,"
                            "des_r00,des_r01,des_r02,des_r10,des_r11,des_r12,des_r20,des_r21,des_r22,"
                            "act_px,act_py,act_pz,"
                            "act_r00,act_r01,act_r02,act_r10,act_r11,act_r12,act_r20,act_r21,act_r22,"
                            "pos_err_m,tilt_deg\n")
                    for t_stamp, idx, q_actual in actual_buffer:
                        T_des, _ = cc.ForwardKin(interpolated_traj[idx])
                        # T_d = self._fk_mat(interpolated_traj[idx])
                        des_pos = T_d[0:3, 3]
                        des_rot = T_d[0:3, 0:3]

                        T_act, _ = cc.ForwardKin(q_actual)
                        # T_a = self._fk_mat(q_actual)
                        act_pos = T_a[0:3, 3]
                        act_rot = T_a[0:3, 0:3]

                        pos_err = np.linalg.norm(act_pos - des_pos)
                        # Angle between desired and actual end-effector z-axes
                        cos_tilt = np.clip(np.dot(des_rot[:, 2], act_rot[:, 2]), -1.0, 1.0)
                        tilt_deg = np.degrees(np.arccos(cos_tilt))
                        pos_errs.append(pos_err)
                        tilt_degs.append(tilt_deg)

                        dr = des_rot.flatten()
                        ar = act_rot.flatten()
                        f.write(
                            f"{t_stamp:.4f},"
                            f"{des_pos[0]:.4f},{des_pos[1]:.4f},{des_pos[2]:.4f},"
                            f"{dr[0]:.4f},{dr[1]:.4f},{dr[2]:.4f},"
                            f"{dr[3]:.4f},{dr[4]:.4f},{dr[5]:.4f},"
                            f"{dr[6]:.4f},{dr[7]:.4f},{dr[8]:.4f},"
                            f"{act_pos[0]:.4f},{act_pos[1]:.4f},{act_pos[2]:.4f},"
                            f"{ar[0]:.4f},{ar[1]:.4f},{ar[2]:.4f},"
                            f"{ar[3]:.4f},{ar[4]:.4f},{ar[5]:.4f},"
                            f"{ar[6]:.4f},{ar[7]:.4f},{ar[8]:.4f},"
                            f"{pos_err:.4f},{tilt_deg:.2f}\n"
                        )
                log.info("[%s] FK trajectory saved to %s", name, csv_path)
                log.info(
                    "[%s] Tracking error — pos (m): max=%.4f mean=%.4f  |  tilt (deg): max=%.2f mean=%.2f",
                    name,
                    max(pos_errs), sum(pos_errs) / len(pos_errs),
                    max(tilt_degs), sum(tilt_degs) / len(tilt_degs),
                )

        except Exception:
            log.critical("[%s] _execute_plan crashed", name, exc_info=True)
            raise

    def _execute_plan_waypoints(self, joints_traj, speed=0.4):
        """Execute a joint trajectory by calling goto_joints on each waypoint sequentially.

        Uses non-dynamic (blocking) goto_joints with wait_for_skill, so each
        waypoint fully completes before the next begins.  Per-segment duration
        is derived from that segment's joint-space arc length divided by speed.

        joints_traj shape: (N x 7).
        speed           : joint-space speed (rad/s) used to compute each segment's duration.
        """
        name = f"arm{self.arm_number}"
        try:
            n_pts = joints_traj.shape[0]
            seg_lens = np.linalg.norm(np.diff(joints_traj, axis=0), axis=1)  # (N-1,)

            log.info("[%s] Waypoint execution: %d segments at %.3f rad/s", name, n_pts - 1, speed)
            for i in range(1, n_pts):
                q = joints_traj[i]
                dur = float(seg_lens[i - 1]) / speed
                T = self._fk_mat(q)
                ee_pos = np.round(T[:3, 3], 4)
                ee_rot = np.round(T[:3, :3], 4)
                log.info("[%s] Moving to waypoint %d/%d (len=%.4f rad, dur=%.2fs): %s",
                         name, i, n_pts - 1, seg_lens[i - 1], dur, np.round(q, 4))
                log.info("[%s]   FK EE pos: %s  rot:\n%s", name, ee_pos, ee_rot)
                self.fa.goto_joints(q, duration=dur, dynamic=False)
                log.info("[%s] Waypoint %d/%d reached", name, i, n_pts - 1)
            log.info("[%s] Waypoint execution complete", name)
        except Exception:
            log.critical("[%s] _execute_plan_waypoints crashed", name, exc_info=True)
            raise
        

    def _execute_plan_waypoints_with_velocity(self, joints_traj, joint_velocities=None, speed=0.3):
        """Execute a joint trajectory by calling goto_joints_with_velocity on each waypoint sequentially.

        Uses a cubic Hermite spline for each segment, interpolating from the
        current joint state to the goal position and velocity.  Per-segment
        duration is derived from that segment's joint-space arc length divided
        by speed.

        joints_traj     : (N x 7) array of joint-angle waypoints.
        joint_velocities: (N x 7) array of goal joint velocities (rad/s) at
                          each waypoint, or None to use zero velocity at every
                          waypoint.
        speed           : joint-space speed (rad/s) used to compute each
                          segment's duration.
        """
        name = f"arm{self.arm_number}"
        n_pts = joints_traj.shape[0]

        if joint_velocities is None:
            joint_velocities = np.zeros_like(joints_traj)

        try:
            seg_lens = np.linalg.norm(np.diff(joints_traj, axis=0), axis=1)  # (N-1,)

            log.info("[%s] Waypoint-with-velocity execution: %d segments at %.3f rad/s", name, n_pts - 1, speed)
            for i in range(1, n_pts):
                q = joints_traj[i]
                qd = joint_velocities[i]
                dur = float(seg_lens[i - 1]) / speed
                T = self._fk_mat(q)
                ee_pos = np.round(T[:3, 3], 4)
                ee_rot = np.round(T[:3, :3], 4)
                log.info("[%s] Moving to waypoint %d/%d (len=%.4f rad, dur=%.2fs): %s",
                         name, i, n_pts - 1, seg_lens[i - 1], dur, np.round(q, 4))
                log.info("[%s]   FK EE pos: %s  rot:\n%s", name, ee_pos, ee_rot)
                self.fa.goto_joints_with_velocity(q, qd, duration=dur)
                log.info("[%s] Waypoint %d/%d reached", name, i, n_pts - 1)
            log.info("[%s] Waypoint-with-velocity execution complete", name)
        except Exception:
            log.critical("[%s] _execute_plan_waypoints_with_velocity crashed", name, exc_info=True)
            raise

    def _execute_plan_spline(self, joints_traj):
        """Stream a multi-waypoint trajectory using CubicHermiteSplineJointTrajectoryGenerator.

        Per-segment durations are derived from joint-space arc length divided by
        self.spline_speed (rad/s).  Falls back to self.duration / (N-1) per segment
        if spline_speed is None.

        joints_traj shape: (N x 7).  Safe to call from any thread.
        """
        # Franka Panda per-joint velocity limits (rad/s) — from hardware documentation
        V_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610])
        # Minimum segment duration: shorter than this risks velocity discontinuities
        # and ROS scheduling jitter overwhelming the segment
        MIN_SEG_DUR = 0.5  # seconds

        name = f"arm{self.arm_number}"

        # ── Validate array shape ──────────────────────────────────────────────
        joints_traj = np.asarray(joints_traj, dtype=float)
        if joints_traj.ndim != 2 or joints_traj.shape[1] != 7:
            raise ValueError(
                f"[{name}] joints_traj must be shape (N, 7), got {joints_traj.shape}"
            )
        N = joints_traj.shape[0]
        if N < 2:
            raise ValueError(f"[{name}] Need at least 2 waypoints, got {N}")

        # # ── Check first waypoint proximity to current robot state ─────────────
        # q_current = np.array(self.fa.get_joints())
        # init_err  = float(np.max(np.abs(joints_traj[0] - q_current)))
        # if init_err > 0.1:
        #     # Not a hard stop — the C++ generator snaps initial state from the live
        #     # robot, so the first segment will start from wherever the robot actually
        #     # is.  Large offsets mean the spline deviates from the planned path.
        #     log.warning(
        #         "[%s] First waypoint is %.4f rad from current joints (max over joints). "
        #         "The spline will begin from the robot's actual position. "
        #         "Consider using fa.get_joints() as waypoints[0].", name, init_err
        #     )

        # ── Compute per-segment arc lengths ───────────────────────────────────
        seg_arcs  = np.linalg.norm(np.diff(joints_traj, axis=0), axis=1)  # (N-1,)
        total_arc = float(seg_arcs.sum())

        # if total_arc < 1e-6:
        #     log.warning("[%s] Total path arc length is near zero — robot already at goal?", name)
        #     return

        # ── Derive per-segment durations from spline_speed ───────────────────
        if self.spline_speed is not None:
            durations = (seg_arcs / self.spline_speed).tolist()
            log.info(
                "[%s] spline_speed=%.4f rad/s  total arc=%.4f rad  est. time=%.2fs",
                name, self.spline_speed, total_arc, sum(durations)
            )
        else:
            per_seg   = self.duration / (N - 1)
            durations = [per_seg] * (N - 1)
            log.warning(
                "[%s] spline_speed not set — uniform %.2fs per segment "
                "(pass --spline-speed for arc-proportional timing)", name, per_seg
            )

        # ── Enforce minimum segment duration ──────────────────────────────────
        for i in range(N - 1):
            if durations[i] < MIN_SEG_DUR:
                log.warning(
                    "[%s] Segment %d duration %.3fs below minimum %.1fs "
                    "(arc=%.4f rad) — clamping",
                    name, i, durations[i], MIN_SEG_DUR, seg_arcs[i]
                )
                durations[i] = MIN_SEG_DUR

        # ── Per-segment velocity sanity check against Franka limits ───────────
        # Average joint velocity over a segment is a lower bound on peak velocity
        # (the Catmull-Rom hermite can overshoot by ~1.5×). Warn at 50%, hard-stop
        # at 80% of the hardware limit to leave headroom for the actual peak.
        for i in range(N - 1):
            dq      = np.abs(joints_traj[i + 1] - joints_traj[i])
            avg_vel = dq / durations[i]
            ratios  = avg_vel / V_MAX
            worst   = int(np.argmax(ratios))
            ratio   = float(ratios[worst])
            if ratio > 0.8:
                raise ValueError(
                    f"[{name}] Segment {i}: joint {worst + 1} average velocity "
                    f"{avg_vel[worst]:.3f} rad/s exceeds 80% of the hardware limit "
                    f"({V_MAX[worst]:.3f} rad/s). Reduce --spline-speed or increase "
                    f"segment duration."
                )
            if ratio > 0.5:
                log.warning(
                    "[%s] Segment %d: joint %d at %.0f%% of velocity limit "
                    "(%.3f / %.3f rad/s) — hermite peak may exceed limit",
                    name, i, worst + 1, ratio * 100, avg_vel[worst], V_MAX[worst]
                )

        # ── Log plan summary ──────────────────────────────────────────────────
        log.info(
            "[%s] Spline plan ready: %d waypoints, %d segments, "
            "total arc=%.4f rad, total time=%.2fs",
            name, N, N - 1, total_arc, sum(durations)
        )
        for i in range(N - 1):
            log.info(
                "[%s]   seg %2d: arc=%.4f rad  dur=%.2fs  "
                "from=%s  to=%s",
                name, i, seg_arcs[i], durations[i],
                np.round(joints_traj[i], 3), np.round(joints_traj[i + 1], 3)
            )

        # ── Execute ───────────────────────────────────────────────────────────
        try:
            waypoints = [joints_traj[i] for i in range(N)]
            self.fa.goto_joints_spline(waypoints, durations)
            log.info("[%s] _execute_plan_spline complete", name)
        except Exception:
            log.critical("[%s] _execute_plan_spline crashed", name, exc_info=True)
            raise

    # ── Public execution methods ──────────────────────────────────────────────

    def execute_real_only_joints(self, q_init, q_goal, label_zone=None):
        """Plan via IK (goal only) + PRM and execute on the real robot.

        Blocking — safe to run directly or as a threading.Thread target.

        Args:
            q_init        : 7-element joint array — current robot state
            q_goal        : 7-element joint array
            label_zone    : integer label zone (1, 2, …) or None for free-space PRM
            goal_yaw      : end-effector yaw at the goal (radians, default 0)
        """
        plan = self._plan_from_joints(q_init, q_goal, label_zone)
        if plan is None:
            log.error("Arm%d: planning failed — aborting execution", self.arm_number)
            return
        q_goal = np.array(plan[-1])
        # self._execute_plan(np.array(plan))
        self._execute_plan_waypoints(np.array(plan))


    def execute_real_only(self, q_init, taskPose_goal, label_zone=None, goal_yaw=0.0):
        """Plan via IK (goal only) + PRM and execute on the real robot.

        Blocking — safe to run directly or as a threading.Thread target.

        Args:
            q_init        : 7-element joint array — current robot state
            taskPose_goal : [x, y, z] goal Cartesian position in arm base frame (metres)
            label_zone    : integer label zone (1, 2, …) or None for free-space PRM
            goal_yaw      : end-effector yaw at the goal (radians, default 0)
        """
        plan = self._plan_from_poses(q_init, taskPose_goal, label_zone, goal_yaw)
        if plan is None:
            log.error("Arm%d: planning failed — aborting execution", self.arm_number)
            return
        q_goal = np.array(plan[-1])
        # self._execute_plan(np.array(plan))
        self._execute_plan_waypoints(np.array(plan))
        # self.fa.wait_for_skill()  # not needed: goto_joints_with_velocity blocks internally
        q_actual = self.fa.get_joints()
        q_err = q_actual - q_goal
        Tcurr, _ = self.collision_checker.ForwardKin(q_actual)
        # ee_z = self._fk_mat(q_actual)[:3, 2]
        # tilt_deg = np.degrees(np.arccos(np.clip(-ee_z[2], -1.0, 1.0)))
        # log.info("Arm%d: execute_real_only done  q_err=%s (max=%.4f rad)  EE tilt=%.1f deg",
        #          self.arm_number, np.round(q_err, 4), np.max(np.abs(q_err)), tilt_deg)

    def plan(self, q_init, taskPose_goal, label_zone=None, goal_yaw=0.0):
        """Run IK + PRM planning and return the interpolated trajectory (N x 7 array).

        Call this before execute_with_sim / execute_sim_only when you need to drive
        the MuJoCo viewer from the main thread while streaming runs in a background thread.
        Returns None if planning fails.
        """
        waypoints = self._plan_from_poses(q_init, taskPose_goal, label_zone, goal_yaw)
        if waypoints is None:
            return None
        return self._build_interpolated_traj(np.array(waypoints))

    def execute_with_sim(self, q_init, taskPose_goal, label_zone=None, goal_yaw=0.0, viz=None):
        """Plan via IK (goal only) + PRM, execute on real robot, and mirror in MuJoCo.

        NOTE: MuJoCo's viewer is not thread-safe. If running two arms in parallel,
        call plan() on both arms first, then drive the viz loop from the main thread
        using the returned interpolated trajectories. See the __main__ block for the
        correct pattern.

        Blocking — safe to run directly (single arm) or as a threading.Thread target
        when the caller owns the viz loop.
        """
        plan = self._plan_from_poses(q_init, taskPose_goal, label_zone, goal_yaw)
        if plan is None:
            log.error("Arm%d: planning failed — aborting execution", self.arm_number)
            return

        joints_traj = np.array(plan)
        t = threading.Thread(target=self._execute_plan, args=(joints_traj,),
                             name=f"arm{self.arm_number}_stream")
        t.start()

        if viz is not None:
            interp = self._build_interpolated_traj(joints_traj)
            mj_substeps = max(1, round(1.0 / (self.STREAM_RATE_HZ * viz.model.opt.timestep)))
            for q_des in interp:
                viz.step_impedance(q_des, q_des, n_substeps=mj_substeps)

        t.join()
        self.fa.wait_for_skill()
        q_actual = self.fa.get_joints()
        q_goal = np.array(plan[-1])
        q_err = q_actual - q_goal
        Tcurr, _ = self.collision_checker.ForwardKin(q_actual)
        # ee_z = self._fk_mat(q_actual)[:3, 2]
        tilt_deg = np.degrees(np.arccos(np.clip(-ee_z[2], -1.0, 1.0)))
        log.info("Arm%d: execute_with_sim done  q_err=%s (max=%.4f rad)  EE tilt=%.1f deg",
                 self.arm_number, np.round(q_err, 4), np.max(np.abs(q_err)), tilt_deg)

        if viz is not None:
            import time
            while viz.viewer.is_running():
                viz.viewer.sync()
                time.sleep(0.05)

    def execute_sim_only(self, q_init, taskPose_goal, label_zone=None, goal_yaw=0.0, viz=None):
        """Plan via IK (goal only) + PRM and replay in MuJoCo only (no real robot).

        Blocking — safe to run directly or as a threading.Thread target.
        """
        import time

        plan = self._plan_from_poses(q_init, taskPose_goal, label_zone, goal_yaw)
        if plan is None:
            log.error("Arm%d: planning failed — aborting sim replay", self.arm_number)
            return

        interp = self._build_interpolated_traj(np.array(plan))
        log.info("Arm%d: execute_sim_only replaying %d setpoints at %.1f Hz",
                 self.arm_number, len(interp), self.STREAM_RATE_HZ)

        if viz is not None:
            mj_substeps = max(1, round(1.0 / (self.STREAM_RATE_HZ * viz.model.opt.timestep)))
            dt_control  = 1.0 / self.STREAM_RATE_HZ
            for q_des in interp:
                t_start = time.time()
                viz.step_impedance(q_des, q_des, n_substeps=mj_substeps)
                elapsed = time.time() - t_start
                time.sleep(max(0.0, dt_control - elapsed))

            log.info("Arm%d: sim replay done", self.arm_number)
            while viz.viewer.is_running():
                viz.viewer.sync()
                time.sleep(0.05)
        else:
            log.warning("Arm%d: execute_sim_only called without a visualizer", self.arm_number)

    def execute_plan(self, joints_traj):
        """Stream a raw (N x 7) waypoint trajectory directly to the real robot.

        Applies a quadratic ramp-up / linear mid / ramp-down interpolation
        then streams at 50 Hz via dynamic joint-position control.

        joints_traj : np.ndarray, shape (N, 7)
        """
        num_interp_slow = 500
        num_interp = 200
        interpolated_traj = []
        t_linear = np.linspace(1 / num_interp, 1, num_interp)
        t_slow = np.linspace(1 / num_interp_slow, 1, num_interp_slow)
        t_ramp_up = t_slow ** 2
        t_ramp_down = 1 - (1 - t_slow) ** 2

        interpolated_traj.append(joints_traj[0, :])
        for t_i in range(len(t_ramp_up)):
            dt = t_ramp_up[t_i]
            interp_traj_i = joints_traj[1, :] * dt + joints_traj[0, :] * (1 - dt)
            interpolated_traj.append(interp_traj_i)

        for i in range(2, joints_traj.shape[0] - 1):
            for t_i in range(len(t_linear)):
                dt = t_linear[t_i]
                interp_traj_i = joints_traj[i, :] * dt + joints_traj[i - 1, :] * (1 - dt)
                interpolated_traj.append(interp_traj_i)

        for t_i in range(len(t_ramp_down)):
            dt = t_ramp_down[t_i]
            interp_traj_i = joints_traj[-1, :] * dt + joints_traj[-2, :] * (1 - dt)
            interpolated_traj.append(interp_traj_i)

        interpolated_traj = np.array(interpolated_traj)
        print('Executing joints trajectory of shape: ', interpolated_traj.shape)

        rate = rospy.Rate(50)
        self.fa.goto_joints(interpolated_traj[1], duration=5, dynamic=True, buffer_time=20)
        init_time = rospy.Time.now().to_time()
        for i in range(2, interpolated_traj.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time,
                joints=interpolated_traj[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            self.pub.publish(ros_msg)
            rate.sleep()

        term_proto_msg = ShouldTerminateSensorMessage(
            timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
        self.pub.publish(ros_msg)


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    from franka_prm_dual_arm import MuJoCoVisualizer

    parser = argparse.ArgumentParser(
        description="Test SingleArmExecutor: PRM-based motion on two Franka arms."
    )
    parser.add_argument("--sim", action="store_true",
                        help="Run MuJoCo visualization alongside the real robots")
    parser.add_argument("--sim-only", action="store_true",
                        help="Run MuJoCo visualization only (no real robots)")
    parser.add_argument("--traj", choices=["ramp", "minjerk", "spline", "totg"], default="ramp",
                        help="Trajectory generator: 'ramp' (default), 'minjerk', 'spline', or 'totg'")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Motion duration in seconds for minjerk/spline (default: 5.0)")
    parser.add_argument("--spline-speed", type=float, default=0.5,
                        help="Spline average speed in rad/s of arc length; overrides --duration for spline mode")
    parser.add_argument("--totg-scale", type=float, default=0.25,
                        help="Scale factor for TOTG joint vel/acc limits, e.g. 0.5 = half speed (default: 0.25)")
    parser.add_argument("--debug", action="store_true",
                        help="Log FK poses to CSV during execution")
    args = parser.parse_args()

    random.seed(1)

    # ── Goal task positions (x, y, z metres + yaw radians, arm base frame) ────
    set_arm1_starting_position = False
    arm1_start_pos_list = [[0.3, 0, 0.4], 
                           [0.3, -0.3, 0.2], 
                           [0.75, 0.2, 0.3], 
                           [0.4, 0.3, 0.2]]
    arm1_start_pos = arm1_start_pos_list[0]
    arm1_start_yaw = 0.0

    task_goal_1_list  = [[0.3, -0.5, 0.1],      # Input
                         [0.3,  0.75, 0.3],     # Output
                         [0.75, -0.2, 0.03],     # L1
                         [0.75,  0.2, 0.2]]     # L2
    task_goal_1 = task_goal_1_list[2]
    task_goal_1 = np.array([0.4, 0.0, 0.5])
    # task_goal_1 = np.array([0.2, 0.0, 0.0])

    goal_yaw_1   = 0.0
    arm1_label_zone = 3

    set_arm2_starting_position = False
    arm2_start_pos_list = [[0.20, 0.0, 0.4], 
                           [0.50, 0.1, 0.4], 
                           [0.20, 0.2, 0.4],]
    arm2_start_pos = arm2_start_pos_list[0]
    arm2_start_yaw = 0.0

    # task_goal_2  = np.array([0.18, 0.001, 0.41])
    task_goal_2_list  = [[0.18, 0.001, 0.41],
                         [0.3656, 0.1851, 0.2184],
                         [0.415, 0.169, 0.2267],
                         [0.3889, 0.1488, 0.0176],
                         [0.5314, 0.2731, 0.0251],
                         [0.5201, 0.1557, 0.0198]]
    task_goal_2 = task_goal_2_list[0]

    q_goal_2 = [1.5633, -1.6532,  0.2592 ,-0.4644,  2.893 ,  1.4371, -0.5655]

    goal_yaw_2   = 0.0
    arm2_label_zone = 1
    # ──────────────────────────────────────────────────────────────────────────

    # Instantiate one executor per arm.
    # Arm 1 initialises the ROS node; arm 2 reuses it.
    _sim_only = args.sim_only
    arm1 = SingleArmExecutor(arm_number=1, traj_type=args.traj,
                             duration=args.duration, log_fk=args.debug,
                             init_node=True, sim_only=_sim_only,
                             totg_scale=args.totg_scale,
                             spline_speed=args.spline_speed)
    arm2 = SingleArmExecutor(arm_number=2, traj_type=args.traj,
                             duration=args.duration, log_fk=args.debug,
                             init_node=False, sim_only=_sim_only,
                             totg_scale=args.totg_scale,
                             spline_speed=args.spline_speed)

    if args.debug:
        _fk2 = arm2._fk_mat(q_goal_2)
        from scipy.spatial.transform import Rotation as _R
        _rpy2 = _R.from_matrix(_fk2[:3, :3]).as_euler('xyz', degrees=True)
        log.debug("FK q_goal_2 -> pos=[%.4f, %.4f, %.4f]  RPY=[%.2f, %.2f, %.2f] deg",
                  _fk2[0, 3], _fk2[1, 3], _fk2[2, 3], _rpy2[0], _rpy2[1], _rpy2[2])

    # Read current joint positions — passed directly as q_init (no IK needed for start).
    # In sim-only mode there is no real robot, so use a known safe default.
    _default_q = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])
    q_start_1 = arm1.fa.get_joints() if arm1.fa is not None else _default_q
    q_start_2 = arm2.fa.get_joints() if arm2.fa is not None else _default_q

    # Override starting positions via IK if custom start poses are set.
    if set_arm1_starting_position:
        q_start_1 = arm1._compute_ik(arm1_start_pos, yaw=arm1_start_yaw)
        log.info("Arm1 using custom start pos=%s yaw=%.3f -> q=%s",
                 arm1_start_pos, arm1_start_yaw, np.round(q_start_1, 3))
    if set_arm2_starting_position:
        q_start_2 = arm2._compute_ik(arm2_start_pos, yaw=arm2_start_yaw)
        log.info("Arm2 using custom start pos=%s yaw=%.3f -> q=%s",
                 arm2_start_pos, arm2_start_yaw, np.round(q_start_2, 3))
    log.info("Arm1 q_start=%s  goal=%s  yaw=%.3f", np.round(q_start_1, 3), task_goal_1, goal_yaw_1)
    log.info("Arm2 q_start=%s  goal=%s  yaw=%.3f", np.round(q_start_2, 3), task_goal_2, goal_yaw_2)

    if args.sim_only or args.sim:
        # Plan both arms on the main thread before touching MuJoCo.
        # MuJoCo's passive viewer (OpenGL) must be driven exclusively from the main thread —
        # calling viz.step_impedance from worker threads causes a segfault.
        interp1 = arm1.plan(q_start_1, task_goal_1, arm1_label_zone, goal_yaw_1)
        interp2 = arm2.plan(q_start_2, task_goal_2, arm2_label_zone, goal_yaw_2)
        if interp1 is None or interp2 is None:
            log.error("Planning failed for one or both arms — aborting")
            raise SystemExit(1)

        viz = MuJoCoVisualizer([q_start_1], [q_start_2])
        mj_substeps = max(1, round(1.0 / (arm1.STREAM_RATE_HZ * viz.model.opt.timestep)))
        n = max(len(interp1), len(interp2))

        if args.sim:
            # Launch real robot streams in background threads.
            t1 = threading.Thread(target=arm1._execute_plan, args=(interp1,), name="arm1_stream")
            t2 = threading.Thread(target=arm2._execute_plan, args=(interp2,), name="arm2_stream")
            t1.start(); t2.start()

        import time
        dt_control = 1.0 / arm1.STREAM_RATE_HZ
        for i in range(n):
            t_start = time.time()
            q1_des = interp1[min(i, len(interp1) - 1)]
            q2_des = interp2[min(i, len(interp2) - 1)]
            viz.step_impedance(q1_des, q2_des, n_substeps=mj_substeps)
            if args.sim_only:
                elapsed = time.time() - t_start
                time.sleep(max(0.0, dt_control - elapsed))

        if args.sim:
            t1.join(); t2.join()
            arm1.fa.wait_for_skill()
            arm2.fa.wait_for_skill()

        while viz.viewer.is_running():
            viz.viewer.sync()
            time.sleep(0.05)

    else:
        t1 = threading.Thread(target=arm1.execute_real_only,
                              args=(q_start_1, task_goal_1, arm1_label_zone, goal_yaw_1))
        # t2 = threading.Thread(target=arm2.execute_real_only,
        #                       args=(q_start_2, task_goal_2, arm2_label_zone, goal_yaw_2))
        t2 = threading.Thread(target=arm2.execute_real_only_joints,
                              args=(q_start_2, q_goal_2, arm2_label_zone))
        t1.start(); t2.start()
        t1.join();  t2.join()

    log.info("Both arms done.")
