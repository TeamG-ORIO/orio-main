#!/usr/bin/env python3
"""
PRM-based point-to-point motion on the real Franka robot.
Uses the pre-built PRM (prm_files/myPRM_arm1.p / myPRM_arm2.p) and frankapy for execution.

Usage:
    python franka_prm_dual_arm.py                       # real robot only
    python franka_prm_dual_arm.py --sim                 # real robot + MuJoCo visualization
    python franka_prm_dual_arm.py --sim-only            # MuJoCo only (no real robot)
"""

import argparse
import logging
import os
import numpy as np
import pickle
import heapq
import random
import sys
import threading
from datetime import datetime

import ikpy.chain
import RobotUtil as rt
import SimpleFranka  # offline kinematics/collision only

from frankapy import FrankaArm, SensorDataMessageType
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
import rospy

# ── Logging ───────────────────────────────────────────────────────────────────

def _setup_logger(log_to_file=True):
    fmt       = "%(asctime)s [%(levelname)-8s] %(name)s: %(message)s"
    datefmt   = "%Y-%m-%dT%H:%M:%S"
    formatter = logging.Formatter(fmt, datefmt=datefmt)

    logger = logging.getLogger("franka_prm")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False   # don't pass records up to root (rospy/ROS handlers)
    logger.handlers.clear()    # safe: only clears our own logger's handlers

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

_setup_logger()
log = logging.getLogger("franka_prm")
np.set_printoptions(suppress=True, precision=4)

# ── Constants ─────────────────────────────────────────────────────────────────

PRM_FILE_ARM1 = "prm_files_old/myPRM_arm1_free.p"
PRM_FILE_ARM2 = "prm_files_old/myPRM_arm2_free.p"
MODEL_XML     = "mujoco_files/orio_dual_scene.xml"
URDF_FILE     = "../orio/panda_arm_hand.urdf"

LEFT_JOINT_NAMES  = [f"mj_left_joint{i}"  for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"mj_right_joint{i}" for i in range(1, 8)]

# Joint impedance gains — identical to FrankaPy's real hardware controller
# (FrankaConstants.DEFAULT_K_GAINS / DEFAULT_D_GAINS)
KP = np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0,  50.0])
KD = np.array([ 50.0,  50.0,  50.0,  50.0,  30.0,  25.0,  15.0])


# ── IK helper ─────────────────────────────────────────────────────────────────

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
    result = angles[1:8]
    # Verify IK quality by FK back-check
    fk_pos = ik_chain.forward_kinematics(angles)[:3, 3]
    err = np.linalg.norm(fk_pos - np.array(task_pos))
    if err > 0.01:
        log.warning("IK position error %.4f m  (target=%s  fk=%s)", err,
                    np.round(task_pos, 4), np.round(fk_pos, 4))
    else:
        log.info("IK solved  joints=%s  pos_err=%.4f m", np.round(result, 3), err)
    return result


# ── PRM Query ─────────────────────────────────────────────────────────────────

def prm_query(q_init, q_goal, collision_checker, prm_file):
    with open(prm_file, 'rb') as f:
        prm_vertices = pickle.load(f)
        prm_edges    = pickle.load(f)
        obs_points   = pickle.load(f)
        obs_axes     = pickle.load(f)

    num_nodes, num_edges, num_components = rt.AnalyzeGraph(prm_vertices, prm_edges)
    log.info("PRM loaded from '%s': %d nodes, %d edges, %d components",
             prm_file, num_nodes, num_edges, num_components)

    def find_neighbors(q):
        sorted_indices = sorted(range(len(prm_vertices)),
                                key=lambda i: np.linalg.norm(np.array(prm_vertices[i]) - np.array(q)))
        neighbors = []
        for i in sorted_indices:
            if not collision_checker.DetectCollisionEdge(prm_vertices[i], q, obs_points, obs_axes):
                neighbors.append(i)
            if len(neighbors) >= 10:
                break
        return neighbors

    if collision_checker.DetectCollision(q_init, obs_points, obs_axes):
        log.error("Start configuration is in collision: %s", np.round(q_init, 3))
        return None
    if collision_checker.DetectCollision(q_goal, obs_points, obs_axes):
        log.error("Goal configuration is in collision: %s", np.round(q_goal, 3))
        return None

    init_neighbors = find_neighbors(q_init)
    goal_neighbors = find_neighbors(q_goal)
    log.info("Neighbors found — init: %d, goal: %d", len(init_neighbors), len(goal_neighbors))
    if not init_neighbors or not goal_neighbors:
        log.error("Could not connect start/goal to PRM (init=%d, goal=%d)",
                  len(init_neighbors), len(goal_neighbors))
        return None

    heuristic = [np.linalg.norm(np.array(v) - np.array(q_goal)) for v in prm_vertices]
    g_cost    = [float('inf')] * len(prm_vertices)
    parent    = [None] * len(prm_vertices)

    open_set = []
    for n in init_neighbors:
        g = np.linalg.norm(np.array(prm_vertices[n]) - np.array(q_init))
        g_cost[n] = g
        heapq.heappush(open_set, (g + heuristic[n], n))

    closed_set = set()
    goal_node  = None
    while open_set:
        _, curr = heapq.heappop(open_set)
        if curr in closed_set:
            continue
        closed_set.add(curr)
        if curr in goal_neighbors:
            goal_node = curr
            break
        for nb in prm_edges[curr]:
            if nb in closed_set:
                continue
            edge_cost = np.linalg.norm(np.array(prm_vertices[nb]) - np.array(prm_vertices[curr]))
            tg = g_cost[curr] + edge_cost
            if tg < g_cost[nb]:
                g_cost[nb] = tg
                parent[nb]  = curr
                heapq.heappush(open_set, (tg + heuristic[nb], nb))

    if goal_node is None:
        log.error("A* failed to find a path (expanded %d nodes)", len(closed_set))
        return None

    path = [goal_node]
    while parent[path[0]] is not None:
        path.insert(0, parent[path[0]])

    plan = ([np.array(q_init)]
            + [np.array(prm_vertices[i]) for i in path]
            + [np.array(q_goal)])

    for _ in range(200):
        if len(plan) <= 2:
            break
        i = random.randint(0, len(plan) - 3)
        j = random.randint(i + 2, len(plan) - 1)
        if not collision_checker.DetectCollisionEdge(plan[i], plan[j], obs_points, obs_axes):
            plan = plan[:i+1] + plan[j:]

    log.info("Plan ready: %d waypoints after shortcutting", len(plan))
    return plan


# ── MuJoCo Visualizer ─────────────────────────────────────────────────────────

class MuJoCoVisualizer:
    """Owns the MuJoCo model/data/viewer and joint index arrays for dual-arm simulation."""

    def __init__(self, plan1, plan2, model_xml=MODEL_XML):
        import mujoco as mj
        from mujoco import viewer as mj_viewer

        self._mj = mj
        self.model = mj.MjModel.from_xml_path(model_xml)
        self.data  = mj.MjData(self.model)

        # Resolve qpos/qvel/ctrl indices by joint name for each arm
        self._l_qpos = [self.model.joint(name).qposadr[0] for name in LEFT_JOINT_NAMES]
        self._l_qvel = [self.model.joint(name).dofadr[0]  for name in LEFT_JOINT_NAMES]
        self._l_ctrl = [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"mj_left_act_trq{i}")
                        for i in range(1, 8)]

        self._r_qpos = [self.model.joint(name).qposadr[0] for name in RIGHT_JOINT_NAMES]
        self._r_qvel = [self.model.joint(name).dofadr[0]  for name in RIGHT_JOINT_NAMES]
        self._r_ctrl = [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"mj_right_act_trq{i}")
                        for i in range(1, 8)]

        self.data.qpos[self._l_qpos] = plan1[0].copy()
        self.data.qpos[self._r_qpos] = plan2[0].copy()
        self.data.qvel[:]            = 0
        mj.mj_forward(self.model, self.data)

        self.viewer = mj_viewer.launch_passive(self.model, self.data)
        self.viewer.cam.distance  = 2.5
        self.viewer.cam.azimuth   = 135
        self.viewer.cam.elevation = -25
        self.viewer.cam.lookat[:] = [0.3, 0.0, 0.3]

    def step_impedance(self, q1_des, q2_des, n_substeps=1):
        """Apply one impedance control step to both arms given desired joint positions.

        The real hardware receives desired positions from the Python setpoint stream
        and runs this same PD + gravity-comp law at 1 kHz inside franka-interface.
        Here we run it once per MuJoCo timestep (called in a loop from execute_sim_only).
        Velocity setpoint is zero — the impedance stiffness drives tracking; damping
        damps out velocity exactly as the hardware does when receiving static setpoints.
        """
        mj = self._mj
        for _ in range(n_substeps):
            q1  = self.data.qpos[self._l_qpos].copy()
            qd1 = self.data.qvel[self._l_qvel].copy()
            self.data.ctrl[self._l_ctrl] = (KP * (q1_des - q1)
                                            - KD * qd1
                                            + self.data.qfrc_bias[self._l_qvel])
            q2  = self.data.qpos[self._r_qpos].copy()
            qd2 = self.data.qvel[self._r_qvel].copy()
            self.data.ctrl[self._r_ctrl] = (KP * (q2_des - q2)
                                            - KD * qd2
                                            + self.data.qfrc_bias[self._r_qvel])
            mj.mj_step(self.model, self.data)
        self.viewer.sync()

    def close(self):
        self.viewer.close()


# ── Dual-Arm Executor ─────────────────────────────────────────────────────────

class DualArmExecutor:
    """Owns the ROS publisher and FrankaArm handles; exposes execution modes."""

    # Trajectory interpolation parameters (must match _execute_plan internals)
    N_RAMP         = 50    # points for ramp-up and ramp-down phases
    N_MID          = 20    # points per mid-trajectory segment
    STREAM_RATE_HZ = 50.0  # ROS publish rate (Hz)

    def __init__(self, fa1=None, fa2=None, traj_type='ramp', minjerk_duration=5.0,
                 collision_checker=None):
        """
        traj_type        : 'ramp'    – quadratic ramp-up/down with linear mid segments (original)
                           'minjerk' – single 5th-order min-jerk polynomial, start→goal
        minjerk_duration : total motion time in seconds (only used when traj_type='minjerk')
        collision_checker: if provided, FK poses are buffered and dumped to CSV after execution
        """
        assert traj_type in ('ramp', 'minjerk'), f"Unknown traj_type '{traj_type}'"
        self.fa1 = fa1
        self.fa2 = fa2
        self.traj_type = traj_type
        self.minjerk_duration = minjerk_duration
        self.collision_checker = collision_checker
        # Each arm listens on its own sensor topic; publish to them separately.
        self.pub1 = rospy.Publisher('/franka_ros_interface/sensor',
                                    SensorDataGroup, queue_size=1000)
        self.pub2 = rospy.Publisher('/franka_ros_interface_2/sensor',
                                    SensorDataGroup, queue_size=1000)

    def execute_real_only(self, plan1, plan2):
        """Execute full plans on both real robots in parallel."""
        joints_traj1 = np.array(plan1)
        joints_traj2 = np.array(plan2)
        log.info("execute_real_only: launching arm1 (%d wpts) and arm2 (%d wpts) threads",
                 len(plan1), len(plan2))
        t1 = threading.Thread(target=self._execute_plan, args=(joints_traj1, self.fa1, self.pub1),
                              name="arm1")
        t2 = threading.Thread(target=self._execute_plan, args=(joints_traj2, self.fa2, self.pub2),
                              name="arm2")
        t1.start(); t2.start()
        t1.join();  t2.join()
        self.fa1.wait_for_skill()
        self.fa2.wait_for_skill()
        log.info("execute_real_only: both arms done")

    def execute_with_sim(self, plan1, plan2, viz):
        """Execute both plans on real robots with MuJoCo visualization."""
        joints_traj1 = np.array(plan1)
        joints_traj2 = np.array(plan2)
        t1 = threading.Thread(target=self._execute_plan, args=(joints_traj1, self.fa1, self.pub1),
                              name="arm1")
        t2 = threading.Thread(target=self._execute_plan, args=(joints_traj2, self.fa2, self.pub2),
                              name="arm2")
        t1.start(); t2.start()

        # Mirror the same setpoint stream in MuJoCo so the viz matches the real arms.
        interp1 = self._build_interpolated_traj(joints_traj1)
        interp2 = self._build_interpolated_traj(joints_traj2)
        n = max(len(interp1), len(interp2))
        # Substeps: run MuJoCo physics at its native timestep between each 50Hz setpoint.
        mj_substeps = max(1, round(1.0 / (self.STREAM_RATE_HZ * viz.model.opt.timestep)))
        for i in range(n):
            q1_des = interp1[min(i, len(interp1) - 1)]
            q2_des = interp2[min(i, len(interp2) - 1)]
            viz.step_impedance(q1_des, q2_des, n_substeps=mj_substeps)

        t1.join(); t2.join()
        self.fa1.wait_for_skill()
        self.fa2.wait_for_skill()
        log.info("execute_with_sim: both arms done")
        while viz.viewer.is_running():
            viz.viewer.sync()
            import time; time.sleep(0.05)

    def execute_sim_only(self, plan1, plan2, viz):
        """Replay the exact same setpoint stream as _execute_plan into MuJoCo.

        Builds the same interpolated trajectory that would be streamed to the real
        hardware (same ramp profile, same N_RAMP/N_MID/STREAM_RATE_HZ), then feeds
        each desired-joint-position setpoint into the MuJoCo impedance controller at
        the equivalent 50Hz rate — so sim behaviour matches real hardware as closely
        as possible without running the actual C++ franka-interface controller.
        """
        import time
        interp1 = self._build_interpolated_traj(np.array(plan1))
        interp2 = self._build_interpolated_traj(np.array(plan2))
        n = max(len(interp1), len(interp2))
        log.info("execute_sim_only: replaying %d setpoints at %.1f Hz", n, self.STREAM_RATE_HZ)

        # How many MuJoCo physics steps to run per 50Hz control tick.
        mj_substeps = max(1, round(1.0 / (self.STREAM_RATE_HZ * viz.model.opt.timestep)))
        dt_control  = 1.0 / self.STREAM_RATE_HZ

        for i in range(n):
            t_start = time.time()
            q1_des = interp1[min(i, len(interp1) - 1)]
            q2_des = interp2[min(i, len(interp2) - 1)]
            viz.step_impedance(q1_des, q2_des, n_substeps=mj_substeps)
            # Pace the loop to real time so the viewer runs at the right speed.
            elapsed = time.time() - t_start
            time.sleep(max(0.0, dt_control - elapsed))

        log.info("execute_sim_only: replay done")
        while viz.viewer.is_running():
            viz.viewer.sync()
            time.sleep(0.05)

    def _est_duration(self, plan):
        """Estimate trajectory duration based on interpolation parameters."""
        n = len(plan)
        pts = self.N_RAMP + max(0, n - 3) * self.N_MID + self.N_RAMP  # ramp_up + middle + ramp_down
        return pts / self.STREAM_RATE_HZ

    def _build_interpolated_traj(self, joints_traj):
        """Dispatch to the selected trajectory generator.

        Single source of truth for trajectory shape — both _execute_plan (real hardware)
        and execute_sim_only call this so they receive identical setpoints.
        joints_traj shape: (N x 7). Returns (M x 7) array.
        """
        if self.traj_type == 'minjerk':
            return self._build_minjerk_traj(joints_traj)
        return self._build_ramp_traj(joints_traj)

    def _build_ramp_traj(self, joints_traj):
        """Quadratic ramp-up → linear mid segments → quadratic ramp-down.

        Uses t² ease-in and (1-(1-t)²) ease-out so velocity starts and ends at
        zero but ramps linearly through the mid section.
        """
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
        """Min-jerk polynomial through each consecutive pair of PRM waypoints.

        Each segment i→i+1 gets its own 5th-order polynomial (zero vel/acc at
        both endpoints). Point count per segment is proportional to joint-space
        distance so speed is roughly consistent across segments. Total duration
        is minjerk_duration seconds spread across all segments by distance weight.

        Handles the common post-shortcutting case where joints_traj has only 2
        rows (start and goal) — produces a single smooth segment.
        """
        N = joints_traj.shape[0]
        total_pts = max(2, round(self.minjerk_duration * self.STREAM_RATE_HZ))

        # Compute arc lengths to allocate points proportionally across segments.
        distances = np.array([
            np.linalg.norm(joints_traj[i+1] - joints_traj[i])
            for i in range(N - 1)
        ])
        total_dist = distances.sum()

        interp = [joints_traj[0, :]]
        for i in range(N - 1):
            q0 = joints_traj[i, :]
            qf = joints_traj[i+1, :]
            # Allocate at least 2 points per segment; remainder by distance weight.
            if total_dist > 0:
                n = max(2, round(total_pts * distances[i] / total_dist))
            else:
                n = max(2, total_pts // (N - 1))
            t = np.linspace(0.0, 1.0, n + 1)[1:]   # exclude start (already appended)
            w = 10*t**3 - 15*t**4 + 6*t**5
            interp.append(q0 + np.outer(w, qf - q0))

        return np.vstack(interp)

    def _execute_plan(self, joints_traj, arm, pub):
        """
        Stream an interpolated joint trajectory to a single real robot arm via ROS.
        joints_traj shape: (N x 7)
        """
        name = threading.current_thread().name
        try:
            interpolated_traj = self._build_interpolated_traj(joints_traj)
            n_pts = interpolated_traj.shape[0]
            deltas = np.linalg.norm(np.diff(interpolated_traj, axis=0), axis=1)
            log.info("[%s] Trajectory: %d pts  delta min/mean/max = %.4f/%.4f/%.4f rad",
                     name, n_pts, deltas.min(), deltas.mean(), deltas.max())

            rate = rospy.Rate(self.STREAM_RATE_HZ)
            # Buffer time is intentionally long to ensure the skill doesn't end early
            arm.goto_joints(interpolated_traj[1], duration=5, dynamic=True, buffer_time=20)
            init_time = rospy.Time.now().to_time()
            log.info("[%s] Streaming started (goto_joints dispatched)", name)

            late_count = 0
            dt_expected = 1.0 / self.STREAM_RATE_HZ
            fk_buffer = []  # only populated in debug mode
            for i in range(2, n_pts):
                t_before = rospy.Time.now().to_time()
                traj_gen_proto_msg = JointPositionSensorMessage(
                    id=i,
                    timestamp=t_before - init_time,
                    joints=interpolated_traj[i]
                )
                ros_msg = make_sensor_group_msg(
                    trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                        traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
                )
                pub.publish(ros_msg)
                if self.collision_checker is not None:
                    Tcurr, _ = self.collision_checker.ForwardKin(interpolated_traj[i])
                    T = Tcurr[-1]
                    fk_buffer.append((t_before - init_time, T[0:3, 3].copy(), T[0:3, 0:3].copy()))
                rate.sleep()
                elapsed = rospy.Time.now().to_time() - t_before
                if elapsed > dt_expected * 1.5:
                    late_count += 1
                    log.debug("[%s] Late publish at step %d: %.3f ms (expected %.1f ms)",
                              name, i, elapsed * 1000, dt_expected * 1000)

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
            pub.publish(ros_msg)
            log.info("[%s] Termination message sent", name)

            if fk_buffer:
                script_dir = os.path.dirname(os.path.abspath(__file__))
                log_dir = os.path.join(script_dir, "logs")
                csv_path = os.path.join(log_dir,
                    f"traj_{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
                with open(csv_path, 'w') as f:
                    f.write("t,px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22\n")
                    for t, pos, rot in fk_buffer:
                        r = rot.flatten()
                        f.write(f"{t:.4f},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                                f"{r[0]:.4f},{r[1]:.4f},{r[2]:.4f},"
                                f"{r[3]:.4f},{r[4]:.4f},{r[5]:.4f},"
                                f"{r[6]:.4f},{r[7]:.4f},{r[8]:.4f}\n")
                log.info("[%s] FK trajectory saved to %s", name, csv_path)

        except Exception:
            log.critical("[%s] _execute_plan crashed", name, exc_info=True)
            raise


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="PRM-based point-to-point motion on the real Franka robot."
    )
    parser.add_argument("--sim", action="store_true",
                        help="Run MuJoCo visualization alongside the real robot")
    parser.add_argument("--sim-only", action="store_true",
                        help="Run MuJoCo visualization only (no real robot)")
    parser.add_argument("--traj", choices=["ramp", "minjerk"], default="ramp",
                        help="Trajectory generator: 'ramp' (default) or 'minjerk'")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Motion duration in seconds for minjerk (default: 5.0)")
    parser.add_argument("--debug", action="store_true",
                        help="Log full trajectory FK poses (pos+rot) to a separate CSV file")
    args = parser.parse_args()

    random.seed(13)


    collision_checker = SimpleFranka.SimpleFrankArm()  # offline collision checker — no robot connection
    log.info("Franka offline collision checker initialized")

    if args.sim_only:
        left_arm  = None
        right_arm = None
        q_init_1  = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])
        q_init_2  = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])
        log.info("Sim-only mode: using default start joints")
    else:
        log.info("Connecting to real Franka robots...")
        left_arm  = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1)
        log.info("Connected to arm1 (left)")
        right_arm = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2, init_node=False)
        log.info("Connected to arm2 (right)")
        q_init_1  = left_arm.get_joints()
        Tcurr_1, _ = collision_checker.ForwardKin(q_init_1)
        log.info("Arm1 start pose: pos=%s  rot=\n%s",
                 np.round(Tcurr_1[-1][0:3, 3], 4), np.round(Tcurr_1[-1][0:3, 0:3], 4))
        q_init_2  = right_arm.get_joints()
        Tcurr_2, _ = collision_checker.ForwardKin(q_init_2)
        log.info("Arm2 start pose: pos=%s  rot=\n%s",
                 np.round(Tcurr_2[-1][0:3, 3], 4), np.round(Tcurr_2[-1][0:3, 0:3], 4))

    # ── Set goal task positions here (x, y, z in metres, arm base frame) ──────
    task_pos_1 = [0.3, -0.5, 0.3]
    task_pos_1 = [0.2, -0.4, 0.3]
    task_pos_2 = [0.2, 0.0, 0.4]

    q_goal_1 = compute_ik(task_pos_1)
    q_goal_2 = compute_ik(task_pos_2)
    # # Hardcoded override: IK solution for arm 2 is unreliable for this pose; use verified joints directly.
    # q_goal_2 = np.array([-0.3957843, -1.61734739,  1.49835695,
    #                       -2.58231489,  1.51090166,  1.57524887,  0.9455237])
    # ──────────────────────────────────────────────────────────────────────────

    log.info("Arm1  start=%s  goal=%s", np.round(q_init_1, 3), np.round(q_goal_1, 3))
    log.info("Arm2  start=%s  goal=%s", np.round(q_init_2, 3), np.round(q_goal_2, 3))

    plan1 = prm_query(q_init_1, q_goal_1, collision_checker, PRM_FILE_ARM1)
    if plan1 is None:
        sys.exit(1)
    log.debug("FK poses for Left Arm (arm1) plan:")
    for idx, q in enumerate(plan1):
        Tcurr, _ = collision_checker.ForwardKin(q)
        pos = Tcurr[-1][0:3, 3]
        rot = Tcurr[-1][0:3, 0:3]
        log.debug("  arm1 wp %d: pos=%s", idx, np.round(pos, 4))
        log.debug("             rot=\n%s", np.round(rot, 4))

    plan2 = prm_query(q_init_2, q_goal_2, collision_checker, PRM_FILE_ARM2)
    if plan2 is None:
        sys.exit(1)
    log.debug("FK poses for Right Arm (arm2) plan:")
    for idx, q in enumerate(plan2):
        Tcurr, _ = collision_checker.ForwardKin(q)
        pos = Tcurr[-1][0:3, 3]
        rot = Tcurr[-1][0:3, 0:3]
        log.debug("  arm2 wp %d: pos=%s", idx, np.round(pos, 4))
        log.debug("             rot=\n%s", np.round(rot, 4))

    executor = DualArmExecutor(left_arm, right_arm, traj_type=args.traj,
                               minjerk_duration=args.duration,
                               collision_checker=collision_checker if args.debug else None)

    if args.sim_only:
        viz = MuJoCoVisualizer(plan1, plan2)
        executor.execute_sim_only(plan1, plan2, viz)
    elif args.sim:
        viz = MuJoCoVisualizer(plan1, plan2)
        executor.execute_with_sim(plan1, plan2, viz)
        # left_arm.reset_joints(block=True)
        # right_arm.reset_joints(block=True)
    else:
        executor.execute_real_only(plan1, plan2)
        # left_arm.reset_joints(block=True)
        # right_arm.reset_joints(block=True)
