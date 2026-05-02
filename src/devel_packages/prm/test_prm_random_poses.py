#!/usr/bin/env python3
"""
Random-pose PRM stress test for a single Franka arm.

Samples x valid Cartesian poses, plans a PRM path to each one from the
previous pose (forming a chain), and optionally executes on the real robot
and/or MuJoCo.  All runs are logged to a timestamped folder under logs/.

Usage examples:
    # 5 poses, label zone 1, arm 1, sim only
    python test_prm_random_poses.py --poses 5 --zone 1 --arm 1 --mode sim-only

    # 10 poses, free space, arm 2, real robot only
    python test_prm_random_poses.py --poses 10 --zone free --arm 2 --mode real-only

    # 8 poses, label zone 2, arm 1, real + sim
    python test_prm_random_poses.py --poses 8 --zone 2 --arm 1 --mode with-sim

Sim modes:
    real-only   – execute on real robot, no MuJoCo
    with-sim    – execute on real robot AND mirror in MuJoCo
    sim-only    – MuJoCo only, no real robot connection
"""

import argparse
import json
import logging
import os
import pickle
import random
import sys
import time
from datetime import datetime

import numpy as np

import devel_packages.prm.SimpleFranka as SimpleFranka
import RobotUtil as rt


# ── Joint limits (from PRMGenerator) ─────────────────────────────────────────

_QMIN = np.array([-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
_QMAX = np.array([ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# Workspace filter: ee_x > 0, 0 < ee_z < 0.7  (same as PRMGenerator)
_EE_X_MIN = 0.05   # small margin so IK is well-conditioned
_EE_Z_MIN = 0.10
_EE_Z_MAX = 0.65

_DEFAULT_Q = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ── Logging setup ─────────────────────────────────────────────────────────────

def setup_logging(run_dir: str) -> logging.Logger:
    """Configure root 'test_prm' logger with console + per-run file handler."""
    fmt       = "%(asctime)s [%(levelname)-8s] %(name)s: %(message)s"
    datefmt   = "%Y-%m-%dT%H:%M:%S"
    formatter = logging.Formatter(fmt, datefmt=datefmt)

    logger = logging.getLogger("test_prm")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    logger.handlers.clear()

    sh = logging.StreamHandler(sys.stdout)
    sh.setLevel(logging.INFO)
    sh.setFormatter(formatter)
    logger.addHandler(sh)

    log_path = os.path.join(run_dir, "run.log")
    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    logger.info("Log file: %s", log_path)
    return logger


# ── PRM file path ─────────────────────────────────────────────────────────────

def get_prm_file(arm_number: int, label_zone) -> str:
    """Return the PRM file path for the given arm and label zone.

    label_zone : 1 | 2 | 'free' | None   (None treated as 'free')
    """
    if label_zone in (None, "free"):
        fname = f"myPRM_arm{arm_number}_free.p"
    else:
        fname = f"myPRM_arm{arm_number}_labelZone{label_zone}.p"
    return os.path.join(SCRIPT_DIR, "prm_files", fname)


# ── Valid-pose sampler ────────────────────────────────────────────────────────

def sample_valid_poses(
    n: int,
    arm_number: int,
    label_zone,
    seed: int,
    log: logging.Logger,
    max_attempts_per_pose: int = 5000,
) -> list:
    """Sample n valid (collision-free, reachable) Cartesian poses.

    A pose is a dict with keys: pos [x,y,z], q [7-DOF joints], id (0-indexed).
    Collision is checked in joint space using the PRM's obstacle set.
    The end-effector position is also used as the Cartesian target so we can
    verify IK round-trips later.

    Returns a list of pose dicts (len == n), or raises RuntimeError if sampling
    fails after exhausting attempts.
    """
    rng = random.Random(seed)
    np.random.seed(seed)

    prm_file = get_prm_file(arm_number, label_zone)
    if not os.path.exists(prm_file):
        raise FileNotFoundError(
            f"PRM file not found: {prm_file}\n"
            "Generate it first with generate_prm_files.py"
        )

    log.info("Loading PRM from: %s", prm_file)
    with open(prm_file, "rb") as f:
        _verts   = pickle.load(f)
        _edges   = pickle.load(f)
        obs_pts  = pickle.load(f)
        obs_axes = pickle.load(f)
    log.info("PRM loaded (%d vertices)", len(_verts))

    checker = SimpleFranka.FrankArm(arm_number)

    import ikpy.chain
    urdf_file = os.path.join(SCRIPT_DIR, "../orio/panda_arm_hand.urdf")
    ik_chain  = ikpy.chain.Chain.from_urdf_file(urdf_file, base_elements=["panda_link0"])
    clen      = len(ik_chain.links)
    ik_chain.active_links_mask = [False] + [True]*7 + [False]*(clen - 8)

    poses   = []
    attempt = 0
    log.info("Sampling %d valid poses (seed=%d)…", n, seed)

    while len(poses) < n:
        attempt += 1
        if attempt > n * max_attempts_per_pose:
            raise RuntimeError(
                f"Could only find {len(poses)}/{n} valid poses after "
                f"{attempt} attempts"
            )

        # Sample random joint config (same bounds as PRMGenerator)
        q = np.array([rng.uniform(lo, hi) for lo, hi in zip(_QMIN, _QMAX)])

        # Workspace filter
        Tcurr, _ = checker.ForwardKin(q)
        ee = Tcurr[-1]
        ee_x, ee_z = float(ee[0, 3]), float(ee[2, 3])
        if ee_x < _EE_X_MIN or ee_z < _EE_Z_MIN or ee_z > _EE_Z_MAX:
            continue

        # Collision check
        if checker.DetectCollision(q, obs_pts, obs_axes):
            continue

        pos = [float(ee[0, 3]), float(ee[1, 3]), float(ee[2, 3])]
        pose_id = len(poses)
        poses.append({"id": pose_id, "pos": pos, "q": q.tolist()})
        log.info(
            "  Pose %d/%d sampled after %d attempts — pos=[%.3f, %.3f, %.3f]",
            pose_id + 1, n, attempt, *pos,
        )
        attempt = 0  # reset per-pose attempt counter

    log.info("All %d poses sampled successfully.", n)
    return poses


# ── Artefact saving helpers ───────────────────────────────────────────────────

def save_pose_json(run_dir: str, poses: list, log: logging.Logger):
    path = os.path.join(run_dir, "sampled_poses.json")
    with open(path, "w") as f:
        json.dump(poses, f, indent=2)
    log.info("Sampled poses saved → %s", path)


def save_prm_run(run_dir: str, pose_id: int, pos: list, plan: list, log: logging.Logger):
    """Pickle the raw PRM waypoint list for one pose."""
    tag  = f"pose{pose_id:03d}_x{pos[0]:.3f}_y{pos[1]:.3f}_z{pos[2]:.3f}"
    path = os.path.join(run_dir, f"prm_run_{tag}.pkl")
    with open(path, "wb") as f:
        pickle.dump(plan, f)
    log.info("PRM waypoints saved  → %s", path)
    return path


def save_traj(run_dir: str, pose_id: int, pos: list, traj: np.ndarray, log: logging.Logger):
    """Save the interpolated trajectory (N×7) as a .npy file."""
    tag  = f"pose{pose_id:03d}_x{pos[0]:.3f}_y{pos[1]:.3f}_z{pos[2]:.3f}"
    path = os.path.join(run_dir, f"traj_{tag}.npy")
    np.save(path, traj)
    log.info("Interpolated traj    → %s  shape=%s", path, traj.shape)
    return path


# ── Single-pose execution ─────────────────────────────────────────────────────

def run_pose(
    executor,          # SingleArmExecutor
    pose: dict,
    q_current: np.ndarray,
    q_idle: np.ndarray,  # fixed joint config for the idle arm
    label_zone,
    mode: str,         # 'real-only' | 'with-sim' | 'sim-only'
    run_dir: str,
    viz,               # MuJoCoVisualizer or None
    log: logging.Logger,
) -> np.ndarray:
    """Plan and execute motion to one pose. Returns the joint config reached."""

    pose_id = pose["id"]
    pos     = pose["pos"]
    log.info(
        "═══ Pose %d | pos=[%.3f, %.3f, %.3f] | mode=%s ═══",
        pose_id, *pos, mode,
    )

    t_plan_start = time.time()
    plan = executor._prm_query(q_current, np.array(pose["q"]),
                               get_prm_file(executor.arm_number, label_zone))
    t_plan = time.time() - t_plan_start

    if plan is None:
        log.error("Pose %d: PRM planning FAILED (%.2f s) — skipping", pose_id, t_plan)
        return q_current

    log.info("Pose %d: planning OK — %d waypoints in %.2f s", pose_id, len(plan), t_plan)

    # Save raw PRM waypoints
    save_prm_run(run_dir, pose_id, pos, plan, log)

    # Build interpolated trajectory
    traj = executor._build_interpolated_traj(np.array(plan))
    save_traj(run_dir, pose_id, pos, traj, log)

    # Execute
    # Map active/idle arms to q1 (left) and q2 (right) for the visualizer
    def _viz_args(q_active):
        if executor.arm_number == 1:
            return q_active, q_idle
        else:
            return q_idle, q_active

    if mode == "sim-only":
        if viz is not None:
            mj_substeps = max(1, round(1.0 / (executor.STREAM_RATE_HZ * viz.model.opt.timestep)))
            dt_control  = 1.0 / executor.STREAM_RATE_HZ
            for q_des in traj:
                t0 = time.time()
                viz.step_impedance(*_viz_args(q_des), n_substeps=mj_substeps)
                time.sleep(max(0.0, dt_control - (time.time() - t0)))
        else:
            log.warning("Pose %d: sim-only mode but no visualizer provided — dry run only", pose_id)

    elif mode == "real-only":
        executor._execute_plan(traj)
        executor.fa.wait_for_skill()

    elif mode == "with-sim":
        import threading
        t = threading.Thread(target=executor._execute_plan, args=(traj,),
                             name=f"arm{executor.arm_number}_pose{pose_id}")
        t.start()
        if viz is not None:
            mj_substeps = max(1, round(1.0 / (executor.STREAM_RATE_HZ * viz.model.opt.timestep)))
            for q_des in traj:
                viz.step_impedance(*_viz_args(q_des), n_substeps=mj_substeps)
        t.join()
        executor.fa.wait_for_skill()

    log.info("Pose %d: execution complete", pose_id)
    return np.array(pose["q"])


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="PRM random-pose stress test for a single Franka arm.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--poses", type=int, default=5, metavar="X",
                        help="Number of random poses to sample and visit")
    parser.add_argument("--zone", default="free",
                        help="Label zone: 1, 2, or 'free'")
    parser.add_argument("--arm", type=int, choices=[1, 2], default=1,
                        help="Arm number (1 = left, 2 = right)")
    parser.add_argument("--mode", choices=["real-only", "with-sim", "sim-only"],
                        default="sim-only",
                        help="Execution mode")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for reproducible pose sampling")
    parser.add_argument("--traj", choices=["ramp", "minjerk"], default="ramp",
                        help="Trajectory interpolation type")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Min-jerk motion duration in seconds (minjerk only)")
    args = parser.parse_args()

    # Parse label zone
    if args.zone == "free":
        label_zone = None
    else:
        try:
            label_zone = int(args.zone)
        except ValueError:
            print(f"ERROR: --zone must be 1, 2, or 'free', got '{args.zone}'")
            sys.exit(1)

    sim_only = args.mode == "sim-only"

    # ── Create timestamped run directory ──────────────────────────────────────
    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    zone_tag = "free" if label_zone is None else f"zone{label_zone}"
    run_name = f"{ts}_arm{args.arm}_{zone_tag}_{args.poses}poses_{args.mode}"
    run_dir  = os.path.join(SCRIPT_DIR, "logs", run_name)
    os.makedirs(run_dir, exist_ok=True)

    log = setup_logging(run_dir)
    log.info("Run directory: %s", run_dir)
    log.info(
        "Config | arm=%d  zone=%s  poses=%d  mode=%s  seed=%d  traj=%s",
        args.arm, args.zone, args.poses, args.mode, args.seed, args.traj,
    )

    # Save run config for reproducibility
    config_path = os.path.join(run_dir, "config.json")
    with open(config_path, "w") as f:
        json.dump(vars(args), f, indent=2)
    log.info("Run config saved → %s", config_path)

    # ── Sample poses ──────────────────────────────────────────────────────────
    try:
        poses = sample_valid_poses(
            n=args.poses,
            arm_number=args.arm,
            label_zone=label_zone,
            seed=args.seed,
            log=log,
        )
    except (FileNotFoundError, RuntimeError) as e:
        log.critical("Pose sampling failed: %s", e)
        sys.exit(1)

    save_pose_json(run_dir, poses, log)

    # ── Import executor (deferred so --help works without ROS) ───────────────
    # Suppress the franka_prm logger from also writing to its own log file
    # by re-using our run_dir handler via the parent logger.
    from franka_prm_single_arm import SingleArmExecutor, _setup_logger
    _setup_logger(log_to_file=False)   # prevent franka_prm from opening its own log

    # Route franka_prm messages into our logger
    fp_log = logging.getLogger("franka_prm")
    fp_log.handlers.clear()
    for h in log.handlers:
        fp_log.addHandler(h)
    fp_log.propagate = False

    # ── Instantiate executor ──────────────────────────────────────────────────
    log.info("Initialising SingleArmExecutor (arm=%d, sim_only=%s)…", args.arm, sim_only)
    executor = SingleArmExecutor(
        arm_number=args.arm,
        traj_type=args.traj,
        minjerk_duration=args.duration,
        log_fk=False,
        init_node=True,
        sim_only=sim_only,
    )

    # Starting joint position
    if executor.fa is not None:
        q_current = executor.fa.get_joints()
        log.info("Real robot q_start: %s", np.round(q_current, 4))
    else:
        q_current = _DEFAULT_Q.copy()
        log.info("Sim-only: using default q_start: %s", np.round(q_current, 4))

    # ── MuJoCo visualizer ─────────────────────────────────────────────────────
    q_idle = _DEFAULT_Q.copy()  # idle arm stays at its default pose

    viz = None
    if args.mode in ("sim-only", "with-sim"):
        from franka_prm_dual_arm import MuJoCoVisualizer
        q1_init = q_current if args.arm == 1 else q_idle
        q2_init = q_idle    if args.arm == 1 else q_current
        viz = MuJoCoVisualizer([q1_init], [q2_init])
        log.info("MuJoCo visualizer created")

    # ── Main loop: visit each pose in sequence ────────────────────────────────
    results = []
    t_total_start = time.time()

    for pose in poses:
        t_pose_start = time.time()
        q_reached = run_pose(
            executor=executor,
            pose=pose,
            q_current=q_current,
            q_idle=q_idle,
            label_zone=label_zone,
            mode=args.mode,
            run_dir=run_dir,
            viz=viz,
            log=log,
        )
        elapsed = time.time() - t_pose_start
        q_current = q_reached
        results.append({
            "pose_id":  pose["id"],
            "pos":      pose["pos"],
            "q_target": pose["q"],
            "q_reached": q_reached.tolist(),
            "elapsed_s": round(elapsed, 3),
        })
        log.info("Pose %d done in %.2f s — cumulative: %.2f s",
                 pose["id"], elapsed, time.time() - t_total_start)

    # ── Save summary ──────────────────────────────────────────────────────────
    summary = {
        "run_name":    run_name,
        "config":      vars(args),
        "total_s":     round(time.time() - t_total_start, 3),
        "poses_done":  len(results),
        "results":     results,
    }
    summary_path = os.path.join(run_dir, "summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    log.info("Summary saved → %s", summary_path)
    log.info(
        "All %d poses complete in %.2f s total.",
        len(results), summary["total_s"],
    )

    # ── Keep MuJoCo window open ───────────────────────────────────────────────
    if viz is not None:
        log.info("Holding MuJoCo viewer open — close the window to exit.")
        while viz.viewer.is_running():
            viz.viewer.sync()
            time.sleep(0.05)


if __name__ == "__main__":
    main()
