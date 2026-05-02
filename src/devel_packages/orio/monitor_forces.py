#!/usr/bin/env python3
"""
monitor_forces.py
-----------------
Run in a separate terminal while state_machine_integrated_waypts.py is running.
Displays live scrolling plots of:
  - Joint torques  (7 joints, Nm)    via fa.get_joint_torques()
  - EE force       (Fx Fy Fz, N)     via fa.get_ee_force_torque()
  - EE torque      (Tx Ty Tz, Nm)    via fa.get_ee_force_torque()

One window per arm, three subplots each. All values are also saved to a
timestamped CSV log in ./logs/.

Usage:
    python3 monitor_forces.py [--rate HZ] [--robot 1] [--robot 2] [--window S]

Defaults: both arms, 10 Hz, 30 s scrolling window.
"""

import argparse
import csv
import datetime
import os
import sys
import threading
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")          # works headless-free; swap to "Qt5Agg" if preferred
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import rospy
from frankapy import FrankaArm

# ── Constants ─────────────────────────────────────────────────────────────────

_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

JOINT_LABELS  = [f"J{i+1}" for i in range(7)]
EE_F_LABELS   = ["Fx", "Fy", "Fz"]
EE_T_LABELS   = ["Tx", "Ty", "Tz"]

# Colour palettes — one colour per signal in each subplot
_JOINT_COLORS = plt.cm.tab10(np.linspace(0, 0.9, 7))
_EEF_COLORS   = ["#e74c3c", "#2ecc71", "#3498db"]   # R G B  → X Y Z
_EET_COLORS   = ["#e67e22", "#1abc9c", "#9b59b6"]


# ── Per-arm data store ────────────────────────────────────────────────────────

class ArmBuffer:
    """Thread-safe ring-buffers for one arm's readings."""

    def __init__(self, window_size: int):
        self.lock = threading.Lock()
        self.times         = deque(maxlen=window_size)
        self.joint_torques = [deque(maxlen=window_size) for _ in range(7)]
        self.ee_force      = [deque(maxlen=window_size) for _ in range(3)]
        self.ee_torque     = [deque(maxlen=window_size) for _ in range(3)]
        self.t0            = time.time()

    def push(self, joint_t, ee_ft):
        with self.lock:
            self.times.append(time.time() - self.t0)
            for i in range(7):
                self.joint_torques[i].append(joint_t[i])
            for i in range(3):
                self.ee_force[i].append(ee_ft[i])
                self.ee_torque[i].append(ee_ft[3 + i])

    def snapshot(self):
        with self.lock:
            t  = list(self.times)
            jt = [list(q) for q in self.joint_torques]
            ef = [list(q) for q in self.ee_force]
            et = [list(q) for q in self.ee_torque]
        return t, jt, ef, et


# ── CSV logging ───────────────────────────────────────────────────────────────

def _open_csv(robot_nums):
    os.makedirs(_LOG_DIR, exist_ok=True)
    ts       = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    arms_tag = "_".join(f"arm{n}" for n in robot_nums)
    path     = os.path.join(_LOG_DIR, f"forces_{arms_tag}_{ts}.csv")

    f = open(path, "w", newline="")
    header = ["timestamp_s", "arm"]
    header += [f"jt_{l}" for l in JOINT_LABELS]
    header += [f"ee_{l}" for l in EE_F_LABELS]
    header += [f"ee_{l}" for l in EE_T_LABELS]
    writer = csv.writer(f)
    writer.writerow(header)
    return f, writer, path


# ── Background polling thread ─────────────────────────────────────────────────

def _poll_loop(arms, buffers, csv_writer, csv_file, poll_interval, stop_event):
    t0 = time.time()
    while not stop_event.is_set() and not rospy.is_shutdown():
        t_start = time.time()
        ts = t_start - t0

        for n, fa in arms.items():
            try:
                joint_t = fa.get_joint_torques()    # (7,) Nm
                ee_ft   = fa.get_ee_force_torque()  # (6,) N / Nm
            except Exception as exc:
                print(f"[ARM {n}] read error: {exc}", file=sys.stderr)
                continue

            buffers[n].push(joint_t, ee_ft)

            row = [f"{ts:.4f}", n] + list(joint_t) + list(ee_ft)
            csv_writer.writerow(row)

        csv_file.flush()

        elapsed    = time.time() - t_start
        sleep_time = poll_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


# ── Plot builder ──────────────────────────────────────────────────────────────

def _build_figure(robot_num, window_s):
    """Create figure + axes for one arm. Returns (fig, ax_jt, ax_ef, ax_et, line dicts)."""
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(f"Arm {robot_num} — Real-time Force / Torque Monitor", fontsize=13, fontweight="bold")
    fig.subplots_adjust(hspace=0.38, left=0.09, right=0.97, top=0.93, bottom=0.07)

    ax_jt, ax_ef, ax_et = axes

    # --- Joint torques ---
    ax_jt.set_title("Joint Torques (Nm)")
    ax_jt.set_ylabel("Nm")
    ax_jt.set_xlim(0, window_s)
    ax_jt.grid(True, linestyle="--", alpha=0.4)
    jt_lines = [ax_jt.plot([], [], lw=1.4, color=_JOINT_COLORS[i], label=JOINT_LABELS[i])[0]
                for i in range(7)]
    ax_jt.legend(loc="upper left", ncol=7, fontsize=7, framealpha=0.5)

    # --- EE Force ---
    ax_ef.set_title("End-Effector Force (N)")
    ax_ef.set_ylabel("N")
    ax_ef.set_xlim(0, window_s)
    ax_ef.grid(True, linestyle="--", alpha=0.4)
    ef_lines = [ax_ef.plot([], [], lw=1.6, color=_EEF_COLORS[i], label=EE_F_LABELS[i])[0]
                for i in range(3)]
    ax_ef.legend(loc="upper left", ncol=3, fontsize=8, framealpha=0.5)

    # --- EE Torque ---
    ax_et.set_title("End-Effector Torque (Nm)")
    ax_et.set_ylabel("Nm")
    ax_et.set_xlabel("Time (s)")
    ax_et.set_xlim(0, window_s)
    ax_et.grid(True, linestyle="--", alpha=0.4)
    et_lines = [ax_et.plot([], [], lw=1.6, color=_EET_COLORS[i], label=EE_T_LABELS[i])[0]
                for i in range(3)]
    ax_et.legend(loc="upper left", ncol=3, fontsize=8, framealpha=0.5)

    return fig, ax_jt, ax_ef, ax_et, jt_lines, ef_lines, et_lines


def _make_updater(buf, window_s, ax_jt, ax_ef, ax_et, jt_lines, ef_lines, et_lines):
    """Return the FuncAnimation callback for one arm's figure."""

    def update(_):
        t, jt, ef, et = buf.snapshot()
        if not t:
            return jt_lines + ef_lines + et_lines

        t_arr = np.asarray(t)
        t_min = max(0.0, t_arr[-1] - window_s)
        t_max = t_min + window_s

        for ax in (ax_jt, ax_ef, ax_et):
            ax.set_xlim(t_min, t_max)

        for i, line in enumerate(jt_lines):
            line.set_data(t_arr, np.asarray(jt[i]))
        ax_jt.relim(); ax_jt.autoscale_view(scalex=False)

        for i, line in enumerate(ef_lines):
            line.set_data(t_arr, np.asarray(ef[i]))
        ax_ef.relim(); ax_ef.autoscale_view(scalex=False)

        for i, line in enumerate(et_lines):
            line.set_data(t_arr, np.asarray(et[i]))
        ax_et.relim(); ax_et.autoscale_view(scalex=False)

        return jt_lines + ef_lines + et_lines

    return update


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Live force/torque plots for Franka arms.")
    parser.add_argument("--rate",   type=float, default=100.0,  help="Poll rate Hz (default 10)")
    parser.add_argument("--window", type=float, default=30.0,  help="Scrolling window seconds (default 30)")
    parser.add_argument("--robot",  type=int,   action="append", dest="robots", metavar="N",
                        help="Robot number (repeatable). Default: 1 and 2.")
    args = parser.parse_args()

    robot_nums    = sorted(set(args.robots)) if args.robots else [1, 2]
    poll_interval = 1.0 / max(args.rate, 0.1)
    window_s      = args.window
    # Ring buffer deep enough for window_s at the given rate, plus headroom
    buf_size      = int(window_s * args.rate * 1.5) + 100

    # ── ROS + arms ──────────────────────────────────────────────────────────
    rospy.init_node("force_torque_monitor", anonymous=True)

    arms = {}
    for n in robot_nums:
        print(f"Connecting to arm {n}...")
        try:
            arms[n] = FrankaArm(
                rosnode_name=f"force_monitor_arm{n}",
                robot_num=n,
                with_gripper=False,
                old_gripper=False,
                init_node=False,
            )
            print(f"  Arm {n} connected.")
        except Exception as exc:
            print(f"  Could not connect to arm {n}: {exc}", file=sys.stderr)

    if not arms:
        print("No arms connected. Exiting.", file=sys.stderr)
        sys.exit(1)

    # ── CSV log ──────────────────────────────────────────────────────────────
    csv_file, csv_writer, log_path = _open_csv(list(arms.keys()))
    print(f"Logging to: {log_path}\n")

    # ── Buffers ──────────────────────────────────────────────────────────────
    buffers = {n: ArmBuffer(buf_size) for n in arms}

    # ── Background polling thread ────────────────────────────────────────────
    stop_event = threading.Event()
    poll_thread = threading.Thread(
        target=_poll_loop,
        args=(arms, buffers, csv_writer, csv_file, poll_interval, stop_event),
        daemon=True,
    )
    poll_thread.start()

    # ── Build one figure per arm, wire up animations ─────────────────────────
    anim_interval_ms = int(poll_interval * 1000)   # redraw at same rate as poll
    anims = []

    for n in arms:
        fig, ax_jt, ax_ef, ax_et, jt_lines, ef_lines, et_lines = _build_figure(n, window_s)
        updater = _make_updater(buffers[n], window_s, ax_jt, ax_ef, ax_et, jt_lines, ef_lines, et_lines)
        ani = animation.FuncAnimation(
            fig, updater,
            interval=anim_interval_ms,
            blit=True,
            cache_frame_data=False,
        )
        anims.append(ani)   # keep reference so GC doesn't kill it

    print("Showing plots — close windows or press Ctrl+C to stop.")
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        poll_thread.join(timeout=2)
        csv_file.close()
        print(f"\nSaved: {log_path}")


if __name__ == "__main__":
    main()
