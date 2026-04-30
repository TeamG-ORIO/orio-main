#!/usr/bin/env python3
"""
Compare desired vs actual end-effector trajectory from a logged CSV file.

CSV columns:
  t, des_px, des_py, des_pz,
  des_r00..des_r22  (3x3 rotation matrix, row-major),
  act_px, act_py, act_pz,
  act_r00..act_r22,
  pos_err_m, tilt_deg
"""

import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless-safe; change to "TkAgg" if you have a display
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401 – needed for 3-D projection


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """Return ZYX Euler angles (roll, pitch, yaw) in degrees from a 3x3 matrix."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.degrees(np.arctan2( R[2, 1],  R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0],  sy))
        yaw   = np.degrees(np.arctan2( R[1, 0],  R[0, 0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1, 2],  R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0],  sy))
        yaw   = 0.0
    return np.array([roll, pitch, yaw])


def rows_to_euler(mats_flat: np.ndarray) -> np.ndarray:
    """Convert each row's flat 9-element rotation matrix to Euler angles (N,3)."""
    mats = mats_flat.reshape(-1, 3, 3)
    return np.array([rotation_matrix_to_euler_zyx(R) for R in mats])


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Plot desired vs actual end-effector trajectory.")
    parser.add_argument("csv", nargs="?",
                        default="logs/traj_arm1_20260412_134104.csv",
                        help="Path to trajectory CSV (default: latest in logs/)")
    parser.add_argument("--out", default=None,
                        help="Output PNG filename (default: <csv_stem>_comparison.png)")
    args = parser.parse_args()

    csv_path = args.csv
    out_path = args.out or csv_path.replace(".csv", "_comparison.png")

    print(f"Loading {csv_path} …")
    # Parse header then data with numpy
    with open(csv_path) as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    def col(name):
        return data[:, header.index(name)]

    t = col("t")

    # ---- position ----------------------------------------------------------
    des_pos = np.column_stack([col("des_px"), col("des_py"), col("des_pz")])
    act_pos = np.column_stack([col("act_px"), col("act_py"), col("act_pz")])

    # ---- orientation (Euler angles) ----------------------------------------
    des_rot_flat = np.column_stack(
        [col(f"des_r{i}{j}") for i in range(3) for j in range(3)])
    act_rot_flat = np.column_stack(
        [col(f"act_r{i}{j}") for i in range(3) for j in range(3)])
    des_euler = rows_to_euler(des_rot_flat)   # (N, 3)  degrees [roll, pitch, yaw]
    act_euler = rows_to_euler(act_rot_flat)

    # ---- pre-computed error columns ----------------------------------------
    pos_err  = col("pos_err_m") * 1e3          # → mm
    tilt_deg = col("tilt_deg")

    # ========================================================================
    # Figure layout:
    #   Row 0: X, Y, Z position (desired vs actual)
    #   Row 1: Roll, Pitch, Yaw (desired vs actual)
    #   Row 2: Position error [mm]  |  Orientation tilt [deg]
    #   Row 3: 3-D trajectory
    # ========================================================================
    fig = plt.figure(figsize=(16, 20))
    fig.suptitle(f"Desired vs Actual Trajectory\n{csv_path}", fontsize=13)

    gs = fig.add_gridspec(4, 3, hspace=0.45, wspace=0.35)

    axis_labels = ["X", "Y", "Z"]
    colors_des  = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    colors_act  = ["#aec7e8", "#ffbb78", "#98df8a"]

    # --- row 0: position axes -----------------------------------------------
    for col, (label, cd, ca) in enumerate(
            zip(axis_labels, colors_des, colors_act)):
        ax = fig.add_subplot(gs[0, col])
        ax.plot(t, des_pos[:, col], color=cd, lw=1.5, label="Desired")
        ax.plot(t, act_pos[:, col], color=ca, lw=1.5, linestyle="--", label="Actual")
        ax.set_title(f"Position – {label}")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(f"{label} [m]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # --- row 1: orientation axes --------------------------------------------
    euler_labels = ["Roll", "Pitch", "Yaw"]
    for col, (label, cd, ca) in enumerate(
            zip(euler_labels, colors_des, colors_act)):
        ax = fig.add_subplot(gs[1, col])
        ax.plot(t, des_euler[:, col], color=cd, lw=1.5, label="Desired")
        ax.plot(t, act_euler[:, col], color=ca, lw=1.5, linestyle="--", label="Actual")
        ax.set_title(f"Orientation – {label}")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("[deg]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # --- row 2: error summary -----------------------------------------------
    ax_perr = fig.add_subplot(gs[2, :2])
    ax_perr.plot(t, pos_err, color="#d62728", lw=1.5)
    ax_perr.fill_between(t, pos_err, alpha=0.2, color="#d62728")
    ax_perr.set_title("Position Error")
    ax_perr.set_xlabel("Time [s]")
    ax_perr.set_ylabel("Error [mm]")
    ax_perr.grid(True, alpha=0.3)
    ax_perr.annotate(f"max {pos_err.max():.2f} mm  |  mean {pos_err.mean():.2f} mm",
                     xy=(0.98, 0.95), xycoords="axes fraction",
                     ha="right", va="top", fontsize=9,
                     bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))

    ax_terr = fig.add_subplot(gs[2, 2])
    ax_terr.plot(t, tilt_deg, color="#9467bd", lw=1.5)
    ax_terr.fill_between(t, tilt_deg, alpha=0.2, color="#9467bd")
    ax_terr.set_title("Orientation Tilt Error")
    ax_terr.set_xlabel("Time [s]")
    ax_terr.set_ylabel("Tilt [deg]")
    ax_terr.grid(True, alpha=0.3)
    ax_terr.annotate(f"max {tilt_deg.max():.2f}°  |  mean {tilt_deg.mean():.2f}°",
                     xy=(0.98, 0.95), xycoords="axes fraction",
                     ha="right", va="top", fontsize=9,
                     bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))

    # --- row 3: 3-D trajectory ----------------------------------------------
    ax3d = fig.add_subplot(gs[3, :], projection="3d")
    ax3d.plot(des_pos[:, 0], des_pos[:, 1], des_pos[:, 2],
              color="#1f77b4", lw=1.5, label="Desired")
    ax3d.plot(act_pos[:, 0], act_pos[:, 1], act_pos[:, 2],
              color="#d62728", lw=1.5, linestyle="--", label="Actual")
    ax3d.scatter(*des_pos[0],  color="#1f77b4", s=50, marker="o", zorder=5)   # start
    ax3d.scatter(*des_pos[-1], color="#1f77b4", s=80, marker="*", zorder=5)   # end
    ax3d.scatter(*act_pos[0],  color="#d62728", s=50, marker="o", zorder=5)
    ax3d.scatter(*act_pos[-1], color="#d62728", s=80, marker="*", zorder=5)
    ax3d.set_title("3-D End-Effector Trajectory")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    ax3d.legend(fontsize=9)

    # ---- summary stats to console ------------------------------------------
    print("\n=== Trajectory Summary ===")
    print(f"  Duration       : {t[-1]:.3f} s  ({len(t)} samples)")
    print(f"  Position error : max={pos_err.max():.3f} mm  "
          f"mean={pos_err.mean():.3f} mm  rms={np.sqrt((pos_err**2).mean()):.3f} mm")
    print(f"  Tilt error     : max={tilt_deg.max():.3f}°   "
          f"mean={tilt_deg.mean():.3f}°   rms={np.sqrt((tilt_deg**2).mean()):.3f}°")
    path_len = np.sum(np.linalg.norm(np.diff(des_pos, axis=0), axis=1))
    print(f"  Desired path length : {path_len*1e3:.1f} mm")

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\nSaved → {out_path}")


if __name__ == "__main__":
    main()
