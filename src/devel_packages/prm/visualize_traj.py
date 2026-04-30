#!/usr/bin/env python3
"""
Visualize a trajectory CSV produced by franka_prm_dual_arm.py --debug.

Usage:
    python visualize_traj.py logs/traj_arm1_<timestamp>.csv
    python visualize_traj.py logs/traj_arm1.csv logs/traj_arm2.csv   # overlay two arms
    python visualize_traj.py logs/traj_arm1.csv --speed 2.0          # 2x playback speed
    python visualize_traj.py logs/traj_arm1.csv --save out.gif       # save animation
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

AXIS_LEN   = 0.04   # length of each frame axis arrow (metres)
TRAIL_LEN  = 50     # number of past positions to show in the trail


def load_csv(path):
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    t    = data[:, 0]
    pos  = data[:, 1:4]
    rot  = data[:, 4:13].reshape(-1, 3, 3)
    return t, pos, rot


def draw_frame(ax, pos, rot, scale, colors=('r', 'g', 'b'), alpha=1.0):
    """Draw x/y/z axes of a rotation frame at pos using line segments."""
    lines = []
    for j, c in enumerate(colors):
        tip = pos + rot[:, j] * scale
        ln, = ax.plot([pos[0], tip[0]], [pos[1], tip[1]], [pos[2], tip[2]],
                      color=c, alpha=alpha, linewidth=2.0)
        lines.append(ln)
    return lines


def make_figure(datasets):
    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')

    all_pos = np.vstack([pos for _, pos, _ in datasets])
    margin  = 0.05
    ax.set_xlim(all_pos[:, 0].min() - margin, all_pos[:, 0].max() + margin)
    ax.set_ylim(all_pos[:, 1].min() - margin, all_pos[:, 1].max() + margin)
    ax.set_zlim(all_pos[:, 2].min() - margin, all_pos[:, 2].max() + margin)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("End-effector trajectory")

    return fig, ax


def animate(datasets, speed=1.0, save_path=None):
    arm_colors = [
        ('r', 'g', 'b'),        # arm 1: standard RGB axes
        ('darkred', 'darkgreen', 'navy'),  # arm 2: darker shade
    ]
    trail_colors = ['steelblue', 'darkorange']

    # Subsample each dataset by the speed factor so we render fewer frames
    # rather than trying to play faster than the display refresh rate.
    # e.g. speed=2.0 -> keep every 2nd point, playing back in half the time.
    step = max(1, round(speed))
    datasets = [(t[::step], pos[::step], rot[::step]) for t, pos, rot in datasets]

    fig, ax = make_figure(datasets)

    # Pre-build artist containers per arm
    arm_artists = []
    for k, (t, pos, rot) in enumerate(datasets):
        trail_line, = ax.plot([], [], [], '-', color=trail_colors[k % 2],
                              linewidth=1.2, alpha=0.6,
                              label=f"arm{k+1} trail")
        frame_lines = draw_frame(ax, pos[0], rot[0], AXIS_LEN,
                                 colors=arm_colors[k % 2], alpha=0.0)
        time_text = ax.text2D(0.02, 0.95 - k * 0.06, "", transform=ax.transAxes,
                              fontsize=9)
        arm_artists.append((trail_line, frame_lines, time_text))

    ax.legend(loc='upper right', fontsize=8)

    # Determine frame count from the longest trajectory
    n_frames = max(len(t) for t, _, _ in datasets)

    def update(frame):
        artists_out = []
        for k, (t, pos, rot) in enumerate(datasets):
            # Clamp frame index for shorter trajectories
            i = min(frame, len(t) - 1)
            trail_line, frame_lines, time_text = arm_artists[k]

            # Update trail (full history from start to current frame)
            trail_line.set_data(pos[:i+1, 0], pos[:i+1, 1])
            trail_line.set_3d_properties(pos[:i+1, 2])

            # Remove old frame lines and redraw
            for ln in frame_lines:
                ln.remove()
            new_frame_lines = draw_frame(ax, pos[i], rot[i], AXIS_LEN,
                                         colors=arm_colors[k % 2], alpha=1.0)
            arm_artists[k] = (trail_line, new_frame_lines, time_text)

            time_text.set_text(f"arm{k+1}  t={t[i]:.2f}s  "
                               f"pos=[{pos[i,0]:.3f}, {pos[i,1]:.3f}, {pos[i,2]:.3f}]")

            artists_out += [trail_line, time_text] + new_frame_lines

        return artists_out

    # Real-time interval based on median dt, adjusted for playback speed
    all_t = datasets[0][0]
    dt_ms = float(np.median(np.diff(all_t)) * 1000 / speed)
    dt_ms = max(dt_ms, 16)  # cap at ~60fps

    anim = animation.FuncAnimation(fig, update, frames=n_frames,
                                   interval=dt_ms, blit=False, repeat=False)

    if save_path:
        writer = animation.PillowWriter(fps=int(1000 / dt_ms))
        anim.save(save_path, writer=writer)
        print(f"Saved to {save_path}")
    else:
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize FK trajectory CSV(s)")
    parser.add_argument("csvfiles", nargs='+', help="One or two trajectory CSV files")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--save", type=str, default=None,
                        help="Save animation to this file (e.g. out.gif)")
    args = parser.parse_args()

    datasets = []
    for path in args.csvfiles:
        t, pos, rot = load_csv(path)
        print(f"Loaded {len(t)} poses from {path}")
        datasets.append((t, pos, rot))

    animate(datasets, speed=args.speed, save_path=args.save)
