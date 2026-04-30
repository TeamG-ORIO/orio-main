#!/usr/bin/env python3
"""
Visualize arm collision blocks and scene obstacle blocks in 3D.

Usage:
    python plot_collision_boxes.py --arm 1
    python plot_collision_boxes.py --arm 2
    python plot_collision_boxes.py --arm 1 --joints 0 -0.785 0 -2.356 0 1.571 0.785
    python plot_collision_boxes.py --both
    python plot_collision_boxes.py --both --joints1 0 -0.785 0 -2.356 0 1.571 0.785 --joints2 0 -0.785 0 -2.356 0 1.571 0.785
    python plot_collision_boxes.py --arm 1 --add_waypts
    python plot_collision_boxes.py --both --add_waypts
"""

import argparse
import json
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import combinations

# Allow imports from the prm directory
sys.path.insert(0, os.path.dirname(__file__))
import RobotUtil as rt
import SimpleFranka as SimpleFranka
from scene_obstacles import get_obstacles_for_arm, ARM_BASE_WORLD, _quat_to_rot

_WAYPTS_FILE = os.path.join(os.path.dirname(__file__),
                            "../orio/Target_Task_Poses.json")

# ── Waypoint helpers ──────────────────────────────────────────────────────────

def _load_waypoints():
    with open(_WAYPTS_FILE) as f:
        return json.load(f)


_ARM1_WAYPT_SKIP = {"LABELLING_SAFE", "LABEL_DISPENSER"}
_ARM2_WAYPT_ONLY = {"LABELLING_SAFE", "LABEL_DISPENSER"}


def draw_waypoints(ax, waypoints, arm_H=None, skip=None, only=None):
    """Plot waypoints as stars with name labels.

    waypoints : dict from Target_Task_Poses.json
    arm_H     : 4x4 world_T_arm transform; if given, positions are transformed
                from arm frame to world frame. If None, plotted as-is.
    skip      : set of waypoint names to exclude (ignored if only is set)
    only      : if given, show only waypoints in this set
    """
    skip = skip or set()
    for name, entry in waypoints.items():
        if "translation" not in entry:
            continue
        if only is not None:
            if name not in only:
                continue
        elif name in skip:
            continue
        pos = np.array(entry["translation"])
        if arm_H is not None:
            pos = arm_H[:3, :3] @ pos + arm_H[:3, 3]
        ax.scatter(*pos, marker='*', s=120, c='black', zorder=10)
        ax.text(pos[0], pos[1], pos[2], f" {name}", fontsize=7, color='black')


# ── Box drawing helper ─────────────────────────────────────────────────────────

def _box_edges():
    """Return the 12 pairs of corner indices that form a box wireframe.
    Corners are indexed 1–8 as produced by BlockDesc2Points."""
    idx = list(range(1, 9))
    edges = []
    for i, j in combinations(idx, 2):
        # Two corners are connected if they differ in exactly one half-axis
        # i.e., their XOR expressed via index distance pattern is a power of 2
        # Easier: connected if exactly one of (x,y,z) coordinate differs
        edges.append((i, j))
    return edges  # we'll filter in draw by distance later


def draw_box(ax, points, color, alpha=0.25, lw=1.0):
    """Draw wireframe box from BlockDesc2Points output (9 points: center + 8 corners)."""
    corners = np.array(points[1:])  # shape (8,3)
    center  = np.array(points[0])

    # Identify edges: two corners share an edge if they differ by exactly one axis step
    # We detect this by checking if the vector between them equals 2 * one of the half-axes.
    # Simpler: just draw all C(8,2)=28 pairs and filter by distance == one axis length.
    # Find the 3 unique edge lengths from center to corners.
    half_vecs = corners - center  # (8,3)
    # Each half_vec is ±a ±b ±c; edge exists when two corners differ by 2*a, 2*b, or 2*c
    # The edge lengths are the lengths of the 3 axes.
    # We'll compute axis vectors from the first three half_vecs decomposition.
    # Faster: axis vectors from the homogeneous transform are already in Caxes,
    # but we only have points here.  Use SVD on half_vecs.
    _, _, Vt = np.linalg.svd(half_vecs)
    axis_dirs = Vt[:3]  # 3 principal directions

    # axis extents: project each corner onto each axis
    projs = half_vecs @ axis_dirs.T  # (8,3)
    axis_extents = projs.max(axis=0) - projs.min(axis=0)  # (3,) full widths

    drawn = set()
    for i in range(8):
        for j in range(i + 1, 8):
            diff = corners[i] - corners[j]
            # Project diff onto each axis
            diff_proj = np.abs(axis_dirs @ diff)
            # An edge has diff along exactly one axis equal to the full extent, others ~0
            tol = 1e-6
            nonzero = diff_proj > tol
            if nonzero.sum() == 1:
                k = np.argmax(nonzero)
                if abs(diff_proj[k] - axis_extents[k]) < tol * 10 + 1e-4:
                    ax.plot([corners[i, 0], corners[j, 0]],
                            [corners[i, 1], corners[j, 1]],
                            [corners[i, 2], corners[j, 2]],
                            color=color, linewidth=lw, alpha=alpha + 0.5)


def draw_box_solid_edges(ax, points, color, lw=1.2):
    """Draw only the 12 true edges of the box (no face or space diagonals)."""
    corners = np.array(points[1:])  # (8, 3)
    center  = np.array(points[0])
    half_vecs = corners - center    # each is ±a ±b ±c

    # Use SVD to recover the 3 principal axis directions and their full extents
    _, _, Vt = np.linalg.svd(half_vecs)
    axis_dirs = Vt[:3]              # (3, 3) orthonormal rows
    projs = half_vecs @ axis_dirs.T # (8, 3) signed projections
    axis_extents = projs.max(axis=0) - projs.min(axis=0)  # full widths along each axis

    for i in range(8):
        for j in range(i + 1, 8):
            diff_proj = np.abs(axis_dirs @ (corners[i] - corners[j]))
            # True edge: nonzero along exactly one axis and matches that axis's full extent
            nonzero = diff_proj > 1e-6
            if nonzero.sum() == 1:
                k = np.argmax(nonzero)
                if abs(diff_proj[k] - axis_extents[k]) < 1e-4:
                    ax.plot([corners[i, 0], corners[j, 0]],
                            [corners[i, 1], corners[j, 1]],
                            [corners[i, 2], corners[j, 2]],
                            color=color, linewidth=lw, alpha=0.7)


# ── Coordinate transform helpers ──────────────────────────────────────────────

def _arm_to_world_H(arm_number):
    """Return 4x4 homogeneous transform: world_T_arm for the given arm."""
    base = ARM_BASE_WORLD[arm_number]
    R = _quat_to_rot(base["quat"])
    H = np.eye(4)
    H[:3, :3] = R
    H[:3,  3] = base["pos"]
    return H


def _transform_points(points, H):
    """Apply 4x4 transform H to a list of 3D points (list of arrays, shape (3,))."""
    pts = np.array(points)  # (N, 3)
    return (H[:3, :3] @ pts.T).T + H[:3, 3]


def _transform_box_points(box_pts, H):
    """Transform a BlockDesc2Points result (9-point list) by H into world frame."""
    arr = np.array(box_pts)  # (9, 3): [center, c0..c7]
    transformed = _transform_points(arr, H)
    return transformed.tolist()


# ── Single-arm plot ────────────────────────────────────────────────────────────

def plot_single_arm(arm_number, joints, add_waypts=False):
    arm = SimpleFranka.SimpleFrankArm(arm_number=arm_number)
    arm.CompCollisionBlockPoints(joints)

    blocks = get_obstacles_for_arm(arm_number)
    obs_points = []
    for name, pos, size in blocks:
        H = rt.rpyxyz2H([0., 0., 0.], pos)
        pts, _ = rt.BlockDesc2Points(H, size)
        obs_points.append(pts)

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')

    # Arm skeleton
    Tcurr = arm.Tcurr
    for i in range(len(Tcurr)):
        ax.scatter(Tcurr[i][0, 3], Tcurr[i][1, 3], Tcurr[i][2, 3], c='k', s=20, zorder=5)
        if i == 0:
            ax.plot([0, Tcurr[i][0, 3]], [0, Tcurr[i][1, 3]], [0, Tcurr[i][2, 3]], c='k', lw=1.5)
        else:
            ax.plot([Tcurr[i-1][0, 3], Tcurr[i][0, 3]],
                    [Tcurr[i-1][1, 3], Tcurr[i][1, 3]],
                    [Tcurr[i-1][2, 3], Tcurr[i][2, 3]], c='k', lw=1.5)

    # Arm collision blocks — payload (last block) drawn in green if present
    n_base = 12
    for k, pts in enumerate(arm.Cpoints):
        color = 'green' if k >= n_base else 'blue'
        draw_box_solid_edges(ax, pts, color=color, lw=1.0)

    # Scene obstacle blocks (red)
    for pts in obs_points:
        draw_box_solid_edges(ax, pts, color='red', lw=1.0)

    if add_waypts:
        if arm_number == 2:
            draw_waypoints(ax, _load_waypoints(), only=_ARM2_WAYPT_ONLY)
        else:
            draw_waypoints(ax, _load_waypoints(), skip=_ARM1_WAYPT_SKIP)

    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='blue',   lw=2, label='Arm collision boxes'),
        Line2D([0], [0], color='red',    lw=2, label='Scene obstacle boxes'),
        Line2D([0], [0], color='black',  lw=2, label='Arm skeleton'),
    ]
    if len(arm.Cpoints) > n_base:
        legend_elements.insert(1, Line2D([0], [0], color='green', lw=2, label='Payload box'))
    if add_waypts:
        from matplotlib.lines import Line2D as _L
        legend_elements.append(_L([0], [0], marker='*', color='black', lw=0, markersize=10, label='Waypoints'))
    ax.legend(handles=legend_elements, loc='upper left')

    ax.set_title(f"Arm {arm_number}")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    all_pts = np.concatenate(
        [np.array(p[1:]) for p in arm.Cpoints] +
        [np.array(p[1:]) for p in obs_points]
    )
    lo, hi = all_pts.min(axis=0), all_pts.max(axis=0)
    mid = (lo + hi) / 2
    span = (hi - lo).max() / 2 * 1.2
    ax.set_xlim(mid[0] - span, mid[0] + span)
    ax.set_ylim(mid[1] - span, mid[1] + span)
    ax.set_zlim(mid[2] - span, mid[2] + span)
    plt.tight_layout()

    print(f"\nScene obstacles in arm {arm_number} frame ({len(blocks)} total):")
    for name, pos, size in blocks:
        print(f"  {name:30s}  pos={np.round(pos, 3)}  size={np.round(size, 3)}")


# ── Dual-arm plot (world frame) ────────────────────────────────────────────────

def plot_both_arms(joints1, joints2, add_waypts=False):
    """Plot both arms and shared scene obstacles, all in world frame."""
    arm_colors = {1: 'blue', 2: 'black'}
    arm_skel_colors = {1: 'navy', 2: 'black'}

    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection='3d')

    all_corner_pts = []

    for arm_number, joints in [(1, joints1), (2, joints2)]:
        arm = SimpleFranka.SimpleFrankArm(arm_number=arm_number)
        arm.CompCollisionBlockPoints(joints)

        W = _arm_to_world_H(arm_number)  # world_T_arm
        base_pos = W[:3, 3]

        # Arm skeleton in world frame
        Tcurr = arm.Tcurr
        skel_col = arm_skel_colors[arm_number]
        prev = base_pos
        for i in range(len(Tcurr)):
            # Joint position in arm frame → world frame
            p = W[:3, :3] @ Tcurr[i][:3, 3] + W[:3, 3]
            ax.scatter(*p, c=skel_col, s=20, zorder=5)
            ax.plot([prev[0], p[0]], [prev[1], p[1]], [prev[2], p[2]],
                    c=skel_col, lw=1.5)
            prev = p

        # Collision boxes transformed to world frame
        n_base = 12
        col = arm_colors[arm_number]
        for k, pts in enumerate(arm.Cpoints):
            world_pts = _transform_box_points(pts, W)
            color = 'green' if k >= n_base else col
            draw_box_solid_edges(ax, world_pts, color=color, lw=1.0)
            all_corner_pts.append(np.array(world_pts[1:]))

    # Scene obstacles — use arm 1's frame to get world-frame positions
    # obstacles are the same scene; transform from arm-1 frame to world
    W1 = _arm_to_world_H(1)
    blocks = get_obstacles_for_arm(1)
    for _, pos, size in blocks:
        H_arm = rt.rpyxyz2H([0., 0., 0.], pos)
        pts_arm, _ = rt.BlockDesc2Points(H_arm, size)
        # pts_arm are in arm-1 frame; bring to world frame
        world_pts = _transform_box_points(pts_arm, W1)
        draw_box_solid_edges(ax, world_pts, color='red', lw=1.0)
        all_corner_pts.append(np.array(world_pts[1:]))

    # Waypoints — arm 1 waypoints in arm 1 frame, arm 2 waypoints in arm 2 frame
    if add_waypts:
        W2 = _arm_to_world_H(2)
        draw_waypoints(ax, _load_waypoints(), arm_H=W1, skip=_ARM1_WAYPT_SKIP)
        draw_waypoints(ax, _load_waypoints(), arm_H=W2, only=_ARM2_WAYPT_ONLY)

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='blue',     lw=2, label='Arm 1 collision boxes'),
        Line2D([0], [0], color='black',    lw=2, label='Arm 2 collision boxes'),
        Line2D([0], [0], color='red',      lw=2, label='Scene obstacle boxes'),
        Line2D([0], [0], color='navy',     lw=2, label='Arm 1 skeleton'),
        Line2D([0], [0], color='black',    lw=2, label='Arm 2 skeleton'),
    ]
    if add_waypts:
        legend_elements.append(Line2D([0], [0], marker='*', color='black', lw=0, markersize=10, label='Waypoints'))
    ax.legend(handles=legend_elements, loc='upper left')

    ax.set_title("Both arms — world frame")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    if all_corner_pts:
        all_pts = np.concatenate(all_corner_pts, axis=0)
        lo, hi = all_pts.min(axis=0), all_pts.max(axis=0)
        mid = (lo + hi) / 2
        span = (hi - lo).max() / 2 * 1.2
        ax.set_xlim(mid[0] - span, mid[0] + span)
        ax.set_ylim(mid[1] - span, mid[1] + span)
        ax.set_zlim(mid[2] - span, mid[2] + span)

    plt.tight_layout()


# ── IK helper ─────────────────────────────────────────────────────────────────

def _fetch_safe_joints():
    """Return arm 2 joint angles for the FETCH_SAFE pose via IK."""
    import ikpy.chain
    _URDF = os.path.join(os.path.dirname(__file__), "../orio/panda_arm_hand.urdf")
    chain = ikpy.chain.Chain.from_urdf_file(_URDF, base_elements=["panda_link0"])
    chain.active_links_mask = [False] + [True]*7 + [False]*(len(chain.links) - 8)

    waypts = _load_waypoints()
    entry  = waypts["FETCH_SAFE"]
    target_pos = entry["translation"]
    target_rot = np.array(entry["rotation"])

    guess = [0.0] * len(chain.links)
    if len(chain.links) > 4:
        guess[4] = -1.5

    angles = chain.inverse_kinematics(
        target_position=target_pos,
        target_orientation=target_rot,
        orientation_mode="all",
        initial_position=guess,
    )
    return angles[1:8].tolist()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    HOME = [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]
    FETCH_SAFE_ARM2 = _fetch_safe_joints()

    parser = argparse.ArgumentParser(
        description="Plot arm collision boxes and scene obstacle boxes in 3D.")

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--arm", type=int, choices=[1, 2],
                      help="Single arm: 1 = left, 2 = right")
    mode.add_argument("--both", action="store_true",
                      help="Show both arms together in world frame")

    parser.add_argument("--joints", type=float, nargs=7, default=HOME,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
                        help="Joint angles for --arm mode (default: home position)")
    parser.add_argument("--joints1", type=float, nargs=7, default=HOME,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
                        help="Arm 1 joint angles for --both mode (default: home position)")
    parser.add_argument("--joints2", type=float, nargs=7, default=FETCH_SAFE_ARM2,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
                        help="Arm 2 joint angles for --both mode (default: FETCH_SAFE)")
    parser.add_argument("--add_waypts", action="store_true",
                        help="Overlay target task waypoints from Target_Task_Poses.json")

    args = parser.parse_args()

    if args.both:
        plot_both_arms(args.joints1, args.joints2, add_waypts=args.add_waypts)
    else:
        plot_single_arm(args.arm, args.joints, add_waypts=args.add_waypts)

    plt.show()


if __name__ == "__main__":
    main()
