"""
Parse orio_dual_scene.xml (and its included orio_mj_dual.xml) and return
obstacle blocks expressed in the chosen arm's base frame.

ARM 1 = left  arm (mj_left_link0)
ARM 2 = right arm (mj_right_link0)

Returns a list of  [name, pos_in_arm_frame, half_size]  entries compatible
with rt.BlockDesc2Points / Franka collision checking.
"""

import xml.etree.ElementTree as ET
import numpy as np
import os

# ── XML files ─────────────────────────────────────────────────────────────────
_SCENE_DIR  = os.path.join(os.path.dirname(__file__), "mujoco_files")
_SCENE_XML  = os.path.join(_SCENE_DIR, "orio_dual_scene.xml")
_DUAL_XML   = os.path.join(_SCENE_DIR, "orio_mj_dual.xml")

# World-frame positions and orientations of each arm's link0
# (taken from orio_mj_dual.xml)
ARM_BASE_WORLD = {
    1: {"pos": np.array([-0.538, 0.0, 0.9]), "quat": np.array([1, 0, 0, 0])},   # left
    2: {"pos": np.array([ 0.709, 0.0, 0.9]), "quat": np.array([0, 0, 0, 1])},   # right
}


# ── Quaternion helpers (MuJoCo convention: w x y z) ───────────────────────────

def _quat_to_rot(q):
    """Return 3×3 rotation matrix from quaternion [w, x, y, z]."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def _parse_vec(s, n=3):
    vals = [float(v) for v in s.split()]
    return np.array(vals[:n]) if len(vals) >= n else np.array(vals + [0.0]*(n-len(vals)))


# ── Walk a body tree, accumulating world transforms ───────────────────────────

def _collect_geoms(body_el, parent_pos, parent_rot, skip_names=None):
    """
    Recursively walk <body> elements, accumulate world-frame transforms,
    and collect box geoms.

    Returns list of (name, world_pos, half_size).
    """
    skip_names = skip_names or set()
    geoms = []

    # This body's local transform
    local_pos = _parse_vec(body_el.get("pos", "0 0 0"))
    quat_str  = body_el.get("quat", "1 0 0 0")
    local_rot = _quat_to_rot(_parse_vec(quat_str, 4))

    world_rot = parent_rot @ local_rot
    world_pos = parent_pos + parent_rot @ local_pos

    for geom in body_el.findall("geom"):
        if geom.get("type", "sphere") != "box":
            continue
        name = geom.get("name", "unnamed")
        if name in skip_names:
            continue
        g_pos_local = _parse_vec(geom.get("pos", "0 0 0"))
        g_pos_world = world_pos + world_rot @ g_pos_local
        half_size   = _parse_vec(geom.get("size", "0 0 0"))
        geoms.append((name, g_pos_world, half_size))

    for child in body_el.findall("body"):
        geoms.extend(_collect_geoms(child, world_pos, world_rot, skip_names))

    return geoms


def _parse_scene_geoms():
    """
    Parse both XML files and return all box geoms with their world positions.
    Skips the floor plane (it's a plane, not a box) and any commented-out geoms.
    """
    geoms = []
    I3 = np.eye(3)
    origin = np.zeros(3)

    # --- orio_dual_scene.xml worldbody bodies (tables, frames) ---
    scene_tree = ET.parse(_SCENE_XML)
    scene_wb   = scene_tree.getroot().find("worldbody")
    for body in scene_wb.findall("body"):
        geoms.extend(_collect_geoms(body, origin, I3))

    # --- orio_mj_dual.xml worldbody bodies (arm links) ---
    # We skip the robot link geoms — only the scene furniture matters
    # (arm self-collision is handled by the FK collision checker separately)

    return geoms


# ── Public API ────────────────────────────────────────────────────────────────

def get_obstacles_for_arm(arm_number):
    """
    Return obstacle blocks in the chosen arm's base frame.

    Parameters
    ----------
    arm_number : int
        1 = left arm (mj_left_link0)
        2 = right arm (mj_right_link0)

    Returns
    -------
    list of [name, pos_in_arm_frame, half_size]
        Compatible with rt.BlockDesc2Points.
    """
    if arm_number not in ARM_BASE_WORLD:
        raise ValueError(f"arm_number must be 1 or 2, got {arm_number}")

    base      = ARM_BASE_WORLD[arm_number]
    base_pos  = base["pos"]
    base_rot  = _quat_to_rot(base["quat"])   # world ← arm
    inv_rot   = base_rot.T                    # arm  ← world  (rotation is orthogonal)

    world_geoms = _parse_scene_geoms()

    blocks = []
    for name, world_pos, half_size in world_geoms:
        arm_pos = inv_rot @ (world_pos - base_pos)
        blocks.append([name, arm_pos.tolist(), (half_size * 2).tolist()])

    return blocks


if __name__ == "__main__":
    for arm in (1, 2):
        print(f"\n── Arm {arm} obstacles ──────────────────────")
        for name, pos, size in get_obstacles_for_arm(arm):
            print(f"  {name:25s}  pos={np.round(pos,3)}  half_size={np.round(size,3)}")
