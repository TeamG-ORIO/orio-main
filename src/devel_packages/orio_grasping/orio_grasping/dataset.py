"""Reading and writing the raw scene format. Plain numpy and Pillow, no Isaac Sim.

This module is the reference implementation of ``docs/DATA_FORMAT.md``. A labelling or
training pipeline can import it or copy it; it has no dependency on the rest of the package
except :mod:`orio_grasping.geometry`.

::

    from orio_grasping.dataset import load_scene, list_scenes
    for folder in list_scenes("data/2026-10-06_143052_catalog_v0"):
        s = load_scene(folder)             # rgb, depth (m), seg, meta, points (camera frame)
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image

from .geometry import matrix_from_quat, rpy_from_matrix

FORMAT_VERSION = 1
DEPTH_SCALE = 10000
"""``depth.png`` stores ``round(metres * DEPTH_SCALE)`` as uint16: 0.1 mm steps, 0 = invalid,
up to 6.55 m."""
SCENE_DIR_RE = "scene_{index:06d}"
FACE_NAMES = {(2, +1): "top", (2, -1): "bottom", (0, +1): "side_x+", (0, -1): "side_x-", (1, +1): "side_y+", (1, -1): "side_y-"}

CONVENTIONS = {
    "world_frame": "workcell frame: +z up, table top at z = workcell.table_top_z, arm base centre at x = robot.base_position[0], y = 0, "
    "overhead camera on the -y side; metres",
    "camera_frame": "OpenCV optical convention: x right, y down, z forward (along the optical axis); camera.T_world_cam maps "
    "camera-frame points into the world frame; camera.K projects camera-frame points to pixels (u right, v down)",
    "object_frame": "origin at the box centre; x along dims[0], y along dims[1], z along dims[2]; the +z face is the printed top "
    "face (catalog items), the front (pouches) or the face the texture's top is projected on (random boxes); for pouches and "
    "imported objects dims are the bounding box of the mesh, the origin is that box's centre and an imported object's "
    "+z is up as in its source model",
    "quaternion": "w, x, y, z",
    "rpy": "extrinsic X-Y-Z (roll about world x, then pitch about world y, then yaw about world z), radians: R = Rz(yaw) Ry(pitch) Rx(roll)",
    "depth": "metres along the optical axis (z-depth, not ray length); depth.png value / camera.depth_scale; 0 = invalid",
    "seg": "uint16 per pixel: 0 = not an object, k = objects[k-1].id",
    "bbox_xyxy": "pixel bounds of the object's visible mask, inclusive min and max of u and v",
    "crop": "when camera.crop is not null the images are the window camera.crop.box_xyxy_full of the sensor frame "
    "(x0, y0 inclusive, x1, y1 exclusive); camera.K, bbox_xyxy and visible_pixels refer to the cropped image, "
    "camera.K_full to the full frame; full-frame pixel = cropped pixel + (x0, y0)",
}


# --- encoding ------------------------------------------------------------------------------------


def encode_depth(depth_m: np.ndarray) -> np.ndarray:
    """float metres (0 or NaN = invalid) to the uint16 stored in ``depth.png``."""
    d = np.asarray(depth_m, dtype=np.float64)
    valid = np.isfinite(d) & (d > 0)
    out = np.where(valid, np.round(d * DEPTH_SCALE), 0)
    return np.clip(out, 0, 65535).astype(np.uint16)


def decode_depth(png: np.ndarray, scale: float = DEPTH_SCALE) -> np.ndarray:
    """``depth.png`` array to float32 metres, 0 where invalid."""
    return (np.asarray(png).astype(np.float32) / scale).astype(np.float32)


def point_cloud(depth_m: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Organised (H, W, 3) float32 point cloud in the camera frame; NaN where depth is invalid."""
    h, w = depth_m.shape
    v, u = np.mgrid[0:h, 0:w].astype(np.float32)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = np.asarray(depth_m, dtype=np.float32)
    pts = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], axis=-1)
    pts[d <= 0] = np.nan
    return pts


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to (..., 3) points."""
    return points @ np.asarray(T)[:3, :3].T + np.asarray(T)[:3, 3]


# --- per-object geometry ------------------------------------------------------------------------


def object_geometry(pos, quat_wxyz, dims, T_world_cam: np.ndarray, seg: np.ndarray | None = None, seg_id: int | None = None) -> dict:
    """The derived per-object fields of ``meta.json`` from a world-frame pose."""
    R = matrix_from_quat(quat_wxyz)
    T_world_obj = np.eye(4)
    T_world_obj[:3, :3] = R
    T_world_obj[:3, 3] = pos
    T_cam_world = np.linalg.inv(np.asarray(T_world_cam, dtype=np.float64))
    T_cam_obj = T_cam_world @ T_world_obj

    # which object axis points up in the world
    up = R.T @ np.array([0.0, 0.0, 1.0])  # world up expressed in the object frame
    axis = int(np.argmax(np.abs(up)))
    sign = 1 if up[axis] > 0 else -1
    top_normal_w = R[:, 2]
    top_center_w = np.asarray(pos, dtype=np.float64) + top_normal_w * (dims[2] / 2.0)
    out = {
        "pose_world": {
            "pos": [float(v) for v in pos],
            "quat_wxyz": [float(v) for v in quat_wxyz],
            "rpy_xyz_rad": [float(v) for v in rpy_from_matrix(R)],
        },
        "T_world_obj": T_world_obj.tolist(),
        "T_cam_obj": T_cam_obj.tolist(),
        "face_up": FACE_NAMES[(axis, sign)],
        "tilt_deg": float(np.degrees(np.arccos(np.clip(abs(up[axis]), -1.0, 1.0)))),
        "top_face": {
            "center_world": top_center_w.tolist(),
            "normal_world": top_normal_w.tolist(),
            "center_cam": transform_points(top_center_w, T_cam_world).tolist(),
            "normal_cam": (T_cam_world[:3, :3] @ top_normal_w).tolist(),
        },
    }
    if seg is not None and seg_id is not None:
        vs, us = np.nonzero(seg == seg_id)
        out["visible_pixels"] = int(len(us))
        out["bbox_xyxy"] = [int(us.min()), int(vs.min()), int(us.max()), int(vs.max())] if len(us) else None
    return out


# --- reading -------------------------------------------------------------------------------------


def list_scenes(run_dir: str | Path) -> list[Path]:
    return sorted(p for p in Path(run_dir).glob("scene_*") if p.is_dir())


def load_manifest(run_dir: str | Path) -> dict:
    return json.loads((Path(run_dir) / "dataset.json").read_text())


def load_scene(folder: str | Path, with_points: bool = False) -> dict:
    """One scene: ``rgb`` uint8 (H, W, 3), ``depth`` float32 metres (0 = invalid; the sensor-
    model depth when one was applied), ``depth_clean`` (the exact simulated depth; same array
    as ``depth`` when no sensor model was applied), ``seg`` uint16 (H, W), ``meta`` dict, and
    ``points`` (H, W, 3) from ``depth`` in the camera frame if requested (NaN where invalid).
    Missing files come back as ``None``."""
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text())
    version = meta.get("format_version", 0)
    if version != FORMAT_VERSION:
        raise ValueError(f"{folder}: format_version {version}, this reader is for {FORMAT_VERSION}")

    def png(name):
        p = folder / name
        return np.asarray(Image.open(p)) if p.is_file() else None

    rgb = png("rgb.png")
    scale = meta["camera"]["depth_scale"]
    depth_png = png("depth.png")
    depth = decode_depth(depth_png, scale) if depth_png is not None else None
    clean_png = png("depth_clean.png")
    depth_clean = decode_depth(clean_png, scale) if clean_png is not None else depth
    seg = png("seg.png")
    out = {"rgb": rgb, "depth": depth, "depth_clean": depth_clean, "seg": seg, "meta": meta, "points": None}
    if with_points and depth is not None:
        out["points"] = point_cloud(depth, np.asarray(meta["camera"]["K"]))
    return out


# --- preview -------------------------------------------------------------------------------------


def preview_image(rgb: np.ndarray, depth_m: np.ndarray, depth_range: tuple[float, float] | None = None) -> np.ndarray:
    """RGB and colourised depth side by side, for eyeballing a scene folder.

    The colour scale spans the pile and the table top (the closest 30 cm of valid depth);
    anything farther, such as the floor, saturates."""
    valid = depth_m > 0
    if depth_range is None:
        if valid.any():
            lo = float(np.percentile(depth_m[valid], 0.5))
            near = depth_m[valid & (depth_m < lo + 0.3)]
            hi = float(np.percentile(near, 99.5)) if near.size else lo + 0.3
        else:
            lo, hi = 0.0, 1.0
        hi = max(hi, lo + 1e-3)
    else:
        lo, hi = depth_range
    t = np.clip((depth_m - lo) / (hi - lo), 0.0, 1.0)
    colour = _viridis(t)
    colour[~valid] = 0
    return np.concatenate([rgb[..., :3], colour], axis=1)


def _viridis(t: np.ndarray) -> np.ndarray:
    """Viridis without matplotlib: a 9-stop piecewise-linear approximation, uint8 RGB."""
    stops = np.array(
        [[68, 1, 84], [72, 36, 117], [65, 68, 135], [53, 95, 141], [42, 120, 142], [33, 145, 140], [34, 168, 132], [68, 191, 112], [122, 209, 81], [189, 223, 38], [253, 231, 37]],
        dtype=np.float64,
    )
    x = np.clip(t, 0, 1) * (len(stops) - 1)
    i = np.clip(np.floor(x).astype(int), 0, len(stops) - 2)
    f = (x - i)[..., None]
    return (stops[i] * (1 - f) + stops[i + 1] * f).astype(np.uint8)
