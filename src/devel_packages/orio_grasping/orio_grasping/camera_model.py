"""Pinhole model of the simulated camera, without Isaac Sim, for offline checks.

Reproduces the intrinsics and extrinsics that ``scene.make_camera_cfg`` gives the simulator so
the same projection can be evaluated in plain Python (see ``scripts/compare_table_outline.py``).
"""

from __future__ import annotations

import numpy as np

from .config import OrioCfg
from .geometry import look_at_quat_world_convention, matrix_from_quat

HORIZONTAL_APERTURE_MM = 20.955


def intrinsics(cfg: OrioCfg) -> np.ndarray:
    """3x3 camera matrix matching Isaac Sim's pinhole camera for the configured FOV."""
    w, h = cfg.camera.width, cfg.camera.height
    fx = cfg.camera.focal_length_mm / HORIZONTAL_APERTURE_MM * w
    return np.array([[fx, 0.0, w / 2], [0.0, fx, h / 2], [0.0, 0.0, 1.0]])


def extrinsics(cfg: OrioCfg) -> np.ndarray:
    """4x4 ``T_world_cam`` for the OpenCV camera frame (x right, y down, z forward)."""
    q = look_at_quat_world_convention(cfg.camera.position, cfg.camera_look_at(), cfg.camera.up_hint)
    R_wc = matrix_from_quat(q)  # columns: forward, left, up
    forward, left, up = R_wc[:, 0], R_wc[:, 1], R_wc[:, 2]
    R_cv = np.stack([-left, -up, forward], axis=1)
    T = np.eye(4)
    T[:3, :3] = R_cv
    T[:3, 3] = cfg.camera.position
    return T


def project(points_w: np.ndarray, K: np.ndarray, T_world_cam: np.ndarray) -> np.ndarray:
    """World points (N, 3) to pixel coordinates (N, 2)."""
    R, t = T_world_cam[:3, :3], T_world_cam[:3, 3]
    p = (np.asarray(points_w, dtype=np.float64) - t) @ R
    uv = (K @ p.T).T
    return uv[:, :2] / uv[:, 2:3]


def table_outline_world(cfg: OrioCfg) -> np.ndarray:
    """The four table-top corners, counter-clockwise from (x_min, y_min)."""
    t = cfg.table
    return np.array(
        [
            [t.x_min, t.y_min, t.top_z],
            [t.x_max, t.y_min, t.top_z],
            [t.x_max, t.y_max, t.top_z],
            [t.x_min, t.y_max, t.top_z],
        ]
    )


def pixel_to_plane(uv: np.ndarray, K: np.ndarray, T_world_cam: np.ndarray, z: float) -> np.ndarray:
    """Intersect the rays through pixels (N, 2) with the horizontal plane at height ``z``."""
    uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
    rays_cam = np.linalg.solve(K, np.column_stack([uv, np.ones(len(uv))]).T).T
    R, t = T_world_cam[:3, :3], T_world_cam[:3, 3]
    rays_w = rays_cam @ R.T
    s = (z - t[2]) / rays_w[:, 2]
    return t + rays_w * s[:, None]


def table_crop(cfg: OrioCfg, shape: tuple[int, int]) -> tuple[tuple[int, int, int, int], np.ndarray]:
    """Pixel box ``(x0, y0, x1, y1)`` around the table top in the simulated camera, clipped to
    the image, and a boolean mask of the table-top polygon inside the full image."""
    h, w = shape
    corners = project(table_outline_world(cfg), intrinsics(cfg), extrinsics(cfg))
    x0 = int(max(0, np.floor(corners[:, 0].min())))
    y0 = int(max(0, np.floor(corners[:, 1].min())))
    x1 = int(min(w, np.ceil(corners[:, 0].max())))
    y1 = int(min(h, np.ceil(corners[:, 1].max())))
    yy, xx = np.mgrid[0:h, 0:w]
    mask = np.ones((h, w), dtype=bool)
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        # keep the side of each edge that contains the polygon centre
        side = (b[0] - a[0]) * (yy - a[1]) - (b[1] - a[1]) * (xx - a[0])
        c = corners.mean(axis=0)
        ref = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        mask &= (side * ref) >= 0
    return (x0, y0, x1, y1), mask


def largest_rectangle(mask: np.ndarray) -> tuple[int, int, int, int]:
    """Largest axis-aligned rectangle ``(x0, y0, x1, y1)`` (exclusive ends) of True pixels."""
    h, w = mask.shape
    heights = np.zeros(w, dtype=int)
    best, box = 0, (0, 0, 0, 0)
    for y in range(h):
        heights = np.where(mask[y], heights + 1, 0)
        stack: list[int] = []  # columns with increasing heights (largest rectangle in a histogram)
        for x in range(w + 1):
            cur = heights[x] if x < w else 0
            while stack and heights[stack[-1]] >= cur:
                top = heights[stack.pop()]
                left = stack[-1] + 1 if stack else 0
                if top * (x - left) > best:
                    best, box = int(top * (x - left)), (int(left), int(y + 1 - top), int(x), int(y + 1))
            stack.append(x)
    return box


def table_inner_crop(cfg: OrioCfg, shape: tuple[int, int], inset_px: int = 0) -> tuple[int, int, int, int]:
    """Largest pixel box ``(x0, y0, x1, y1)`` that shows nothing but table top: the biggest
    axis-aligned rectangle inside the table polygon (clipped to the image), with every side
    moved ``inset_px`` further inside the polygon to absorb calibration error."""
    _, mask = table_crop(cfg, shape)
    for _ in range(inset_px):
        inner = mask.copy()
        inner[1:] &= mask[:-1]
        inner[:-1] &= mask[1:]
        inner[:, 1:] &= mask[:, :-1]
        inner[:, :-1] &= mask[:, 1:]
        mask = inner
    return largest_rectangle(mask)
