"""Reading and measuring the real Xtion captures in ``misc/sim-calib-imgs/``. Plain numpy.

A capture folder (packed by ``scripts/pack_xtion_capture.py``) holds ``color.png``,
``depth_mm.png`` and ``K.json``. The captures are stored the way the sensor delivered them,
which is rotated 180 degrees from the reference photos in ``misc/item-images/`` that the item
corners were clicked in; ``reference_frame`` turns them. A 180 degree turn about the image
centre keeps the pinhole model unchanged, so ``K`` applies to both orientations.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def load_capture(folder: str | Path) -> dict:
    """``color`` uint8 (H, W, 3), ``depth`` float32 metres with NaN where invalid, ``K`` (3, 3)."""
    folder = Path(folder)
    color = np.asarray(Image.open(folder / "color.png").convert("RGB"))
    depth_mm = np.asarray(Image.open(folder / "depth_mm.png")).astype(np.float32)
    depth = np.where(depth_mm > 0, depth_mm / 1000.0, np.nan).astype(np.float32)
    K = np.asarray(json.loads((folder / "K.json").read_text())["K"], dtype=np.float64)
    return {"color": color, "depth": depth, "K": K}


def reference_frame(img: np.ndarray) -> np.ndarray:
    """A capture image in the orientation of the reference photos (rotated 180 degrees)."""
    return np.ascontiguousarray(np.rot90(img, 2))


def backproject(depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """(H, W) z-depth to (H, W, 3) camera-frame points (OpenCV axes). NaN where depth is."""
    h, w = depth.shape
    yy, xx = np.mgrid[0:h, 0:w]
    x = (xx - K[0, 2]) / K[0, 0] * depth
    y = (yy - K[1, 2]) / K[1, 1] * depth
    return np.stack([x, y, depth], axis=-1)


@dataclass
class Plane:
    normal: np.ndarray
    """Unit normal pointing toward the camera (negative z component)."""
    point: np.ndarray
    """A point on the plane, camera frame."""

    @property
    def distance(self) -> float:
        """Distance of the camera origin from the plane."""
        return float(abs(self.point @ self.normal))

    def height_above(self, points: np.ndarray) -> np.ndarray:
        """Signed height of ``points`` (..., 3) above the plane, positive toward the camera."""
        return (points - self.point) @ self.normal

    def intersect_pixels(self, uv: np.ndarray, K: np.ndarray) -> np.ndarray:
        """Camera-frame points where the rays through pixels (N, 2) meet the plane."""
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        rays = np.linalg.solve(K, np.column_stack([uv, np.ones(len(uv))]).T).T
        s = (self.point @ self.normal) / (rays @ self.normal)
        return rays * s[:, None]


def fit_plane(points: np.ndarray, inlier_dist: float = 0.01, passes: int = 3) -> Plane:
    """Least-squares plane through (N, 3) points, refitted on inliers to shed outliers."""
    P = np.asarray(points, dtype=np.float64)
    keep = np.ones(len(P), dtype=bool)
    for _ in range(passes):
        c = P[keep].mean(axis=0)
        _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
        n = vt[-1]
        if n[2] > 0:
            n = -n
        keep = np.abs((P - c) @ n) < inlier_dist
    return Plane(normal=n, point=c)


def quad_mask(shape: tuple[int, int], quad: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """Boolean mask of a pixel quad, scaled about its centroid (``scale`` > 1 grows it)."""
    quad = np.asarray(quad, dtype=np.float64)
    c = quad.mean(axis=0)
    quad = c + (quad - c) * scale
    m = Image.new("L", (shape[1], shape[0]), 0)
    ImageDraw.Draw(m).polygon([tuple(p) for p in quad], fill=255)
    return np.asarray(m) > 0


def table_plane(depth: np.ndarray, K: np.ndarray, exclude: np.ndarray | None = None, max_depth: float = 1.2) -> Plane:
    """Plane of the table top: everything closer than ``max_depth`` that is not excluded (items,
    and anything outside the table edges), with the robust refit removing what is left."""
    P = backproject(depth, K)
    sel = np.isfinite(depth) & (depth < max_depth)
    if exclude is not None:
        sel &= ~exclude
    return fit_plane(P[sel], inlier_dist=0.01)


# --- table edges in the image -----------------------------------------------------------------
#
# In the captures the near (-y) table edge runs along the top of the image and drops to the
# floor; the -x edge (image right) is a 1 to 2 cm step down to the neighbouring table; the +x
# edge (image left) has a keyboard and clutter against it and is only visible as the green line
# hand-drawn in the reference photo.


@dataclass
class Line2D:
    centre: np.ndarray
    direction: np.ndarray
    points: np.ndarray
    """Inlier pixels (N, 2)."""

    def x_at(self, y):
        return self.centre[0] + (y - self.centre[1]) / self.direction[1] * self.direction[0]

    def y_at(self, x):
        return self.centre[1] + (x - self.centre[0]) / self.direction[0] * self.direction[1]

    @property
    def angle_deg(self) -> float:
        return float(np.degrees(np.arctan2(self.direction[1], self.direction[0])) % 180)


def ransac_line(pts: np.ndarray, thresh: float, iters: int = 1000, seed: int = 0) -> Line2D:
    """Line through 2D points with the most inliers within ``thresh``, refitted to them."""
    pts = np.asarray(pts, dtype=np.float64)
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i, j = rng.choice(len(pts), 2, replace=False)
        d = pts[j] - pts[i]
        if np.linalg.norm(d) < 1e-9:
            continue
        d /= np.linalg.norm(d)
        inl = np.abs((pts - pts[i]) @ np.array([-d[1], d[0]])) < thresh
        if best is None or inl.sum() > best.sum():
            best = inl
    c = pts[best].mean(axis=0)
    _, _, vt = np.linalg.svd(pts[best] - c, full_matrices=False)
    return Line2D(c, vt[0], pts[best])


def near_edge_pixels(depth: np.ndarray) -> np.ndarray:
    """(N, 2) pixels on the table-to-floor drop along the top of the image, one per column."""
    pts = []
    rows = np.arange(depth.shape[0])
    for x in range(20, depth.shape[1] - 20):
        col = depth[:, x]
        ok = np.isfinite(col)
        far = np.flatnonzero(ok & (col > 1.5) & (rows < 200))
        if not len(far):
            continue
        y_far = far.max()
        ref = np.nanmedian(depth[150:450, x])
        tab = np.flatnonzero(ok & (np.abs(col - ref) < 0.03) & (rows > y_far))
        if len(tab) and tab.min() - y_far < 12:
            pts.append((x, (y_far + tab.min()) / 2.0))
    return np.array(pts, dtype=np.float64)


def side_step_pixels(depth: np.ndarray, x0: int, x1: int, y0: int = 110, y1: int = 470) -> np.ndarray:
    """(N, 2) pixels where the depth steps by 1.2 to 4 cm across 6 px, one per row, within
    columns ``x0:x1``."""
    pts = []
    for y in range(y0, y1):
        d = depth[y, x0:x1]
        dd = d[6:] - d[:-6]
        k = np.flatnonzero(np.isfinite(dd) & (np.abs(dd) > 0.012) & (np.abs(dd) < 0.04))
        if len(k):
            pts.append((x0 + k[np.argmax(np.abs(dd[k]))] + 3, y))
    return np.array(pts, dtype=np.float64)


def green_pixels(img: np.ndarray) -> np.ndarray:
    """(N, 2) pixels of the green hand-drawn lines in a reference photo."""
    r, g, b = (img[..., i].astype(int) for i in range(3))
    ys, xs = np.nonzero((g > 140) & (g > r + 60) & (g > b + 60))
    return np.stack([xs, ys], axis=1).astype(np.float64)


@dataclass
class TableEdges:
    near: Line2D
    """-y edge, from the depth drop to the floor."""
    right: Line2D
    """-x edge (image right), from the depth step to the neighbouring table."""
    left: Line2D
    """+x edge (image left), from the hand-drawn green line."""

    def table_mask(self, shape: tuple[int, int], margin: int = 10) -> np.ndarray:
        """Pixels inside the three edges by at least ``margin`` px (the bottom is the image edge)."""
        yy, xx = np.mgrid[0 : shape[0], 0 : shape[1]]
        return (yy > self.near.y_at(xx) + margin) & (xx < self.right.x_at(yy) - margin) & (xx > self.left.x_at(yy) + margin)


def detect_table_edges(depth: np.ndarray, reference_rgb: np.ndarray) -> TableEdges:
    """The three visible table edges of a capture in the reference orientation."""
    g = green_pixels(reference_rgb)
    return TableEdges(
        near=ransac_line(near_edge_pixels(depth), 1.0),
        right=ransac_line(side_step_pixels(depth, 480, 600), 2.0),
        left=ransac_line(g[(g[:, 1] > 160) & (g[:, 0] < 300)], 1.5),
    )
