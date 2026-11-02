"""Top-down height map and surface normals from one depth image.

Plain numpy and scipy, no Isaac dependency, so the simulator's labeller, the training data
builder and the robot all run this same code on sim-noisy, sim-clean or real Xtion depth.

The grid lies in the table plane. Column ``j`` runs along world -x and row ``i`` along world +y
(the camera image's right and down), so a map reads like the camera image::

    x = x_max - (j + 0.5) * cell        y = y_min + (i + 0.5) * cell

Every output channel is in metres or the world frame, never normalised per image.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .config import HeightMapCfg
from .dataset import point_cloud, transform_points

# 8-neighbourhood offsets (drow, dcol)
_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass(frozen=True)
class Grid:
    """Placement of the map's cells in the world frame."""

    x_range: tuple[float, float]
    y_range: tuple[float, float]
    cell: float

    @property
    def shape(self) -> tuple[int, int]:
        """(rows, cols)."""
        return (
            int(round((self.y_range[1] - self.y_range[0]) / self.cell)),
            int(round((self.x_range[1] - self.x_range[0]) / self.cell)),
        )

    def world_to_cell(self, xy: np.ndarray) -> np.ndarray:
        """(..., 2) world x, y to (..., 2) fractional (row, col); integer values are cell centres."""
        xy = np.asarray(xy, dtype=np.float64)
        col = (self.x_range[1] - xy[..., 0]) / self.cell - 0.5
        row = (xy[..., 1] - self.y_range[0]) / self.cell - 0.5
        return np.stack([row, col], axis=-1)

    def cell_to_world(self, rc: np.ndarray) -> np.ndarray:
        """(..., 2) (row, col) to (..., 2) world x, y of the cell centres."""
        rc = np.asarray(rc, dtype=np.float64)
        x = self.x_range[1] - (rc[..., 1] + 0.5) * self.cell
        y = self.y_range[0] + (rc[..., 0] + 0.5) * self.cell
        return np.stack([x, y], axis=-1)

    def T_world_map(self, table_z: float) -> np.ndarray:
        """4x4 taking (col, row, height) map coordinates to world points."""
        T = np.eye(4)
        T[:3, :3] = [[-self.cell, 0, 0], [0, self.cell, 0], [0, 0, 1]]
        T[:3, 3] = [self.x_range[1] - 0.5 * self.cell, self.y_range[0] + 0.5 * self.cell, table_z]
        return T

    def to_dict(self) -> dict:
        return {
            "x_range": list(self.x_range),
            "y_range": list(self.y_range),
            "cell": self.cell,
            "shape": list(self.shape),
            "axes": "col along world -x, row along world +y",
        }


def grid_for_scene(meta: dict, cfg: HeightMapCfg) -> Grid:
    """The map grid for a scene: the configured extent, or the scene's bin region."""
    b = meta["workcell"]["bin_inner"]
    return Grid(tuple(cfg.x_range or b["x"]), tuple(cfg.y_range or b["y"]), cfg.cell_size)


def bin_max_height(points_w: np.ndarray, grid: Grid, table_z: float) -> np.ndarray:
    """Highest point above the table per cell; NaN where no point landed."""
    rows, cols = grid.shape
    rc = np.floor(grid.world_to_cell(points_w[:, :2]) + 0.5).astype(np.int64)
    inside = (rc[:, 0] >= 0) & (rc[:, 0] < rows) & (rc[:, 1] >= 0) & (rc[:, 1] < cols)
    flat = rc[inside, 0] * cols + rc[inside, 1]
    h = np.full(rows * cols, -np.inf)
    np.maximum.at(h, flat, points_w[inside, 2] - table_z)
    h[np.isinf(h)] = np.nan
    return h.reshape(rows, cols)


def _neighbour_stack(a: np.ndarray, fill: float) -> np.ndarray:
    """(8, H, W): each cell's 8 neighbours, ``fill`` beyond the border."""
    p = np.pad(a, 1, constant_values=fill)
    h, w = a.shape
    return np.stack([p[1 + dr : 1 + dr + h, 1 + dc : 1 + dc + w] for dr, dc in _NEIGHBOURS])


def close_gaps(h: np.ndarray, min_neighbours: int) -> np.ndarray:
    """Fill empty cells surrounded by measured ones with their neighbours' median (one pass)."""
    nb = _neighbour_stack(h, np.nan)
    count = np.isfinite(nb).sum(axis=0)
    fill = np.isnan(h) & (count >= min_neighbours)
    out = h.copy()
    if fill.any():
        out[fill] = np.nanmedian(nb[:, fill], axis=0)
    return out


def fill_unseen(h: np.ndarray) -> np.ndarray:
    """Grow measured heights into empty cells, always taking the lowest measured neighbour, so a
    filled cell never looks like a higher surface than what bounds it (repeated 3x3 erosion)."""
    if np.isnan(h).all():
        return np.zeros_like(h)
    out = np.where(np.isnan(h), np.inf, h).astype(np.float32)
    kernel = np.ones((3, 3), np.uint8)
    missing = np.isinf(out)
    while missing.any():
        low = cv2.erode(out, kernel, borderType=cv2.BORDER_REPLICATE)
        out[missing] = low[missing]
        missing = np.isinf(out)
    return out.astype(h.dtype)


def disc_kernels(radius: float, cell: float) -> tuple[np.ndarray, np.ndarray, float]:
    """Correlation kernels whose responses on a height map are the least-squares slopes
    dh/dx and dh/dy over a disc (the moments of a full disc are symmetric, so the plane fit
    reduces to two weighted sums). Returns (kx, ky, sum of w * dx^2)."""
    r = max(1, int(round(radius / cell)))
    di, dj = np.mgrid[-r : r + 1, -r : r + 1].astype(np.float64)
    w = (di**2 + dj**2 <= r**2 + 1e-9).astype(np.float64)
    dx = -dj * cell  # columns run along -x
    dy = di * cell
    s2 = float((w * dx**2).sum())
    return w * dx, w * dy, s2


def surface_normals(h: np.ndarray, cell: float, radius: float) -> np.ndarray:
    """(3, H, W) unit normals in the world frame from a plane fit over a disc at every cell."""
    kx, ky, s2 = disc_kernels(radius, cell)
    h = np.asarray(h, dtype=np.float32)
    gx = cv2.filter2D(h, cv2.CV_32F, (kx / s2).astype(np.float32), borderType=cv2.BORDER_REPLICATE)
    gy = cv2.filter2D(h, cv2.CV_32F, (ky / s2).astype(np.float32), borderType=cv2.BORDER_REPLICATE)
    n = np.stack([-gx, -gy, np.ones_like(h)])
    return n / np.sqrt((n * n).sum(axis=0, keepdims=True))


def bin_top_id(points_w: np.ndarray, ids: np.ndarray, grid: Grid) -> np.ndarray:
    """(H, W) int32 id of the highest point per cell (``ids`` per point, e.g. a predicted
    instance); -1 where no point landed."""
    rows, cols = grid.shape
    rc = np.floor(grid.world_to_cell(points_w[:, :2]) + 0.5).astype(np.int64)
    inside = (rc[:, 0] >= 0) & (rc[:, 0] < rows) & (rc[:, 1] >= 0) & (rc[:, 1] < cols)
    flat = rc[inside, 0] * cols + rc[inside, 1]
    z, idv = points_w[inside, 2], np.asarray(ids)[inside]
    order = np.lexsort((z, flat))  # by cell, then height: the last of each cell is its highest point
    flat, idv = flat[order], idv[order]
    last = np.r_[flat[1:] != flat[:-1], True]
    out = np.full(rows * cols, -1, np.int32)
    out[flat[last]] = idv[last]
    return out.reshape(rows, cols)


def project_instances(instances: np.ndarray, valid: np.ndarray, pts_w: np.ndarray, grid: Grid) -> np.ndarray:
    """(H, W) int16 instance id per cell (0 = no object) from a per-pixel instance image: the id of
    the highest depth point in the cell, and for cells no point reached the id of the nearest
    cell that one did."""
    from scipy import ndimage

    seg = bin_top_id(pts_w, np.asarray(instances)[valid], grid)
    hole = seg < 0
    if hole.all():
        return np.zeros(seg.shape, np.int16)
    if hole.any():
        idx = ndimage.distance_transform_edt(hole, return_distances=False, return_indices=True)
        seg = seg[idx[0], idx[1]]
    return seg.astype(np.int16)


SEG_DIST_MAX = 0.05
"""Metres: the ``seg`` channels' distance to the object's edge is clipped here and scaled to 1."""


def seg_channels(seg: np.ndarray, cell: float) -> np.ndarray:
    """(3, H, W) float32 network channels from a cell instance map (0 = no object): on a predicted
    object (0 / 1), at the boundary between two different instances or an instance and the table
    (0 / 1, either side), and distance from the cell to the nearest cell outside its own
    instance in metres / ``SEG_DIST_MAX``, clipped to 1 (0 off objects)."""
    from scipy import ndimage

    seg = np.asarray(seg)
    on = seg > 0
    edge = np.zeros(seg.shape, bool)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = seg[max(dy, 0):, max(dx, 0):seg.shape[1] + min(dx, 0)]
        b = seg[: seg.shape[0] - dy, max(-dx, 0):seg.shape[1] - max(dx, 0)]
        diff = (a != b) & ((a > 0) | (b > 0))
        edge[max(dy, 0):, max(dx, 0):seg.shape[1] + min(dx, 0)] |= diff
        edge[: seg.shape[0] - dy, max(-dx, 0):seg.shape[1] - max(dx, 0)] |= diff
    dist = np.zeros(seg.shape, np.float32)
    if on.any():
        # inside each instance, the distance to the nearest cell that is not that instance
        inner = ndimage.distance_transform_edt(~edge & on) * cell
        dist = np.where(on, np.minimum(inner / SEG_DIST_MAX, 1.0), 0.0).astype(np.float32)
    return np.stack([on, edge, dist]).astype(np.float32)


def build_heightmap(depth_m: np.ndarray, K: np.ndarray, T_world_cam: np.ndarray, table_z: float, grid: Grid, cfg: HeightMapCfg,
                    instances: np.ndarray | None = None) -> dict:
    """One depth image (metres, 0 = invalid) to the height map channels.

    Returns ``height`` (H, W) metres above the table with every cell filled, ``normals``
    (3, H, W) world frame with nz > 0, ``unseen`` (H, W) bool for cells no depth reached
    (occlusions, projector shadows, dropouts; their height is filled from the lowest neighbour),
    and ``grid``. With ``instances`` (an (h, w) per-pixel instance image registered to the depth,
    0 = no object, e.g. a segmentation model's masks) also ``seg`` (H, W) int16 from
    :func:`project_instances`.
    """
    pts = point_cloud(depth_m, K)
    valid = np.isfinite(pts[..., 2])
    pts_w = transform_points(pts[valid].astype(np.float64), np.asarray(T_world_cam, dtype=np.float64))
    h = bin_max_height(pts_w, grid, table_z)
    h = close_gaps(h, cfg.gap_min_neighbours)
    unseen = np.isnan(h)
    h = np.clip(fill_unseen(h), 0.0, cfg.max_height).astype(np.float32)
    out = {
        "height": h,
        "normals": surface_normals(h, grid.cell, cfg.normal_radius),
        "unseen": unseen,
        "grid": grid,
    }
    if instances is not None:
        out["seg"] = project_instances(instances, valid, pts_w, grid)
    return out


def exact_heightmap(objects: list[dict], grid: Grid, table_z: float) -> dict:
    """The true top surface from the objects' poses and sizes: a vertical ray down through every
    cell centre against each box (an oriented cuboid, slab test), keeping the highest hit.

    Returns ``height`` (H, W) metres above the table, ``normals`` (3, H, W) of the face hit
    (table: +z), ``object_id`` (H, W) int32 (``meta`` object id, 0 = table) and ``face``
    (H, W) int8 (0..5 = object axis * 2 + (0 for the +axis face, 1 for -axis), -1 = table), and
    ``covered`` {object id: fraction of the object's own top-down outline inside the map where a
    different object is the top surface} (0 = nothing rests on it)."""
    rows, cols = grid.shape
    height = np.zeros((rows, cols), np.float64)
    normals = np.zeros((3, rows, cols), np.float64)
    normals[2] = 1.0
    object_id = np.zeros((rows, cols), np.int32)
    face = np.full((rows, cols), -1, np.int8)
    rc_all = np.stack(np.mgrid[0:rows, 0:cols], axis=-1)
    top = 10.0  # ray start, metres above the table
    outlines = []  # (id, row slice, col slice, cells the box's outline covers)
    for o in objects:
        T = np.asarray(o["T_world_obj"], dtype=np.float64)
        R, c = T[:3, :3], T[:3, 3]
        e = np.asarray(o["dims"], dtype=np.float64) / 2
        # only the cells under the box's world-axis bounding box
        reach = np.abs(R) @ e
        lo = grid.world_to_cell(np.array([c[0] + reach[0], c[1] - reach[1]]))
        hi = grid.world_to_cell(np.array([c[0] - reach[0], c[1] + reach[1]]))
        r0, c0 = np.maximum(np.floor(lo).astype(int), 0)
        r1, c1 = np.minimum(np.ceil(hi).astype(int) + 1, [rows, cols])
        if r0 >= r1 or c0 >= c1:
            continue
        xy = grid.cell_to_world(rc_all[r0:r1, c0:c1])
        start = np.concatenate([xy, np.full(xy.shape[:2] + (1,), table_z + top)], axis=-1)
        p = (start - c) @ R  # ray origins in the box frame
        d = R.T @ np.array([0.0, 0.0, -1.0])
        d = np.where(np.abs(d) < 1e-12, 1e-12, d)
        t1, t2 = (-e - p) / d, (e - p) / d
        t_near_axis = np.minimum(t1, t2)
        t_near = t_near_axis.max(axis=-1)
        t_far = np.maximum(t1, t2).min(axis=-1)
        z = top - t_near  # height above the table of the entry point
        cur = height[r0:r1, c0:c1]
        outline = t_near <= t_far
        outlines.append((o["id"], slice(r0, r1), slice(c0, c1), outline))
        hit = outline & (z > cur)
        if not hit.any():
            continue
        axis = t_near_axis.argmax(axis=-1)
        sign = -np.sign(d[axis])  # the entered face points against the ray
        cur[hit] = z[hit]
        n_world = R[:, axis] * sign  # (3, h, w)
        normals[:, r0:r1, c0:c1][:, hit] = n_world[:, hit]
        object_id[r0:r1, c0:c1][hit] = o["id"]
        face[r0:r1, c0:c1][hit] = (axis * 2 + (sign < 0))[hit]
    covered = {}
    for oid, rs, cs, outline in outlines:
        n = int(outline.sum())
        covered[oid] = float((outline & (object_id[rs, cs] != oid)).sum() / n) if n else 0.0
    return {
        "height": height.astype(np.float32),
        "normals": normals.astype(np.float32),
        "object_id": object_id,
        "face": face,
        "covered": covered,
        "grid": grid,
    }


DEPTH_CHANNELS = ("height", "normals", "unseen")


def network_input(hm: dict, channels=DEPTH_CHANNELS) -> np.ndarray:
    """(C, H, W) float32 network input with the named channels in order, as
    ``train_data.load_sample`` reads them: ``height`` (metres), ``normals`` (3), ``unseen`` (0 / 1),
    ``seg`` (3, :func:`seg_channels`; needs ``hm["seg"]``). The default is the depth-only input."""
    parts = {
        "height": lambda: hm["height"][None],
        "normals": lambda: hm["normals"],
        "unseen": lambda: hm["unseen"][None].astype(np.float32),
        "seg": lambda: seg_channels(hm["seg"], hm["grid"].cell),
    }
    return np.concatenate([parts[c]() for c in channels]).astype(np.float32)


def scene_heightmaps(scene: dict, cfg: HeightMapCfg, instances: np.ndarray | None = None) -> dict:
    """Height maps of a loaded scene (``dataset.load_scene``) on one grid: ``noisy`` from the
    sensor-model depth (what the network and robot see), ``clean`` from the exact rendered
    depth, and ``exact`` from the object poses and sizes (the true surface, for labels).
    ``instances`` (a predicted instance image of the scene, ``seg_pred.png``) adds ``seg`` to
    the noisy map."""
    meta = scene["meta"]
    cam = meta["camera"]
    K = np.asarray(cam["K"])
    T = np.asarray(cam["T_world_cam"])
    table_z = meta["workcell"]["table_top_z"]
    grid = grid_for_scene(meta, cfg)
    return {
        "noisy": build_heightmap(scene["depth"], K, T, table_z, grid, cfg, instances=instances),
        "clean": build_heightmap(scene["depth_clean"], K, T, table_z, grid, cfg),
        "exact": exact_heightmap(meta["objects"], grid, table_z),
    }


def save_heightmaps(path, maps: dict, cfg: HeightMapCfg, compress: bool = False) -> None:
    """Write the ``scene_heightmaps`` result to one ``.npz`` (about 3 MB uncompressed; compressing
    halves it but takes several times longer than building the maps)."""
    import json

    grid = maps["noisy"]["grid"]
    arrays = {}
    for name, hm in maps.items():
        arrays[f"{name}_height"] = hm["height"]
        arrays[f"{name}_normals"] = hm["normals"].astype(np.float16)
        for key in ("unseen", "seg", "object_id", "face"):
            if key in hm:
                arrays[f"{name}_{key}"] = hm[key]
    info = {"grid": grid.to_dict(), "normal_radius": cfg.normal_radius, "max_height": cfg.max_height, "gap_min_neighbours": cfg.gap_min_neighbours}
    (np.savez_compressed if compress else np.savez)(path, info=json.dumps(info), **arrays)
