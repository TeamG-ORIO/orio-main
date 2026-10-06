"""Depth-image geometry for the DexNet path: local plane fit and table plane.

Pure numpy/scipy (no ROS, open3d or torch) so test/test_grasp_geometry.py can
import it on the host. All points are in the camera frame, metres. A plane is
(normal, d) with normal . p + d = 0 and the normal pointing toward the camera
(d > 0), so normal . p + d is the height of p above the plane.
"""
import numpy as np
from scipy import ndimage


def deproject(depth_m, fx, fy, cx, cy, mask):
    """Camera-frame points (N, 3) for the pixels selected by a boolean mask."""
    v, u = np.nonzero(mask)
    z = depth_m[v, u].astype(np.float64)
    return np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])


def _toward_camera(normal, d):
    return (-normal, -d) if d < 0 else (normal, d)


def fit_plane_ransac(points, threshold, iterations=200, rng=None):
    """RANSAC plane, refined by least squares on the inliers.

    Returns (normal, d, inlier_mask) or None if no plane could be formed.
    """
    n_pts = len(points)
    if n_pts < 3:
        return None
    rng = np.random.default_rng(0) if rng is None else rng
    best = None
    best_count = 0
    for _ in range(iterations):
        p0, p1, p2 = points[rng.choice(n_pts, 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal /= norm
        inliers = np.abs((points - p0).dot(normal)) < threshold
        count = int(inliers.sum())
        if count > best_count:
            best, best_count = inliers, count
    if best is None or best_count < 3:
        return None

    centroid = points[best].mean(axis=0)
    _, _, vt = np.linalg.svd(points[best] - centroid, full_matrices=False)
    normal = vt[2]
    normal, d = _toward_camera(normal, -float(normal.dot(centroid)))
    inliers = np.abs(points.dot(normal) + d) < threshold
    return normal, d, inliers


def fit_local_plane(depth_m, fx, fy, cx, cy, center_px, grasp_point,
                    radius_m=0.03, depth_gate_m=0.03, ransac_threshold_m=0.004,
                    min_points=100, min_inlier_frac=0.7, max_point_offset_m=0.008):
    """Fit the plane under the suction cup from raw depth around the grasp pixel.

    The region is the cup-footprint disc around center_px, restricted to valid
    depth within depth_gate_m of the grasp depth and connected to the grasp
    pixel, so the bin floor and neighbouring objects across a gap drop out.

    Returns (fit, reason). fit is None when the patch is not a trustworthy
    plane, else a dict with normal (toward the camera), d, n_points,
    inlier_frac and offset (distance of grasp_point from the plane).
    """
    grasp_point = np.asarray(grasp_point, dtype=np.float64)
    z = float(grasp_point[2])
    if not np.isfinite(z) or z <= 0:
        return None, "grasp depth invalid"
    h, w = depth_m.shape[:2]
    u0, v0 = float(center_px[0]), float(center_px[1])
    r_px = max(fx * radius_m / z, 3.0)
    x1, x2 = max(int(u0 - r_px), 0), min(int(u0 + r_px) + 2, w)
    y1, y2 = max(int(v0 - r_px), 0), min(int(v0 + r_px) + 2, h)
    if x1 >= x2 or y1 >= y2:
        return None, "grasp pixel outside the image"

    win = depth_m[y1:y2, x1:x2]
    vv, uu = np.mgrid[y1:y2, x1:x2]
    dist2 = (uu - u0) ** 2 + (vv - v0) ** 2
    keep = (dist2 <= r_px ** 2) & np.isfinite(win) & (win > 0)
    keep &= np.abs(np.where(keep, win, z) - z) < depth_gate_m

    # Keep only the component under the grasp pixel. The pixel itself may be a
    # depth hole, so seed from the nearest kept pixel within a few pixels.
    labels, _ = ndimage.label(keep)
    seed_d2 = np.where(keep, dist2, np.inf)
    seed = np.unravel_index(np.argmin(seed_d2), seed_d2.shape)
    if not np.isfinite(seed_d2[seed]) or seed_d2[seed] > 3.0 ** 2:
        return None, "no valid depth at the grasp pixel"
    keep = labels == labels[seed]

    full = np.zeros((h, w), dtype=bool)
    full[y1:y2, x1:x2] = keep
    points = deproject(depth_m, fx, fy, cx, cy, full)
    if len(points) < min_points:
        return None, "only %d points under the cup" % len(points)

    result = fit_plane_ransac(points, ransac_threshold_m, iterations=100)
    if result is None:
        return None, "plane fit failed"
    normal, d, inliers = result
    inlier_frac = float(inliers.mean())
    if inlier_frac < min_inlier_frac:
        return None, "surface not planar (inliers %.0f%%)" % (100 * inlier_frac)
    offset = abs(float(normal.dot(grasp_point) + d))
    if offset > max_point_offset_m:
        return None, "plane misses the grasp point by %.1f mm" % (1000 * offset)
    return {"normal": normal, "d": d, "n_points": len(points),
            "inlier_frac": inlier_frac, "offset": offset}, ""


def fit_table_plane(depth_m, fx, fy, cx, cy, mask, up_cam,
                    ransac_threshold_m=0.006, max_tilt_deg=15.0,
                    below_margin_m=0.015, max_below_frac=0.05, max_samples=4000):
    """Fit the table as the dominant plane of the masked depth.

    Accepted only if it is roughly horizontal (up_cam is world +Z in the camera
    frame) and almost nothing lies below it; a box top fails the second test.
    Returns (normal, d) or None.
    """
    points = deproject(depth_m, fx, fy, cx, cy, mask)
    if len(points) < 500:
        return None
    rng = np.random.default_rng(0)
    if len(points) > max_samples:
        points = points[rng.choice(len(points), max_samples, replace=False)]
    result = fit_plane_ransac(points, ransac_threshold_m, iterations=200, rng=rng)
    if result is None:
        return None
    normal, d, _ = result
    up = np.asarray(up_cam, dtype=np.float64)
    cos_tilt = float(normal.dot(up) / np.linalg.norm(up))
    if cos_tilt < np.cos(np.radians(max_tilt_deg)):
        return None
    below_frac = float(((points.dot(normal) + d) < -below_margin_m).mean())
    if below_frac > max_below_frac:
        return None
    return normal, d


def height_above_plane(depth_m, fx, fy, cx, cy, plane):
    """Per-pixel height above the plane (m); NaN where depth is invalid."""
    normal, d = plane
    h, w = depth_m.shape[:2]
    vv, uu = np.mgrid[0:h, 0:w]
    z = depth_m.astype(np.float64)
    height = z * (normal[0] * (uu - cx) / fx + normal[1] * (vv - cy) / fy + normal[2]) + d
    height[~(np.isfinite(z) & (z > 0))] = np.nan
    return height


def above_plane_mask(depth_m, fx, fy, cx, cy, plane, min_height_m, open_px=5):
    """Pixels clearly above the plane, with depth-noise speckle opened away."""
    height = height_above_plane(depth_m, fx, fy, cx, cy, plane)
    mask = np.nan_to_num(height, nan=-np.inf) > min_height_m
    if open_px > 1:
        mask = ndimage.binary_opening(mask, structure=np.ones((open_px, open_px), bool))
    return mask
