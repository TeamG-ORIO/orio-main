#!/usr/bin/env python3
"""Offline checks for the DexNet grasp/motion maths. No ROS or GPU needed.

    python3 test/test_grasp_geometry.py
"""
import sys

import numpy as np

VERTICAL_ORI = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

FAILURES = []


def check(label, ok, detail=""):
    print("  %-52s %s%s" % (label, "PASS" if ok else "FAIL",
                            "" if ok else "  <- " + detail))
    if not ok:
        FAILURES.append(label)


def rotation_from_approach(approach):
    """Mirror of CombinedPerceptionNode._rotation_from_approach."""
    z = approach / np.linalg.norm(approach)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, z)) > 0.95:
        ref = np.array([0.0, 1.0, 0.0])
    x = ref - np.dot(ref, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def waypoints(task_pos, task_ori=None, pre=0.10, contact=0.05):
    """Mirror of compute_pick_joints' waypoint maths."""
    ori = VERTICAL_ORI if task_ori is None else np.asarray(task_ori)
    retreat = -ori[:, 2]
    t = np.asarray(task_pos, dtype=float)
    return t + retreat * pre, t + retreat * contact


def build_approach(deg):
    return np.array([np.sin(np.radians(deg)), 0.0, -np.cos(np.radians(deg))])


print("rotation_from_approach: orthonormal, right-handed, Z == approach")
for a in [np.array([0, 0, -1.0]), np.array([0.3, -0.2, -0.93]),
          np.array([1.0, 0, 0]), np.array([0, 1.0, 0])]:
    a = a / np.linalg.norm(a)
    R = rotation_from_approach(a)
    check("axis %s" % np.round(a, 3),
          np.allclose(R.T @ R, np.eye(3), atol=1e-9)
          and abs(np.linalg.det(R) - 1.0) < 1e-9
          and np.allclose(R[:, 2], a, atol=1e-9))

print("quaternion round-trip preserves the approach axis")
try:
    from scipy.spatial.transform import Rotation as R_scipy
    a = np.array([0.3, -0.2, -0.93])
    a /= np.linalg.norm(a)
    R = rotation_from_approach(a)
    back = R_scipy.from_quat(R_scipy.from_matrix(R).as_quat()).as_matrix()
    check("max error %.1e" % np.abs(R - back).max(),
          np.allclose(back[:, 2], a, atol=1e-9))
except ImportError:
    print("  (scipy unavailable, skipped)")

print("waypoint parity: default == the original world-Z offsets")
task = np.array([0.5, 0.1, 0.3])
pre, fin = waypoints(task)
check("pre == z+0.10", np.allclose(pre, [task[0], task[1], task[2] + 0.10]))
check("final == z+0.05", np.allclose(fin, [task[0], task[1], task[2] + 0.05]))
pre2, fin2 = waypoints(task, VERTICAL_ORI)
check("explicit vertical == default",
      np.allclose(pre, pre2) and np.allclose(fin, fin2))

print("tilted waypoints lie along the approach axis")
for deg in (0, 20, 45):
    ori = rotation_from_approach(build_approach(deg))
    pre_t, fin_t = waypoints(task, ori)
    v1, v2 = pre_t - task, fin_t - task
    check("tilt %2d deg colinear, correct length and direction" % deg,
          np.allclose(np.cross(v1, v2), 0, atol=1e-9)
          and abs(np.linalg.norm(v1) - 0.10) < 1e-9
          and np.dot(v1, -build_approach(deg)) > 0)

print("retract undoes the contact approach")
for deg in (0, 20, 45):
    ori = rotation_from_approach(build_approach(deg))
    _, fin_t = waypoints(task, ori)
    landed = fin_t + (-ori[:, 2] * 0.05)
    check("tilt %2d deg lands 0.10 m off the target along the axis" % deg,
          abs(np.linalg.norm(landed - task) - 0.10) < 1e-9)

print("tilt gate arithmetic")
for deg in (0, 20, 45, 60):
    ori = rotation_from_approach(build_approach(deg))
    measured = np.degrees(np.arccos(np.clip(-ori[2, 2], -1.0, 1.0)))
    check("built %2d deg -> measured %.2f deg" % (deg, measured),
          abs(measured - deg) < 1e-6)

print("bin mask depth band")
lo, hi = 0.60, 1.20
depth = np.array([[0.5, 0.7], [1.1, 1.5]], dtype=np.float32)
mask = ((depth >= lo) & (depth <= hi) & np.isfinite(depth)).astype(np.uint8) * 255
check("keeps only in-band pixels", mask.tolist() == [[0, 255], [255, 0]])
empty = np.full((4, 4), 5.0, dtype=np.float32)
check("empty bin yields an empty mask",
      not ((empty >= lo) & (empty <= hi)).any())

print("plane fit under the cup (scripts/dexnet_geometry.py)")
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import dexnet_geometry as dg

FX = FY = 570.3
CX, CY = 204.5, 199.5
H, W = 365, 400
VV, UU = np.mgrid[0:H, 0:W]
RNG = np.random.default_rng(1)


def plane_depth(normal, point):
    """Depth image of the plane through `point` with the given normal."""
    rays = np.dstack([(UU - CX) / FX, (VV - CY) / FY, np.ones((H, W))])
    return (np.dot(normal, point) / rays.dot(normal)).astype(np.float32)


def angle_deg(a, b):
    return np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), -1.0, 1.0)))


def point_at(depth, px):
    z = float(depth[int(px[1]), int(px[0])])
    return np.array([(px[0] - CX) * z / FX, (px[1] - CY) * z / FY, z])


table = plane_depth(np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, 0.90]))
for deg in (0, 20, 40):
    n = np.array([np.sin(np.radians(deg)), 0.0, -np.cos(np.radians(deg))])
    depth = plane_depth(n, np.array([0.0, 0.0, 0.85]))
    depth = depth + RNG.normal(0.0, 0.0015, depth.shape).astype(np.float32)
    px = (204.0, 199.0)
    fit, why = dg.fit_local_plane(depth, FX, FY, CX, CY, px, point_at(depth, px))
    check("noisy %2d deg surface recovered within 3 deg" % deg,
          fit is not None and angle_deg(fit["normal"], n) < 3.0, why)

# A flat box top next to a 5 cm drop to the table: the floor must not tilt the fit.
depth = table.copy()
depth[150:250, 150:215] = 0.85
depth = depth + RNG.normal(0.0, 0.0015, depth.shape).astype(np.float32)
px = (208.0, 199.0)
fit, why = dg.fit_local_plane(depth, FX, FY, CX, CY, px, point_at(depth, px))
check("grasp beside a step edge stays flat",
      fit is not None and angle_deg(fit["normal"], [0, 0, 1]) < 3.0, why)

# Depth hole at the grasp pixel itself is tolerated.
holed = depth.copy()
holed[199, 208] = 0.0
fit, why = dg.fit_local_plane(holed, FX, FY, CX, CY, px, np.array([0.006, 0.0, 0.85]))
check("hole at the grasp pixel is tolerated", fit is not None, why)

# A ridge (two faces meeting under the cup) is not a plane.
ridge = np.where(UU < 204, plane_depth(np.array([0.5, 0.0, -0.866]), np.array([0, 0, 0.85])),
                 plane_depth(np.array([-0.5, 0.0, -0.866]), np.array([0, 0, 0.85])))
fit, why = dg.fit_local_plane(ridge, FX, FY, CX, CY, (204.0, 199.0), np.array([0, 0, 0.85]))
check("ridge under the cup is rejected", fit is None)

fit, why = dg.fit_local_plane(np.zeros((H, W), np.float32), FX, FY, CX, CY,
                              (204.0, 199.0), np.array([0, 0, 0.85]))
check("no depth is rejected", fit is None)

print("table plane and above-table mask")
up_cam = np.array([0.0, 0.0, -1.0])
scene = table.copy()
scene[100:160, 100:180] = 0.85            # 5 cm box
scene[250:300, 250:330] = 0.897           # 3 mm sheet: invisible to geometry
scene = scene + RNG.normal(0.0, 0.002, scene.shape).astype(np.float32)
band = (scene > 0.6) & (scene < 1.2)
plane = dg.fit_table_plane(scene, FX, FY, CX, CY, band, up_cam)
check("table plane found and horizontal",
      plane is not None and angle_deg(plane[0], up_cam) < 1.0
      and abs(plane[1] - 0.90) < 0.003)
if plane is not None:
    above = dg.above_plane_mask(scene, FX, FY, CX, CY, plane, 0.015)
    check("box is above the table", above[105:155, 105:175].mean() > 0.95)
    check("bare table and thin sheet are not", above[200:240, 200:240].sum() == 0
          and above[250:300, 250:330].sum() == 0)

# A large box top over a small visible table strip must not be taken for the table.
covered = table.copy()
covered[:, :300] = 0.80
check("box top is not accepted as the table",
      dg.fit_table_plane(covered, FX, FY, CX, CY, np.ones((H, W), bool), up_cam) is None)

print()
if FAILURES:
    print("%d FAILED: %s" % (len(FAILURES), ", ".join(FAILURES)))
    sys.exit(1)
print("all geometry checks passed")
