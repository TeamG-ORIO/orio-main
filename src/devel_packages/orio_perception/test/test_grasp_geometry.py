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

print()
if FAILURES:
    print("%d FAILED: %s" % (len(FAILURES), ", ".join(FAILURES)))
    sys.exit(1)
print("all geometry checks passed")
