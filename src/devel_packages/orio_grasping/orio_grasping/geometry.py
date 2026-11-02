"""Small pure-numpy helpers. Nothing here imports Isaac Sim, so it can be unit tested anywhere."""

from __future__ import annotations

import math

import numpy as np


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation matrix (3x3) to quaternion (w, x, y, z)."""
    m = np.asarray(R, dtype=np.float64)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def matrix_from_quat(q: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) to rotation matrix (3x3)."""
    w, x, y, z = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy_from_matrix(R: np.ndarray) -> tuple[float, float, float]:
    """Rotation matrix to (roll, pitch, yaw) in radians, extrinsic X-Y-Z convention:
    ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``. Pitch is in [-pi/2, pi/2]."""
    m = np.asarray(R, dtype=np.float64)
    pitch = math.atan2(-m[2, 0], math.hypot(m[0, 0], m[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:  # gimbal lock: fold roll into yaw
        return (0.0, pitch, math.atan2(-m[0, 1], m[1, 1]))
    return (math.atan2(m[2, 1], m[2, 2]), pitch, math.atan2(m[1, 0], m[0, 0]))


def matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Inverse of :func:`rpy_from_matrix`."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_from_axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), a[0] * s, a[1] * s, a[2] * s])


def look_at_quat_world_convention(
    eye: tuple[float, float, float], target: tuple[float, float, float], up_hint: tuple[float, float, float]
) -> np.ndarray:
    """Orientation (w, x, y, z) of a frame at ``eye`` whose +x axis points at ``target``.

    This is Isaac Lab's ``"world"`` camera convention: forward +x, up +z. The frame's +z axis is
    the projection of ``up_hint`` perpendicular to the viewing direction, so ``up_hint`` controls
    the roll of the image (which world direction appears at the top of the picture).
    """
    f = np.asarray(target, dtype=np.float64) - np.asarray(eye, dtype=np.float64)
    f /= np.linalg.norm(f)
    u = np.asarray(up_hint, dtype=np.float64)
    u = u - np.dot(u, f) * f
    if np.linalg.norm(u) < 1e-6:
        raise ValueError("up_hint is parallel to the viewing direction")
    u /= np.linalg.norm(u)
    left = np.cross(u, f)
    R = np.stack([f, left, u], axis=1)
    return quat_from_matrix(R)


def random_quat(rng: np.random.Generator, n: int) -> np.ndarray:
    """Uniformly distributed random unit quaternions (w, x, y, z), shape (n, 4)."""
    u1, u2, u3 = rng.random((3, n))
    a, b = np.sqrt(1 - u1), np.sqrt(u1)
    q = np.stack(
        [b * np.cos(2 * np.pi * u3), a * np.sin(2 * np.pi * u2), a * np.cos(2 * np.pi * u2), b * np.sin(2 * np.pi * u3)],
        axis=1,
    )
    return q


def yaw_quat(yaw: np.ndarray) -> np.ndarray:
    """Quaternions (w, x, y, z) for rotations about +z, shape (n, 4)."""
    yaw = np.asarray(yaw, dtype=np.float64)
    return np.stack([np.cos(yaw / 2), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2)], axis=1)


def rotated_half_extents(dims: np.ndarray, quats: np.ndarray) -> np.ndarray:
    """World-axis-aligned half extents of boxes with full dimensions ``dims`` (n, 3) and orientations ``quats`` (n, 4)."""
    out = np.zeros_like(dims, dtype=np.float64)
    for i in range(len(dims)):
        R = matrix_from_quat(quats[i])
        out[i] = np.abs(R) @ (np.asarray(dims[i]) / 2)
    return out
