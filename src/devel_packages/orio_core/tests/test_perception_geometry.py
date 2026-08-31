"""Tests for the pure pinhole deprojection extracted from compute_label_joints."""
import numpy as np

from orio_core import perception_geometry as pg

# The real label-camera intrinsics from state_machine.py (for a realistic check).
FX, FY = 1509.65, 1509.65
CX, CY = 964.33, 559.28
CAMERA_HEIGHT = 1.0


def test_depth_from_camera_height():
    assert pg.depth_from_camera_height(1.0, 0.2) == 0.8


def test_deproject_center_pixel_has_zero_xy():
    X, Y, Z = pg.deproject_pixel(CX, CY, 0.8, FX, FY, CX, CY)
    assert np.isclose(X, 0.0)
    assert np.isclose(Y, 0.0)
    assert Z == 0.8


def test_deproject_matches_inline_formula():
    u, v, z = 1200.0, 400.0, 0.8
    X, Y, Z = pg.deproject_pixel(u, v, z, FX, FY, CX, CY)
    assert np.isclose(X, (u - CX) * z / FX)
    assert np.isclose(Y, (v - CY) * z / FY)
    assert Z == z


def test_camera_to_world_identity():
    world = pg.camera_to_world([1.0, 2.0, 3.0], np.eye(4))
    assert np.allclose(world, [1.0, 2.0, 3.0])


def test_camera_to_world_translation_and_rotation():
    T = np.eye(4)
    T[:3, 3] = [10.0, 0.0, 0.0]          # translate +x by 10
    world = pg.camera_to_world([1.0, 2.0, 3.0], T)
    assert np.allclose(world, [11.0, 2.0, 3.0])


def test_label_target_position_composes_and_keeps_item_depth():
    T = np.eye(4)
    T[:3, 3] = [0.5, -0.5, 0.0]
    item_depth = 0.2
    world_xyz, task_pos = pg.label_target_position(
        u=CX, v=CY, item_depth=item_depth, camera_height=CAMERA_HEIGHT,
        fx=FX, fy=FY, cx=CX, cy=CY, T_world_cam=T)
    # center pixel -> camera xy 0, so world xy == translation
    assert np.allclose(world_xyz[:2], [0.5, -0.5])
    # task_pos z is always the item depth, not the camera Z
    assert task_pos == [world_xyz[0], world_xyz[1], item_depth]
