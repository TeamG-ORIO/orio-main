"""SAT box-box collision tests (the pure primitives the CollisionChecker relies on).

These cover the plan's 'SAT overlap/disjoint boxes' case using the pure helpers
already living in robot_util (BlockDesc2Points + CheckBoxBoxCollision).
"""
import numpy as np

from orio_core import robot_util as ru


def _axis_aligned_box(center, dims):
    """Build (points, axes) for an axis-aligned box at `center` with side `dims`."""
    H = np.eye(4)
    H[:3, 3] = center
    return ru.BlockDesc2Points(H, np.asarray(dims))


def test_overlapping_boxes_collide():
    a_pts, a_ax = _axis_aligned_box([0, 0, 0], [1, 1, 1])
    b_pts, b_ax = _axis_aligned_box([0.5, 0, 0], [1, 1, 1])  # overlaps in x
    assert ru.CheckBoxBoxCollision(a_pts, a_ax, b_pts, b_ax) is True


def test_disjoint_boxes_do_not_collide():
    a_pts, a_ax = _axis_aligned_box([0, 0, 0], [1, 1, 1])
    b_pts, b_ax = _axis_aligned_box([5, 0, 0], [1, 1, 1])  # far away in x
    assert ru.CheckBoxBoxCollision(a_pts, a_ax, b_pts, b_ax) is False


def test_touching_faces_are_a_separating_axis_boundary():
    # Boxes exactly touching at x = 1 (gap 0) — SAT with >= treats contact as overlap.
    a_pts, a_ax = _axis_aligned_box([0, 0, 0], [2, 2, 2])
    b_pts, b_ax = _axis_aligned_box([2, 0, 0], [2, 2, 2])
    assert ru.CheckBoxBoxCollision(a_pts, a_ax, b_pts, b_ax) is True


def test_separated_on_a_single_axis_is_disjoint():
    a_pts, a_ax = _axis_aligned_box([0, 0, 0], [1, 1, 1])
    # overlaps in x and y, but clearly separated in z
    b_pts, b_ax = _axis_aligned_box([0, 0, 10], [1, 1, 1])
    assert ru.CheckBoxBoxCollision(a_pts, a_ax, b_pts, b_ax) is False
