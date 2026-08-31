"""Pure camera/perception geometry.

These functions are the *honest* extraction of the pinhole-deprojection math
currently inlined in the state machine's ``compute_label_joints`` (see
``state_machine.py`` lines ~389-392). They take intrinsics, depth, and a
camera->world transform as plain arguments and return numbers — no rospy, no
service calls, no TF lookups. Those side effects stay in the node, which fetches
``K``, ``depth`` and ``T`` from ROS and then calls these functions.

Reference (inlined) computation being reproduced::

    Z   = CAMERA_HEIGHT - item_depth
    X   = (u - CX) * Z / FX
    Y   = (v - CY) * Z / FY
    pos = T_world_cam @ [X, Y, Z, 1.0]
"""

import numpy as np


def depth_from_camera_height(camera_height, item_depth):
    """Camera-frame Z of a point ``item_depth`` metres below a camera mounted
    ``camera_height`` metres above the robot base frame."""
    return camera_height - item_depth


def deproject_pixel(u, v, z, fx, fy, cx, cy):
    """Back-project a pixel ``(u, v)`` at camera-frame depth ``z`` to a 3D point
    in the camera frame using a pinhole model.

    Returns ``(X, Y, Z)`` with ``X = (u-cx)*z/fx``, ``Y = (v-cy)*z/fy``, ``Z = z``.
    """
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    return X, Y, z


def camera_to_world(xyz_cam, T_world_cam):
    """Transform a camera-frame point ``(x, y, z)`` into the world frame using the
    4x4 homogeneous transform ``T_world_cam``. Returns a length-3 numpy array."""
    p = np.asarray([xyz_cam[0], xyz_cam[1], xyz_cam[2], 1.0])
    world = np.asarray(T_world_cam) @ p
    return world[:3]


def label_target_position(u, v, item_depth, camera_height, fx, fy, cx, cy,
                          T_world_cam):
    """Compose the full deprojection used by ``compute_label_joints``.

    Given a detected label pixel ``(u, v)``, the item's depth, the camera height,
    the pinhole intrinsics and the camera->world transform, return
    ``(world_xyz, task_pos)`` where ``task_pos = [world_x, world_y, item_depth]``
    — exactly the quantity the state machine feeds to IK.
    """
    z = depth_from_camera_height(camera_height, item_depth)
    xyz_cam = deproject_pixel(u, v, z, fx, fy, cx, cy)
    world_xyz = camera_to_world(xyz_cam, T_world_cam)
    task_pos = [world_xyz[0], world_xyz[1], item_depth]
    return world_xyz, task_pos
