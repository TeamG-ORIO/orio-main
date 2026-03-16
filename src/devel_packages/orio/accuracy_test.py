import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

if __name__ == "__main__":
    print("Testing FrankaArm API...")
    fa = FrankaArm(with_gripper=False)
    fa.reset_joints()

    T_ee_world = fa.get_pose()
    print("Current end-effector pose in world frame:")
    print(T_ee_world)

    joints = fa.get_joints()
    print("Current joint angles:")
    print(joints)

    force_torque = fa.get_ee_force_torque()
    print("Current end-effector force-torque readings:")
    print(force_torque)

    # Move to a new pose 0.13
    des_pose = RigidTransform(translation=np.array([0.8, 0.0, 0.09]), \
                              rotation=[[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],\
                               from_frame="franka_tool", to_frame="world")
    print(des_pose)
    fa.goto_pose(des_pose, duration=10.0, use_impedance=True,\
                 cartesian_impedances=[1000, 1000, 100, 100, 100, 100], ignore_virtual_walls=False)
    print("End pose joint angles:")
    end_joints = fa.get_joints()
    print(end_joints)
    print(fa.get_links_transforms(end_joints, use_rigid_transforms=False))
    # fa.reset_joints()

    # TO_REACH_JOINTS = [-4.49758033e-04, -7.42432995e-01,  9.18272823e-04, -2.32617059e+00, 6.83349259e-04,  1.57222114e+00,  7.26028551e-01]
    # fa.goto_joints(TO_REACH_JOINTS, duration=5.0, use_impedance=False, ignore_virtual_walls=False)
    # print("End pose joint angles:")
    # print(fa.get_joints())
    # fa.reset_joints()