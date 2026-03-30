import ikpy.chain
import numpy as np
from frankapy import FrankaArm

fa = FrankaArm(with_gripper=False, robot_num=2)

my_chain = ikpy.chain.Chain.from_urdf_file(
    "panda_arm_hand.urdf", 
    base_elements=["panda_link0"] 
)

# 1. Get the exact number of links in the chain
chain_length = len(my_chain.links)

# 2. Dynamically build the active links mask
# Start by setting ALL joints to False (inactive)
active_mask = [False] * chain_length
# Turn on exactly the 7 revolute arm joints (indices 1 through 7)
for i in range(1, 8):
    if i < chain_length:
        active_mask[i] = True
my_chain.active_links_mask = active_mask

fa.reset_joints()

# 3. Set up the target translation and rotation
target_matrix = np.eye(4)
target_matrix[:3, 3] = [0.5, 0.0, 0.4] 

target_pos = target_matrix[:3, 3]
target_ori = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

# 4. Dynamically build the initial guess array
initial_guess = [0.0] * chain_length
# Keep Joint 4 within its physical limits to avoid the bounds crash
if 4 < chain_length:
    initial_guess[4] = -1.5 

# 5. Calculate IK
joint_angles = my_chain.inverse_kinematics(
    target_position=target_pos,
    target_orientation=target_ori,
    orientation_mode="all",
    initial_position=initial_guess
)

# 6. Command the robot
fa.goto_joints(joint_angles[1:8])
print("===============================")
end_joints = fa.get_joints()
print(fa.get_links_transforms(end_joints, use_rigid_transforms=False))
print("===============================")
print(end_joints)
print(joint_angles[1:8])
