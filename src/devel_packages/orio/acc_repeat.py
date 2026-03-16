import ikpy.chain
import numpy as np
from frankapy import FrankaArm

# ---------------------------------------------------------
# 1. INITIALIZATION & SETUP
# ---------------------------------------------------------
fa = FrankaArm(with_gripper=False)

my_chain = ikpy.chain.Chain.from_urdf_file(
    "panda_arm_hand.urdf", 
    base_elements=["panda_link0"] 
)

chain_length = len(my_chain.links)
active_mask = [False] * chain_length
for i in range(1, 8):
    if i < chain_length:
        active_mask[i] = True
my_chain.active_links_mask = active_mask

# Fixed orientation (facing downwards)
target_ori = np.array([
    [1.0,  0.0,  0.0], 
    [0.0, -1.0,  0.0], 
    [0.0,  0.0, -1.0]
])

# ---------------------------------------------------------
# 2. GENERATE RANDOM POSES (FIXED SEED)
# ---------------------------------------------------------
np.random.seed(42)
NUM_POSES = 10
NUM_TRIALS = 3  # Number of times to visit each pose for repeatability

# Generate 10 target poses within the requested workspace limits
target_positions = []
for _ in range(NUM_POSES):
    x = np.random.uniform(0.2, 0.8)
    y = np.random.uniform(-0.3, 0.3)
    z = np.random.uniform(0.15, 0.75)
    target_positions.append(np.array([x, y, z]))

# Dictionary to store collected data
results = {i: {
    'target_pos': target_positions[i],
    'actual_positions': [],
    'actual_joints': [],
    'target_joints': None 
} for i in range(NUM_POSES)}

# ---------------------------------------------------------
# 3. EXECUTE TESTING LOOP
# ---------------------------------------------------------
print(f"Starting Accuracy & Repeatability Test: {NUM_POSES} Poses, {NUM_TRIALS} Trials each.")

for trial in range(NUM_TRIALS):
    print(f"\n--- Starting Trial {trial + 1}/{NUM_TRIALS} ---")
    
    for pose_idx, target_pos in enumerate(target_positions):
        # Always start from reset position per instructions
        fa.reset_joints()
        
        # Formulate initial guess
        initial_guess = [0.0] * chain_length
        if 4 < chain_length:
            initial_guess[4] = -1.5 
            
        # Compute Inverse Kinematics
        joint_angles = my_chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_ori,
            orientation_mode="all",
            initial_position=initial_guess
        )
        
        # Save the IK target joints once (extracting the 7 arm joints)
        if results[pose_idx]['target_joints'] is None:
            results[pose_idx]['target_joints'] = np.array(joint_angles[1:8])
            
        # Command the robot
        fa.goto_joints(joint_angles[1:8])
        
        # ---------------------------------------------------------
        # 4. DATA EXTRACTION
        # ---------------------------------------------------------
        # Read the exact joints reached via encoders (returns 7 joint angles)
        end_joints = np.array(fa.get_joints())
        
        # Calculate world frame pose using Forward Kinematics from actual joints
        transforms = fa.get_links_transforms(end_joints, use_rigid_transforms=False)
        actual_ee_pos = transforms[-1][:3, 3] 
        
        # Store data
        results[pose_idx]['actual_positions'].append(actual_ee_pos)
        results[pose_idx]['actual_joints'].append(end_joints)

        print(f"  Pose {pose_idx + 1:02d} reached.")

# ---------------------------------------------------------
# 5. DATA ANALYSIS & RESULTS
# ---------------------------------------------------------
print("\n=========================================================================================")
print("TEST RESULTS (Cartesian values in meters, Joint values in radians)")
print("=========================================================================================")

cartesian_accuracy_errors = []
cartesian_repeatability_errors = []
joint_accuracy_errors = []
joint_repeatability_errors = []

for pose_idx in range(NUM_POSES):
    # --- Cartesian Data ---
    target_p = results[pose_idx]['target_pos']
    actual_ps = np.array(results[pose_idx]['actual_positions'])
    mean_actual_p = np.mean(actual_ps, axis=0)
    
    # --- Joint Data ---
    target_j = results[pose_idx]['target_joints']
    actual_js = np.array(results[pose_idx]['actual_joints'])
    mean_actual_j = np.mean(actual_js, axis=0)
    
    # --- Calculate Cartesian Metrics ---
    c_acc_err = np.linalg.norm(mean_actual_p - target_p)
    c_devs = [np.linalg.norm(p - mean_actual_p) for p in actual_ps]
    c_rep_err = np.sqrt(np.mean(np.square(c_devs)))
    
    cartesian_accuracy_errors.append(c_acc_err)
    cartesian_repeatability_errors.append(c_rep_err)

    # --- Calculate Joint Metrics ---
    # Accuracy: 7D distance between target joint state and mean actual joint state
    j_acc_err = np.linalg.norm(mean_actual_j - target_j)
    # Repeatability: RMS of 7D distances from mean actual joint state
    j_devs = [np.linalg.norm(j - mean_actual_j) for j in actual_js]
    j_rep_err = np.sqrt(np.mean(np.square(j_devs)))

    joint_accuracy_errors.append(j_acc_err)
    joint_repeatability_errors.append(j_rep_err)
    
    # --- Print Results for the Pose ---
    print(f"Pose {pose_idx + 1:02d} | Target X,Y,Z: [{target_p[0]:.3f}, {target_p[1]:.3f}, {target_p[2]:.3f}]")
    print(f"          | Cartesian -> Acc: {c_acc_err:.6f} m   | Rep: {c_rep_err:.6f} m")
    print(f"          | Joint     -> Acc: {j_acc_err:.6f} rad | Rep: {j_rep_err:.6f} rad")
    print("          |----------------------------------------------------------------------")

print("=========================================================================================")
print(f"OVERALL AVG CARTESIAN ACCURACY      : {np.mean(cartesian_accuracy_errors):.6f} m")
print(f"OVERALL AVG CARTESIAN REPEATABILITY : {np.mean(cartesian_repeatability_errors):.6f} m")
print(f"OVERALL AVG JOINT ACCURACY          : {np.mean(joint_accuracy_errors):.6f} rad")
print(f"OVERALL AVG JOINT REPEATABILITY     : {np.mean(joint_repeatability_errors):.6f} rad")
print("=========================================================================================")