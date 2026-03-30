import time
from frankapy import FrankaArm

def parse_calibration_file(filepath):
    """
    Reads the text file, removes brackets, and chunks the data 
    into lists of 7 joint angles without using 're'.
    """
    with open(filepath, 'r') as f:
        text = f.read()
            
    # Replace brackets with spaces
    text = text.replace("[", " ").replace("]", " ")
    
    # Split by any whitespace and convert to floats
    floats = []
    for token in text.split():
        try:
            floats.append(float(token))
        except ValueError:
            pass # Skips anything that isn't a valid number
            
    # Group the flat list of floats into lists of exactly 7 joints
    joint_configs = [floats[i:i+7] for i in range(0, len(floats), 7) if len(floats[i:i+7]) == 7]
    
    return joint_configs


if __name__ == "__main__":
    # 1. Initialize the arm
    fa = FrankaArm(with_gripper=False, robot_num=1)
    
    # 2. Load the joint configurations
    filepath = "franka_calibration_joints.txt"
    calibration_joints = parse_calibration_file(filepath)
    print(f"Successfully loaded {len(calibration_joints)} configurations.")
    
    # 3. Start at the reset position
    print("Moving to initial reset position...")
    fa.reset_joints()
    print("At reset position. Starting calibration sequence")
    
    try:
        # 4. Sequentially move through the calibration joints
        for i, joints in enumerate(calibration_joints):
            print(f"Moving to configuration {i + 1} / {len(calibration_joints)}...")
            
            # Command the robot to the specific joint angles
            fa.goto_joints(joints)
            
            # Print trigger and wait 5 seconds
            print("PRESS")
            time.sleep(5)
            
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user.")
        
    finally:
        # 5. End at the reset position (runs even if you Ctrl+C to stop early)
        print("Returning to reset position...")
        fa.reset_joints()
        print("Done.")