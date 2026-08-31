"""
Usage:
python run_guide_mode.py
Commands:
1-> Print Tool Pose
2-> Print Joint Pose
3-> Stop Skill and Exit
4-> Stop Skill and Reset to Home Position
"""
import numpy as np
import time
import sys
import argparse
from frankapy import FrankaArm

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run guide mode for a specific Franka arm.")
    parser.add_argument(
        '--robot_num', '-n', 
        type=int, 
        default=1, 
        help='The robot number to connect to (e.g., 1 or 2). Defaults to 1.'
    )

    args = parser.parse_args()
    if args.robot_num == 1:
        robot_name = "Pick-and-Place Arm"
    elif args.robot_num == 2:
        robot_name = "Labelling Arm"
    else:
        robot_name = None

    start = time.time()
    print(f"Guide Mode Started for Robot {args.robot_num} ({robot_name})")
    fa = FrankaArm(with_gripper=False, old_gripper=False, robot_num=args.robot_num)
    #fa.open_gripper()
    fa.run_guide_mode(10000,block=False)

    while((time.time()-start) < 10000):
        input_num = int(input("Enter a number: "))
        if (input_num==1):
            T_ee_world = fa.get_pose()
            print("pose: ")
            print(T_ee_world)
            time.sleep(0.01)
        if(input_num==2):
            joints = fa.get_joints()
            print("joints: ")
            print(joints)
            time.sleep(0.01)
        if(input_num==3):
            fa.stop_skill()
            break
        if(input_num==4):
            fa.stop_skill()
            fa.reset_joints()
            break

    print("done")
