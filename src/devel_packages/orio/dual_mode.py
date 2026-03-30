import numpy as np
import time
import sys
from frankapy import FrankaArm

if __name__ == "__main__":
    
    start = time.time()
    #print("Guide Mode Started")
    fa1 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1)
    print("connected to arm1")
    fa2 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2)
    print("connected to arm2")
    #fa.open_gripper()
    fa1.run_guide_mode(10000,block=False)
    print("guide mode for arm1 started")
    fa2.run_guide_mode(10000,block=False)
    print("guide mode for arm2 started")

    while((time.time()-start) < 10000):
        input_num = int(input("Enter a number: "))
        if (input_num==1):
            T_ee_world_1 = fa1.get_pose()
            T_ee_world_2 = fa2.get_pose()
            print("pose 1: ")
            print(T_ee_world_1)
            print("pose 2: ")
            print(T_ee_world_2)
            time.sleep(0.01)
        if(input_num==2):
            joints1 = fa1.get_joints()
            joints2 = fa2.get_joints()
            print("joints 1: ")
            print(joints1)
            print("joints 2: ")
            print(joints2)
            time.sleep(0.01)
        if(input_num==3):
            fa1.stop_skill()
            fa2.stop_skill()
            break
        if(input_num==4):
            fa1.stop_skill()
            fa1.reset_joints()
            fa2.stop_skill()
            fa2.reset_joints()
            break

    print("done")
