import time
from frankapy import FrankaArm

if __name__ == "__main__":
    start = time.time()

    fa1 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1)
    print("connected to arm1")
    fa2 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2)
    print("connected to arm2")

    fa1.stop_skill()
    fa2.stop_skill()

    fa1.reset_joints(block=False)
    fa2.goto_joints([-0.065, -1.2718, 0.0356, -2.707, 0.0343, 1.4354, 0.7355], block=False)
    # fa2.reset_joints(block=False)

    fa1.wait_for_skill()
    fa2.wait_for_skill()

    print(f"done in {time.time() - start:.1f}s")
