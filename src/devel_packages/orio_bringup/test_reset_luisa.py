#!/usr/bin/env python3
"""Isolated reset test for the labelling arm (robot 2). Times each step so a slow
connect can be told apart from a slow/absent motion. Run inside orio_docker_container."""
import sys
import time

import numpy as np
import rospy

from frankapy import FrankaArm

HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


def main():
    robot_num = int(sys.argv[1]) if len(sys.argv) > 1 else 2

    t0 = time.time()
    rospy.init_node('test_reset_luisa', anonymous=True)
    print(f"[{time.time()-t0:6.2f}s] ros node initialised")

    t = time.time()
    fa = FrankaArm(with_gripper=False, old_gripper=False, robot_num=robot_num,
                   init_node=False)
    print(f"[{time.time()-t0:6.2f}s] FrankaArm(robot_num={robot_num}) connected "
          f"(took {time.time()-t:.2f}s)")

    t = time.time()
    before = fa.get_joints()
    print(f"[{time.time()-t0:6.2f}s] joints before: {np.round(before, 3)} "
          f"(read took {time.time()-t:.2f}s)")
    print(f"[{time.time()-t0:6.2f}s] distance from home: "
          f"{np.linalg.norm(before - HOME):.3f} rad")

    print(f"[{time.time()-t0:6.2f}s] calling reset_joints() - ARM SHOULD MOVE NOW")
    t = time.time()
    fa.reset_joints()
    print(f"[{time.time()-t0:6.2f}s] reset_joints() returned (took {time.time()-t:.2f}s)")

    after = fa.get_joints()
    moved = np.linalg.norm(after - before)
    print(f"[{time.time()-t0:6.2f}s] joints after:  {np.round(after, 3)}")
    print(f"[{time.time()-t0:6.2f}s] moved {moved:.3f} rad; "
          f"now {np.linalg.norm(after - HOME):.3f} rad from home")

    if np.linalg.norm(after - HOME) < 0.05:
        print("RESULT: PASS - arm reached home")
    elif moved < 0.01:
        print("RESULT: FAIL - arm did not move at all")
    else:
        print("RESULT: PARTIAL - arm moved but is not at home")


if __name__ == '__main__':
    main()
