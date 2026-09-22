import argparse
from frankapy import FrankaArm

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_pose', '-u', action='store_true')
    parser.add_argument('--close_grippers', '-c', action='store_true')
    parser.add_argument('--robot_num', '-r', type=int, default=1,
                        help='1 = iam-doc (default), 2 = iam-luisa')
    parser.add_argument('--no_gripper', '-n', action='store_true',
                        help='skip the gripper open/close (arms started with --with_gripper 0)')
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm(robot_num=args.robot_num)

    if args.use_pose:
        print('Reset with pose')
        fa.reset_pose()
    else:
        print('Reset with joints')
        fa.reset_joints()

    if args.no_gripper:
        print('Skipping gripper')
    elif args.close_grippers:
        print('Closing Grippers')
        fa.close_gripper()
    else:
        print('Opening Grippers')
        fa.open_gripper()
