"""
build_obstacles.py

Build obstacle points and axes for a given arm and store them in a pickle file.

Usage:
    python build_obstacles.py --arm 1 --inflate 10 --output obs_arm1.p
"""

import argparse
import pickle
import numpy as np
import RobotUtil as rt
from scene_obstacles import get_obstacles_for_arm


def build_obstacles(arm_number, inflation_pct=0.0):
    blocks = get_obstacles_for_arm(arm_number)
    pointsObs = []
    axesObs   = []
    scale = 1.0 + inflation_pct / 100.0
    for _, pos, size in blocks:
        inflated_size = [s * scale for s in size]
        envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0, 0., 0.], pos), inflated_size)
        pointsObs.append(envpoints)
        axesObs.append(envaxes)
    return np.array(pointsObs), np.array(axesObs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build obstacle points/axes and save to pickle.")
    parser.add_argument("--arm", type=int, choices=[1, 2], required=True,
                        help="Arm to build obstacles for: 1=left, 2=right")
    parser.add_argument("--inflate", type=float, default=0.0,
                        help="Inflate obstacle dimensions by this percentage (e.g. 10 = +10%%)")
    parser.add_argument("--output", type=str, default="../orio/obstacles_files/obs_arm{}_free.p".format(2),
                        help="Output pickle file path")
    args = parser.parse_args()

    print(f"Building obstacles for arm {args.arm} with {args.inflate}%% inflation...")
    pointsObs, axesObs = build_obstacles(args.arm, args.inflate)
    print(f"  {len(pointsObs)} obstacle blocks built.")

    with open(args.output, "wb") as f:
        pickle.dump(pointsObs, f)
        pickle.dump(axesObs, f)

    print(f"Saved to {args.output}")
