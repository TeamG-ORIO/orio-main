import devel_packages.prm.SimpleFranka as SimpleFranka
import numpy as np
import random
import pickle
import RobotUtil as rt
import time
from scene_obstacles import get_obstacles_for_arm

random.seed(13)

#Initialize robot object
mybot=SimpleFranka.FrankArm()

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
    return pointsObs, axesObs

start = time.time()

def FindKNN(q_new, k, prmVertices):
    dists = np.linalg.norm(np.array(prmVertices) - q_new, axis=1)
    knn_indices = np.argsort(dists)[:k]
    return knn_indices


def PRMGenerator(arm_number, inflation_pct=0.0):
    prmVertices = []
    prmEdges    = []

    pointsObs, axesObs = build_obstacles(arm_number, inflation_pct)
    pointsObs = np.array(pointsObs)
    axesObs   = np.array(axesObs)

    while len(prmVertices) < 1000:
        qmin = [-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
        qmax = [ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
        q_new = np.random.uniform(low=qmin, high=qmax)
        if q_new is None:
            continue
        Tcurr, _ = mybot.ForwardKin(q_new)
        ee_x = Tcurr[-1][0, 3]
        ee_z = Tcurr[-1][2, 3]
        if ee_x < 0 or ee_z < 0 or ee_z > 0.7:
            continue
        if not mybot.DetectCollision(q_new, pointsObs, axesObs):
            if len(prmVertices) == 0:
                prmVertices.append(q_new)
                prmEdges.append([])
                continue
            knn = FindKNN(q_new, 10, prmVertices)
            prmVertices.append(q_new)
            prmEdges.append([])
            new_idx = len(prmVertices) - 1

            if new_idx % 10 == 0:
                print(new_idx)

            for idx in knn:
                q_near = prmVertices[idx]
                if not mybot.DetectCollisionEdge(q_new, q_near, pointsObs, axesObs):
                    prmEdges[new_idx].append(idx)
                    prmEdges[idx].append(new_idx)

    # Save the PRM — filename encodes which arm it was built for
    fname = f"prm_files/myPRM_arm{arm_number}.p"
    with open(fname, 'wb') as f:
        pickle.dump(prmVertices, f)
        pickle.dump(prmEdges, f)
        pickle.dump(pointsObs, f)
        pickle.dump(axesObs, f)
    print(f"Saved {len(prmVertices)} vertices to {fname}")
    return prmVertices, prmEdges, pointsObs, axesObs

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", type=int, choices=[1, 2], required=True,
                        help="Arm to build PRM for: 1=left, 2=right")
    parser.add_argument("--inflate", type=float, default=0.0,
                        help="Inflate obstacle dimensions by this percentage (e.g. 10 = +10%%)")
    args = parser.parse_args()

    PRMGenerator(args.arm, args.inflate)
    print("\nTime Taken:", time.time() - start)