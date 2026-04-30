import SimpleFranka
import numpy as np
import random
import pickle
import RobotUtil as rt
import time
from scene_obstacles import get_obstacles_for_arm

random.seed(13)


# Desired EE rotation in SimpleFranka's FK frame (panda_link8).
#
# libfranka's O_T_EE = O_T_link8 @ T_hand_joint, where panda_hand_joint
# applies Rz(-pi/4) between link8 and the hand/EE frame.  So:
#   R_real = R_SimpleFranka @ Rz(-pi/4)
#
# For the real EE to point straight down (R_real = diag(1,-1,-1)), we need:
#   R_SimpleFranka = diag(1,-1,-1) @ Rz(+pi/4)
#
# This R_DESIRED is what SimpleFranka.ForwardKin should report for all
# PRM nodes; the real robot's O_T_EE will then equal diag(1,-1,-1).
_c45 = np.cos(np.pi / 4)
_s45 = np.sin(np.pi / 4)
R_DESIRED = np.array([
    [ 1,  0,  0],
    [ 0, -1,  0],
    [ 0,  0, -1],
], dtype=float) @ np.array([
    [ _c45, -_s45, 0],
    [ _s45,  _c45, 0],
    [    0,     0, 1],
], dtype=float)

def project_to_vertical_down(q, lam=0.01, max_iter=200, r_eps=1e-3):
    """
    Project joint angles q so the gripper points straight down using
    Damped Least Squares (DLS) pseudo-inverse on the orientation rows of J.

    Only the rotational error (rows 3-5 of J) is used — translation is
    unconstrained so the XY position drifts freely while satisfying the
    orientation constraint.

    Args:
        q      : 7-DOF joint angle array
        lam    : DLS damping factor
        max_iter: maximum iterations
        r_eps  : convergence threshold on orientation error norm

    Returns:
        q_proj : projected joint angles (may violate joint limits; caller checks)
    """
    q = np.array(q, dtype=float)
    qmin = np.array([-1.57, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    qmax = np.array([ 1.57,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

    for _ in range(max_iter):
        Tcurr, J = mybot.ForwardKin(q)
        R_curr = np.array(Tcurr[-1][:3, :3])

        # Orientation error via axis-angle: R_err = R_des @ R_curr^T
        R_err = R_DESIRED @ R_curr.T
        axis, ang = rt.R2axisang(R_err)
        r_err = np.array(axis) * ang  # 3-vector

        if np.linalg.norm(r_err) < r_eps:
            break

        # Clamp step size to avoid large jumps
        if abs(ang) > 0.1:
            r_err = np.array(axis) * 0.1

        # Orientation rows of the Jacobian (rows 3-5)
        Jo = J[3:6, :]  # 3×7

        # DLS pseudo-inverse: J^† = J^T (J J^T + λ² I)^{-1}
        A = Jo @ Jo.T + (lam ** 2) * np.eye(3)
        J_dls = Jo.T @ np.linalg.inv(A)  # 7×3

        dq = J_dls @ r_err
        q = q + dq
        q = np.clip(q, qmin, qmax)

    return q

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


def PRMGenerator(arm_number, inflation_pct=0.0, out_path=None, progress_cb=None):
    global mybot
    mybot = SimpleFranka.SimpleFrankArm(arm_number=arm_number)

    prmVertices = []
    prmEdges    = []

    pointsObs, axesObs = build_obstacles(arm_number, inflation_pct)
    pointsObs = np.array(pointsObs)
    axesObs   = np.array(axesObs)

    while len(prmVertices) < 10000:
        qmin = [-2.09, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973] # Joint 1 has been restricted to -120 degrees as the arm is not expected to reach behind
        qmax = [ 2.09,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
        q_new = np.random.uniform(low=qmin, high=qmax)
        # Project random sample onto the gripper-vertical-down constraint manifold
        q_new = project_to_vertical_down(q_new)
        Tcurr, _ = mybot.ForwardKin(q_new)
        ee_x = Tcurr[-1][0, 3]
        ee_z = Tcurr[-1][2, 3]
        if ee_x < 0 or ee_z < 0 or ee_z > 0.7:
            continue
        # Verify orientation constraint was satisfied (discard if DLS didn't converge)
        R_curr = np.array(Tcurr[-1][:3, :3])
        R_err = R_DESIRED @ R_curr.T
        _, ang = rt.R2axisang(R_err)
        if abs(ang) > 0.05:  # ~3 degrees tolerance
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

            if progress_cb:
                progress_cb(new_idx)
            else:
                pct = new_idx / 10000
                bar = int(pct * 40)
                print(f"\r  [{'█'*bar}{'░'*(40-bar)}] {new_idx}/10000", end="", flush=True)

            for idx in knn:
                q_near = prmVertices[idx]
                if not mybot.DetectCollisionEdge(q_new, q_near, pointsObs, axesObs):
                    prmEdges[new_idx].append(idx)
                    prmEdges[idx].append(new_idx)

    # Save the PRM — filename encodes which arm it was built for
    fname = out_path if out_path else f"prm_files_10k/myPRM_arm{arm_number}_none.p"
    with open(fname, 'wb') as f:
        pickle.dump(prmVertices, f)
        pickle.dump(prmEdges, f)
        pickle.dump(pointsObs, f)
        pickle.dump(axesObs, f)
    print(f"\nSaved {len(prmVertices)} vertices to {fname}")
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