# DexNet 4.0 suction grasping

Runtime-switchable alternative to the classical RANSAC/PCA grasp solver on the
pick-and-place path. `FC-GQCNN-4.0-SUCTION`, GPU, ~44 ms warm inference.

## Why a separate container

Stock TF 1.15 targets CUDA 10.0 and has no Ampere (SM 8.6) kernels, so it cannot use
the RTX 3060 Ti. `nvcr.io/nvidia/tensorflow:23.03-tf1-py3` is NVIDIA's TF 1.15.5 fork
built against CUDA 12.1 — the last `-tf1-` image they published. It is Ubuntu 20.04 /
Python 3.8, matching ROS Noetic on the host, so `rospy` and the generated messages
bind-mount straight in.

## Setup

```bash
git submodule update --init --recursive
bash src/devel_packages/orio_perception/scripts/download_dexnet_model.sh
catkin_make                                    # generates custom_msgs/PlanDexnetGrasp
docker build -f src/devel_packages/orio_bringup/docker/Dockerfile.dexnet \
             -t orio/dexnet:latest src/devel_packages
```

Berkeley's Box download links are all dead (404; upstream issues #133/#141). The script
pulls from a HuggingFace mirror of their `model_zoo.zip` and verifies the SHA256.

## Running

```bash
bash launch_demo.sh --dexnet          # adds a DexNet pane to the tmux session
```

or standalone:

```bash
bash src/devel_packages/orio_bringup/docker/run_dexnet.sh   # DEXNET_MIN_Q_VALUE=0.30
docker logs -f orio_dexnet
```

A ROS master must be up **before** the container starts.

## Switching backends

`orio_perception/config/grasp.yaml` — `grasp_backend: dexnet | classical`
(defaults to `classical`, so nothing changes until switched).

`orio/config/motion.yaml` — `use_grasp_orientation` makes the robot honour the grasp
orientation instead of always approaching straight down. Enable it together with
`grasp_backend: dexnet`.

Both must be set for the full DexNet behaviour: perception plans the tilted grasp,
motion executes it.

## First bringup

1. `max_tilt_deg: 0.0` in **both** configs — vertical grasps only. Confirms the motion
   refactor changed nothing.
2. Raise to `45.0` once parity is confirmed.
3. Test an **empty bin**: with no semantic segmentation, only `min_q_value` prevents a
   confident grasp on the bin floor.
4. Tune `bin_depth_min` / `bin_depth_max` against a loaded bin.

## ** Before trusting any A/B result **

Resolve the XY offsets. `dexnet.offset_x/offset_y` ship as `0.0`, which is a safe
dry-run default but probably wrong for picking.

The classical path's `0.015 / -0.05` is a hand-tuned lateral calibration residual — not
a two-cup correction (`grasp_solver` already returns the cup midpoint) and not tool
length (already in the URDF: `panda_joint8` z=0.17, edited from the stock 0.107).
Since the residual belongs to the camera calibration and gripper rather than the
algorithm, **DexNet probably needs the same values**.

Check with a known object at a known world position:
- both backends off by the same XY -> copy `0.015 / -0.05` to `dexnet`
- only classical off -> solver bias; keep DexNet at 0
- different errors -> tune DexNet's independently

A systematic XY error would read as "DexNet picks badly" when it is a coordinate
convention issue.

## A/B data

Every attempt from either backend appends a row to `grasp_attempts.csv` (override with
`~ab_log`):

```
stamp,backend,planned,q_value,tilt_deg,x,y,z,plan_time_s,reason
```

Rejections are logged with `planned=0` and the reason, so tilt rejections and
low-confidence rejections stay distinguishable. Success rate is counted separately —
this log records what was *planned*, not whether the lift held.

## Notes

- DexNet plans from depth alone. `handle_pnp` branches **before** GroundingDINO/SAM,
  which still serve the labelling path.
- Depth must reach the planner as `32FC1` in metres; the orchestrator converts from the
  Xtion's `16UC1` millimetres.
- Inpainting is mandatory — skipping it collapses `q_value` from ~0.94 to ~0.0002.
- The interface is `custom_msgs/PlanDexnetGrasp`, not gqcnn's own srv: importing
  `gqcnn.srv` pulls in TensorFlow, which host-side clients do not have.
- The container carries a patch for a gqcnn bug where the inference batch size stays at
  the model's training value of 64, computing 64x the work for one image (~11 s vs
  ~0.4 s). The node logs `Inference batch size: 1` on startup; a warning there means
  the patch did not take.
