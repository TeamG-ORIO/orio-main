# Dex-Net suction grasping: status and handbook

Written for an engineer or agent picking up the Dex-Net integration cold. It records what
exists, how the pieces fit, what has been measured, what is still open, and the traps that
cost time. Last updated 2026-10-05 on branch `feature/dexnet`. Dates and commit hashes are
from `git log`; numbers are from `logging/dexnet_pnp/*.log`.

Related docs: `COMMANDS.md` (launcher flags), `docs/LOGGING.md` (run recording and the
affordance panels), `docs/TROUBLESHOOTING.md` and `docs/OPEN_ISSUES.md` (hardware
failures), `src/devel_packages/orio_perception/test/README.md` (offline tests).

---

## 1. One-paragraph status

Dex-Net 4.0's fully-convolutional suction network (FC-GQCNN-4.0-SUCTION) runs as a ROS
service in its own GPU container and is wired end to end: Xtion RGB-D in, one suction pose
out, executed by the robot-1 Franka through `dexnet_pnp.py`. Single-item picks from a bin
work on hardware with real vacuum. The pipeline is tested, logged, and has a per-grasp
diagnostic figure. The two things not finished are (a) measuring and applying the
constant 1 to 2 cm landing offset that comes from the camera calibration, for which the
plumbing now exists but the value has not been measured, and (b) any evaluation on
cluttered piles beyond ad-hoc runs. The full dual-arm demo (`state_machine.py`) still uses
the classical grasp solver; Dex-Net is only exercised by the single-arm test loop.

---

## 2. What Dex-Net is doing here

- **Model:** FC-GQCNN-4.0-SUCTION from Berkeley's gqcnn, vendored as the submodule
  `src/devel_packages/gqcnn` (upstream `499a609`). Weights are not in git; fetch them with
  `src/devel_packages/orio_perception/scripts/download_dexnet_model.sh` (HuggingFace mirror,
  SHA-256 checked, Berkeley's own links are dead). They land in `gqcnn/models/`.
- **Input:** a depth image plus a binary segmask of "things you may grasp", with camera
  intrinsics. Colour is passed but only used for the diagnostic figure.
- **Output:** a dense per-pixel suction quality map, argmaxed to one grasp: a pixel, a
  depth, a surface normal and a q-value in 0 to 1. No parallel-jaw mode is used.
- **Suction only, single arm.** Robot 1 (control PC `iam-doc`) with the pneumatic cup.
  Robot 2 and labelling are not involved.

---

## 3. Architecture and data flow

```
Xtion (openni2, depth_registration=true)
   /camera/rgb/image_raw, /camera/depth_registered/image_raw, /camera/rgb/camera_info
        │
        ▼
perception node  (orio_perception/scripts/perception_control_combined_pass_through.py)
   container orio_perception, ROS1 Noetic, launched by docker/run_perception.sh dexnet
   /compute_grasps (Trigger)  →  crop → depth-band segmask → service call → world pose
        │  custom_msgs/PlanDexnetGrasp (color, depth 32FC1 m, CameraInfo, segmask)
        ▼
Dex-Net planner  (orio_bringup/docker/dexnet_grasp_planner.py)
   container orio_dexnet (NVIDIA TF1 image, Dockerfile.dexnet), docker/run_dexnet.sh
   /dexnet_grasp_planner/plan_grasp  →  rescale 0.5 → inpaint → FC-GQCNN → best grasp
   optional: six-panel affordance PNG per call
        │  DexnetGrasp: camera-frame pose, q_value, center_px, depth
        ▼
perception node: camera → world via manipulation/config/realsense_tf.yaml, tilt gate,
   publishes PoseArray on /grasp_poses (length 1)
        │
        ▼
pick loop  (orio/dexnet_pnp.py, container orio_docker_container, foreground)
   + optional world XY offset → bounds check → IK (ikpy, panda_arm_hand.urdf)
   → FrankaArm.goto_joints: pre-pick, contact, vacuum on, retract, transit, drop, home
```

Everything above is started by `run_dexnet_pnp_single.sh` (one terminal) or the tmux
layout `orio_bringup/tmux/layouts/orio_dexnet_pnp.session.sh` via `launch_demo.sh`. The
single-terminal script is the one that has been used for all hardware runs and is the one
kept current.

### File map

| Role | File |
| --- | --- |
| Planner service (runs in the TF1 container) | `src/devel_packages/orio_bringup/docker/dexnet_grasp_planner.py` |
| Planner container image / start | `docker/Dockerfile.dexnet`, `docker/run_dexnet.sh` (`run_dexnet_local.sh` is a historical variant, superseded) |
| Service and message contract | `src/devel_packages/custom_msgs/srv/PlanDexnetGrasp.srv`, `msg/DexnetGrasp.msg` |
| Perception node, Dex-Net branch | `perception_control_combined_pass_through.py`: `_plan_grasp_dexnet`, `_handle_pnp_dexnet`, `_build_bin_mask`, `_cropped_intrinsics`, `_resolve_pnp_intrinsics` |
| Perception config | `src/devel_packages/orio_perception/config/grasp.yaml` (loaded by `launch/perception.launch`) |
| Pick loop | `src/devel_packages/orio/dexnet_pnp.py` |
| Launcher | `run_dexnet_pnp_single.sh` (repo root) |
| Camera to robot transform | `src/devel_packages/manipulation/config/realsense_tf.yaml` (name is historical; it is the Xtion) |
| Offline tests and viewers | `src/devel_packages/orio_perception/test/` (see its README) |
| RViz grasp viewer, no arm | `docker/run_dexnet_viz.sh` + `test/visualize_dexnet_rviz.py` |

### Conventions that are easy to get wrong

- **Approach axis is the X column** of the rotation gqcnn returns, not Z. See
  `SuctionPoint2D.pose()` in gqcnn. The perception node converts this into a tool rotation
  whose Z is the approach direction (`_rotation_from_approach`), pointing down into the bin.
- **Camera to world** is a near 180° flip about X. Camera +x is world +x, camera +y is
  world −y, camera +z (depth) is world −z. Hence **image up is world +Y, image right is
  world +X**. Use this when reading the affordance figure against robot behaviour.
- **Crop:** the node crops the 640x480 frame to rows 40 to 405 and columns 115 to 515
  before anything else (`PNP_CROP_*`). Intrinsics are shifted into the crop. All pixel
  values in logs and figures are crop pixels unless stated.
- **Rescale:** the planner downsamples the crop by 0.5 (`DEXNET_RESCALE`) before inference
  and scales intrinsics to match. This lifted q from about 0.24 to about 0.9 at this camera
  height. `center_px` in the service reply is scaled back to crop pixels; the 3D pose needs
  no correction. The affordance figure is drawn in rescaled pixels (200x182).
- **Segmask** is a depth band only (`bin_depth_min` to `bin_depth_max`, 0.60 to 1.20 m),
  no semantic segmentation. GroundingDINO and SAM are not loaded on the Dex-Net backend.
- **Intrinsics** come from `/camera/rgb/camera_info` at node start (5 s wait), else the
  PrimeSense 525 defaults. The Xtion has no calibration file on this machine, so the driver
  publishes its generic defaults: fx = fy ≈ 570.3, cx = 319.5, cy = 239.5. This was
  confirmed by reprojecting a logged grasp pixel back to the logged world point.
- **Depth topic:** both `/camera/depth/image_raw` (16UC1 mm) and
  `/camera/depth_registered/image_raw` (32FC1 m) carry the same hardware-registered
  stream; the node converts either to metres. The working tree currently points at the
  registered one.
- **Thresholds and gates**, in order: planner rejects q below `min_q_value` (default 0.30);
  perception rejects tilt above `max_tilt_deg` (45°); the pick loop rejects tilt above its
  own `--max-tilt-deg` (45°) and positions outside `PICK_BOUNDS`.
- **End effector in Desk must be "Other"**, otherwise the Franka Hand model is used and the
  arm fights the cup's mass. `lock_arms.py --unlock` now enforces this before unlocking.

---

## 4. How to run it

Prerequisites: the images built (`docker/build.sh`), the model weights downloaded, the
`custom_msgs` package built (`catkin_make`), robot 1 unlocked, and the Xtion plus ZED
plugged in. The host is Ubuntu 22.04 with ROS 2 only; **every ROS 1 process runs in a
container**, there is no host `rospy`.

```bash
bash run_dexnet_pnp_single.sh                       # dry run, no vacuum, Enter per pick
bash run_dexnet_pnp_single.sh --vacuum              # real suction
bash run_dexnet_pnp_single.sh --vacuum --auto       # no per-pick prompt
bash run_dexnet_pnp_single.sh --vacuum --offset-y -0.015 --min-q 0.2
```

At the confirm prompt: Enter executes, `r` re-plans on a fresh frame, Ctrl-C aborts. After
five consecutive declines the loop parks the arm and waits for a refill.

Flags most relevant to Dex-Net work (full table in `COMMANDS.md`):

| Flag | Effect |
| --- | --- |
| `--min-q Q` | Planner's accept threshold. Env `DEXNET_MIN_Q_VALUE`. Default 0.30. |
| `--offset-x M`, `--offset-y M` | World-frame correction added in the pick loop after Dex-Net. Env `ORIO_PNP_OFFSET_X/Y`. Default 0. |
| `--straight-down` | Ignore the Dex-Net approach tilt, approach vertically. |
| `--no-affordance` | Skip the per-grasp six-panel figure (saves a second forward pass). |
| `--no-logging` | Skip the rerun recorder. |

Environment knobs without flags: `DEXNET_RESCALE` (0.5), and `bin_depth_min/max`,
`max_tilt_deg` in `grasp.yaml`.

Each run writes `logging/dexnet_pnp/<stamp>.log` (every container's output, tagged by
module), `<stamp>.affordance/afford_NNNN_hhmmss.png`, and when the recorder is available
`logging/rerun/<stamp>/`. The services that stay up between runs (see below) log to
`logging/dexnet_pnp/stack/<service>.log`, the control PC to `stack/control_pc.log`.

Ctrl-C stops only the run: the pick loop, pneumatics and recorder. roscore, the main
container, the control PC, the planner, both cameras and perception stay up, and the next
run reuses whatever is still healthy (the planner and perception are restarted when
`--planner`, `--min-q` or the panel setting changes). `bash stop_dexnet_pnp.sh` stops
everything; `--fresh` does the same before starting.

### Offline checks that need no robot

```bash
python3 src/devel_packages/orio_perception/test/test_grasp_geometry.py    # maths, no ROS
# with roscore + planner container up:
src/devel_packages/orio_perception/venv/bin/python \
    src/devel_packages/orio_perception/test/test_dexnet_service.py --json /tmp/r.json
```

The service test plans on Berkeley's five PhoXi sample scenes and checks the empty-bin and
tilt guards. `test/sample_results.json` is the reference; q-values should match to 1e-3.

---

## 5. What has been done, in order

| Date | Commit | What |
| --- | --- | --- |
| 2026-09-03 | `accc789` | Dex-Net suction backend: planner service, custom_msgs contract, perception branch, grasp.yaml. |
| 2026-09-10 | `ebbcfce`, `feba493` | Policy graph rebuilt for the incoming image size; offline tests and visualiser. |
| 2026-09-14 | `e975d27` | `capture_xtion.py` and `live_affordance.py` (first look at the affordance map on our camera). |
| 2026-09-21 | `4aaec7c` | roscore, perception and Dex-Net moved into ROS 1 containers; rospy baked into the TF1 image. |
| 2026-09-21 | `8f25c78` | Input rescale 0.5 (q from ~0.24 to ~0.9); unused models skipped on the Dex-Net backend. |
| 2026-09-21 | `09b652e`, `5bfa3e0` | `dexnet_pnp.py` single-arm pick loop; tmux layout and single-terminal launcher. |
| 2026-09-21 | `65b3f1f` | RViz viewer and sample results. |
| 2026-09-22 | `eafe57a`, `73a1dab` | Regenerate-grasp prompt; branch merged to main. |
| 2026-09-22 | `bbf473b` | Six-panel affordance figure per grasp, rendered inside the planner. |
| 2026-09-22 | `330a707`, `8998716` | Refill wait, run events, recorder sidecar, pneumatics in the container. |
| 2026-09-22 | `f877f8a` | End effector forced to "Other" before unlock. |
| 2026-09-22 | `0944227` | Docs for the above. |

Hardware runs: 73 logged sessions in `logging/dexnet_pnp/` between 2026-09-20 and
2026-09-22. The last two of that day (`20260922_190555`, `20260922_191625`) were saved by
the operator as `logging/dexnet_pnp/*.affordance_good` as reference examples of good
behaviour.

---

## 6. What has been measured

- **Planner latency:** 5 to 10 ms per call once warm, 0.3 to 0.5 s on the first call after
  a graph rebuild. The affordance figure adds a second forward pass plus ~0.3 s render.
- **q-values on our bin, rescale 0.5:** single isolated items on the black cloth score
  0.95 to 0.99. The run inspected in detail (`20260922_181407`, a small white box) scored
  0.38 to 0.47 with tilt 0 to 16°. Rejections seen at q 0.10 to 0.12. The 0.30 threshold
  has been fine for isolated items; it has not been tuned for piles.
- **Landing offset (found 2026-09-22, not yet corrected):** the cup consistently lands 1 to
  2 cm image-up (world +Y) of the grasp drawn in the figure. Verified by reprojecting the
  logged pixel for call 0002 of `20260922_181407` through the crop intrinsics and the
  camera TF: it reproduces the logged world point to the millimetre, so **the Dex-Net path
  is internally consistent and the error is in the calibration chain**, not the planner:
  - the hand-eye TF has hand-rounded translation (0.1667, −0.47, 0.97) and ~1° of roll,
    worth ~1.6 cm at 0.9 m;
  - the Xtion has no RGB calibration file, so the driver's generic intrinsics are used
    (principal point error of 5 to 10 px is ~1 to 1.6 cm here);
  - the URDF models the cup 6.3 cm along the flange axis with no lateral offset.
  The classical path has always carried this residual as `classical.offset_x/y`
  (0.015, −0.05). The Dex-Net path applied none. See section 7 for the fix path.

---

## 7. Open items

Ordered by how much they block Dex-Net work.

1. **Measure and set the landing offset.** The plumbing exists (`--offset-x/-y`, applied
   in `dexnet_pnp.request_grasp` after the raw pose, logged on every `[Grasp]` line). Do
   three to five picks at different bin positions, record world error, set the mean via
   `ORIO_PNP_OFFSET_X/Y`. Start near `--offset-y -0.015`. Keep `dexnet.offset_x/y` in
   `grasp.yaml` at 0 so the correction is not applied twice.
2. **Proper camera calibration** removes the residual for both backends and its position
   dependence: calibrate the Xtion RGB (writes
   `~/.ros/camera_info/rgb_PS1080_PrimeSense.yaml` inside the cameras container), then redo
   the hand-eye calibration (`manipulation/launch/xtion_calibration.launch`, easy_handeye
   with an ArUco marker).
3. **Pile performance is unmeasured.** The target use is grasping from cluttered piles. No
   structured evaluation exists: no success rate per item type, no comparison with the
   classical solver. `docs/LOGGING.md` describes the intended A/B method (`runs.py compare`)
   but it has not been run.
4. **Uncommitted working-tree changes** as of this writing, all from 2026-09-22 to
   2026-10-05 and all small, need review and a commit:
   `run_dexnet_pnp_single.sh` (`--offset-x/-y`, `--min-q`), `dexnet_pnp.py` (offset
   application), `grasp.yaml` (comments), `COMMANDS.md`, and the depth topic swap in
   `perception_control_combined_pass_through.py`. The GroundingDINO submodule also shows
   as modified (untracked content inside it); not Dex-Net related.
5. **`grasp.yaml` `dexnet.min_q_value` is dead.** The planner enforces the threshold from
   `DEXNET_MIN_Q_VALUE`; the perception node never reads that key. A comment now says so.
   Either remove the key or make `run_perception.sh` forward it.
6. **Perception's own image logging is off** on the Dex-Net test because `rerun` is broken
   on the perception image's Python 3.8. The host-side recorder sidecar and the affordance
   PNGs cover most of the need. See `docs/LOGGING.md`.
7. **Hardware flakiness, not Dex-Net specific but hits every run:** the Xtion often fails
   its IR stream on first start (launcher retries), and the `iam-doc` control PC has frozen
   in the past (probable cause fixed 2026-09-21, unconfirmed). Both in `docs/OPEN_ISSUES.md`.
8. **Dex-Net is not in the full demo.** `state_machine.py` and `pick-place-label.py` have
   the approach-axis waypoint logic ready but `grasp.yaml` still defaults to
   `grasp_backend: classical`; only the single-arm launcher forces `dexnet`.

---

## 8. Traps for the next person

- **Nothing ROS 1 runs on the host.** `import rospy` fails outside the containers. Run
  perception-side scripts with `docker exec orio_perception ...` or the perception venv for
  the few host-side tools that only need message decoding.
- **Do not import gqcnn outside the TF1 container.** It pulls TensorFlow 1. That is why the
  service uses `custom_msgs` instead of gqcnn's own srv.
- **The affordance tensor only exists inside the planner.** Anything that needs the dense
  quality map (figures, heat-map analysis) must live in `dexnet_grasp_planner.py`, not in
  perception. The service reply carries only the chosen grasp.
- **The figure is ground truth for "where Dex-Net thinks the grasp is."** If the arm lands
  elsewhere, suspect calibration, the TF yaml or the tool, not the planner. The chain from
  figure pixel to world point was verified exactly (section 6).
- **Two `offset_x/y` knobs exist.** `grasp.yaml` `dexnet.*` (perception node, applied to the
  published pose) and the launcher's `--offset-*` (pick loop, applied before IK). Use one.
  The launcher's is the intended one because it leaves the published pose raw for logging.
- **The FC-GQCNN batch-size patch** at the top of the planner is load-bearing: without it
  every call computes 64 images and discards 63.
- **Inpainting and the valid-pixel mask are mandatory** before inference; without them q
  collapses to ~0.
- **`realsense_tf.yaml` is the Xtion transform** despite the name.
- **Classic failure signatures:** `No module named 'rospy'` in the planner means a stale
  image, rebuild from `Dockerfile.dexnet`; `/compute_grasps` never appearing means
  perception could not reach the planner service within 30 s.

---

## 9. Suggested next steps, in order

1. Commit the working tree (section 7 item 4).
2. Run the offset measurement and set `ORIO_PNP_OFFSET_X/Y` (item 1). Re-check the figure
   against where the cup lands on three items at different bin positions.
3. Build a pile test: fixed set of items, N picks, record success/failure, q, tilt, and the
   affordance figure per pick. The `grasp accepted` / `pick placed` events in `run.json`
   already give most of the numbers.
4. Tune `--min-q` and `bin_depth_min/max` against that test, then decide whether
   `grasp.yaml` should default to `dexnet` for the full demo.
5. Calibrate the camera properly (item 2) once the above shows the residual is the limiting
   factor rather than grasp selection.
