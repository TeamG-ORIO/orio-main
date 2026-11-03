# Labelling arm test

Run the labelling arm (robot 2, iam-luisa) on its own, with the label height measured by
**Fast-FoundationStereo** on the ZED instead of carried over from the pick-and-place arm.

In the demo, `/compute_grasps_labelling` finds the label spot in the ZED image but not its
depth: the ZED's own depth was too noisy, so the state machine reuses the height robot 1
measured when it picked the item from the bin. That means the labelling arm cannot run
without robot 1. Here the depth comes from Fast-FoundationStereo on the ZED's rectified
stereo pair (the experiment in `/home/orio/newest/zed/`, see `ffs/reference/`), so one arm
and one camera are enough.

The full demo now uses this depth too (`run_full_demo.sh`): the Fast-FoundationStereo
node moved to `orio_perception/scripts/ffs_depth_node.py` (started by
`orio_bringup/docker/run_ffs.sh`, image from `orio_bringup/docker/Dockerfile.ffs`), and
both share its `orio_ffs` container. The test keeps its own perception container
(`orio_lbl_perception`), ROS names (`/labelling_test/*`) and logs
(`logging/labelling_test/`). It imports the demo's label placement, prompt, crop and
camera TF rather than copying them, so it stays in step with the demo.

## Run

```bash
bash research/labelling_arm_test/run_labelling_test.sh              # dry run: no vacuum, Enter before each placement
bash research/labelling_arm_test/run_labelling_test.sh --vacuum     # real suction on the labelling cup
bash research/labelling_arm_test/run_labelling_test.sh --no-motion  # perception + IK only, no robot
bash research/labelling_arm_test/stop_labelling_test.sh             # everything off
```

| Option | Description |
| --- | --- |
| `--vacuum` / `--no-vacuum` | Real suction on the labelling cup / skip the cup services (default). |
| `--confirm` / `--auto` | Ask before each placement (default) / no prompt. |
| `--zone 1\|2\|auto` | Which label zone (default `auto`: zone 1 if it has a pose, else 2). |
| `--skip-dispenser` | Do not fetch a label first; place with what is on the cup. |
| `--cycles N` | Stop after N labels (default: until `q`). |
| `--z-offset M` | Metres added to the measured label z (e.g. `-0.005` presses 5 mm less). |
| `--depth ffs\|zed` | Depth source: Fast-FoundationStereo (default) or the ZED's own depth, for comparison. |
| `--prompt TEXT` | GroundingDINO prompt (default: the demo's `bright box on black background`). A pale flat packet scores ~0.19 on that and is dropped; `packet` scores it ~0.44. |
| `--box-threshold Q` | Lowest detection score kept (default: the demo's 0.25). |
| `--no-motion` | Perception and IK only: prints the pose and joints; no control PC, no arm. |
| `--unlock` | Open robot 2's brakes first (`lock_arms.py --unlock 2`). |
| `--fresh` | Stop the running stack first. |

At the prompt: **Enter** places the label, **r** computes the pose again, **s** skips this
cycle, **q** stops. Ctrl-C stops the run; the stack stays up for the next one.

One cycle is the state machine's two labelling states: **GET_LABEL** (dispenser pre →
contact, cup on, back, `LABELLING_SAFE`) then **APPLY** (pre → label pose, cup off, back,
`LABELLING_SAFE`), with the label pose from this directory's perception instead of a
zone's carried-over depth. In a dry run put a label on the cup by hand to see a placement.

Before the arm moves, the pose must be inside the zone box the state machine uses
(x 0.2–0.7, |y| ≤ 0.6) and its z inside −0.01–0.40 m (items lie on the table, which is at
z ≈ 0, so a flat pouch comes out near 0 and a box at its height); otherwise that zone is
refused with the reason, and `--zone auto` tries the other one. The joints are
collision-checked against the arm-2 obstacle file and FK-validated, as in
`compute_pick_joints`, with two differences: the cup's collision block may meet the table
model at the final pose (the block reaches ~1 cm past the cup, so a flat item at z ≈ 0.005
would otherwise always count as a table collision), and solutions that reach the zone from
behind (base or upper arm turned past 2 rad) are refused — `compute_pick_joints` re-seeds
over the whole joint range and can pick one, which swings the arm out over the dispenser
side. Retries here re-seed near the nominal configuration instead.

A label is the same rectangle turned by 180° about the cup, so the arm solves IK for the
published orientation and for it turned by 180° about the tool axis, and takes the one
with the smaller joint motion from where the arm is (the log shows both, with joint 7's
turn). Perception also normalises the label angle to (−90°, 90°], since the placement
code's `angle % 180` jumps between ~0° and ~180° for a nearly axis-aligned item.

The dispenser is visited only when the cup has no label. A placement the robot could not
finish (user stop, controller fault) ends the run; any other failure returns to the prompt.

## What runs

```
ZED (zed_only.launch, in orio_docker_container)
  left/right rect images + camera_info
        │
        ▼
orio_perception/scripts/ffs_depth_node.py   container orio_ffs (image orio/ffs-ros)
  /ffs/compute  → /ffs/depth (32FC1 m), /ffs/left, /ffs/camera_info, /ffs/disp_vis
        │
        ▼
perception/labelling_perception_node.py   container orio_lbl_perception (image orio/perception)
  GroundingDINO + SAM + opt_label_loc on the left frame, depth at the chosen pixel,
  back-projection with camera_info, zed_to_label_tf.yaml → world pose
  /labelling_test/compute_label_pose → /labelling_test/label_pose_z1, _z2 (PoseArray, panda_link0)
        │
        ▼
arm/labelling_arm_test.py    in orio_docker_container (frankapy, robot 2)
  dispenser → label pose → LABELLING_SAFE; /orio/lbl_cup/on|off with --vacuum
```

`run_labelling_test.sh` brings these up in one terminal (roscore, robot 2's control PC,
the main container, ZED, Fast-FoundationStereo, perception, pneumatics with `--vacuum`) and
runs the arm script in the foreground. Everything but the arm script and pneumatics stays
up between runs. Output is merged under `[module]` prefixes and saved to
`logging/labelling_test/<stamp>.log`; the services' own logs are in
`logging/labelling_test/stack/`.

### Depth

Fast-FoundationStereo runs on the ZED pair at half resolution (`ORIO_FFS_SCALE=0.5`; the
authors' operating point is width < 1000 px, ~40 ms a frame on the 4090) and the depth is
resized back to full resolution, so a pixel of the left image indexes it directly. The focal
length comes from the left `camera_info` and the baseline from the right one, so it follows
the wrapper's resolution (HD1080 in `zedm.yaml`). Depth is computed on request only
(`ORIO_FFS_RATE=0`); set a rate to watch `/ffs/depth` or `/ffs/disp_vis` live.

The perception node reads the median depth in a 7×7 window around the chosen pixel and
back-projects with the `camera_info` intrinsics — not the hard-coded `LBL_FX`/`LBL_CX` in
the demo node, which are for a different resolution than the wrapper publishes. Both are
printed, so they can be compared. `--depth zed` runs the same placement on the ZED's own
`depth_registered` for a side-by-side.

### Images

Each perception call saves to `logging/labelling_test/<stamp>.images/`:
`label_HHMMSS.png` (colour with masks, boxes, the chosen points and their depths, next to
the depth map), `label_HHMMSS_color.png` and `label_HHMMSS_depth.npy` (the raw inputs, to
rerun offline).

To save a frame without running perception, use `snap_zed.sh` while the stack is up:

```bash
bash research/labelling_arm_test/snap_zed.sh [outdir]    # default logging/zed/<timestamp>/
```

It writes `color.png`, `right.png`, `preview.png` (colour beside depth), `color.npy`,
`depth.npy` (metres, NaN where invalid), `K.npy` and `meta.json` (the ZED's exposure, gain
and white balance at the time, and the FFS summary).

## Setup (once per machine)

1. **Fast-FoundationStereo checkout and weights.** The node mounts
   `ORIO_FFS_DIR` (default `/home/orio/newest/zed/fast-foundationstereo`, the team's
   checkout) and loads `ORIO_FFS_CHECKPOINT` (default its
   `weights/23-36-37/model_best_bp2_serialize.pth`, the HF `nvidia/c-fast-foundationstereo`
   release). On a new machine: `git clone https://github.com/nvlabs/fast-foundationstereo.git`
   and fetch the weights as `ffs/reference/ORIO_QUICKSTART.md` describes.
2. **The `ffs` image** from that repo's `docker/dockerfile`, with the xformers pin in
   `ffs/reference/dockerfile_xformers_pin.patch` (without it torch 2.14 is pulled in and
   torchvision breaks). Already built here.
3. **This test's image**, which adds rospy to it:
   ```bash
   docker build -t orio/ffs-ros:latest - < src/devel_packages/orio_bringup/docker/Dockerfile.ffs
   ```
4. The demo's `orio_docker`, `orio/perception` images and the perception weights
   (`orio_perception/{weights,SAM_weights}`), as for `run_full_demo.sh`.

Env: `ORIO_LUISA_HOST` (`snaak@iam-luisa`), `ORIO_FFS_DIR`, `ORIO_FFS_CHECKPOINT`,
`ORIO_FFS_SCALE`, `ORIO_FFS_ITERS`, `ORIO_FFS_RATE`, `ORIO_VACUUM_PORT` (`/dev/ttyACM0`).

## Files

| Path | What |
| --- | --- |
| `run_labelling_test.sh`, `stop_labelling_test.sh` | Launcher and teardown. |
| `arm/labelling_arm_test.py` | The arm cycle (robot 2), runs in `orio_docker_container`. |
| `perception/labelling_perception_node.py`, `run_lbl_perception.sh` | Label placement with measured depth, in the perception container. |
| `orio_perception/scripts/ffs_depth_node.py`, `orio_bringup/docker/run_ffs.sh`, `Dockerfile.ffs` (outside this directory) | Fast-FoundationStereo as a ROS node, in its own container. |
| `ffs/reference/` | The original experiment: `ffs_orio_infer.py` (offline inference this follows), `ORIO_QUICKSTART.md` (setup, measured speed, gotchas), `assets_orio_K.txt`, the dockerfile patch. |
