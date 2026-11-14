# Offline SLAM tests: record once, run ORB-SLAM3 and blur checks on the bag

> Moved here from `~/orio/SLAM_TESTS_README.md` on 2026-11-13. `scripts/`, `launch/` and `config/nav/` are relative to this
> package (`orio_stretch`); `datasets/`, `maps/`, `viewer/` and `config/waypoints_*`, `config/truth_*` are relative
> to the data root `$ORIO_DATA` (default `~/orio`). See `../README.md` for the layout and the demo run order.
> `bag_sharpness.py`, `nav_plot.py` and `orbslam_viewer.py` (with `viewer/map_viewer.html`) are not in this package:
> where this file says `scripts/<one of those>`, run `~/orio/scripts/<it>`.

Written 2026-10-05. Robot `stretch-re1-1076`, ROS 1 Noetic. Recording basics, bag topics and TF rules
are in `RECORDING.md`; this file only covers what was added for the ORB-SLAM3 / exposure / IR tests.

## The three commands

```bash
# 1. at the robot: record one run (you drive with the keyboard teleop, Ctrl-C once to finish)
scripts/slam_record.sh lab_loop_01

# 2. anywhere, no robot needed: how blurry was it?
scripts/bag_sharpness.py lab_loop_01

# 3. no robot needed: run ORB-SLAM3 in every mode and compare
scripts/orbslam_run.sh lab_loop_01 all
```

Each script prints its options when run without arguments.

## What one recording contains

`slam_record.sh` = `record.launch infra:=true` at 30 fps + `cam_setup.py` + teleop. On top of the usual
bag (colour, aligned depth, native depth, IMU, lidar, wheel odometry, TF) it records

* `/camera/infra1|infra2/image_rect_raw` + `camera_info`: the two global-shutter IR cameras (left, right)
* `/camera/{infra1,infra2,depth,color}/metadata`: per-frame exposure, gain and projector state
* `/slam_test/config`: the camera settings that were applied (also in `meta.txt`)

The IR projector runs in **alternate** mode by default: it toggles every frame, so half the frames
(15 Hz) have clean IR images and the other half (15 Hz) have normal projector depth. That is what lets
one bag serve IR stereo *and* RGB-D. The offline scripts pick the right half by the metadata.
`--emitter on` or `--emitter off` records a single kind at the full 30 Hz.

Size: about 3 GB per minute while driving (the first real run was 14 GB for 4:49).

**What cannot be changed after recording: exposure and gain.** Compare them by recording the same
route once per setting:

```bash
scripts/slam_record.sh lab_auto
scripts/slam_record.sh lab_short --color-exposure 50 --color-gain 128 --ir-exposure 3000 --ir-gain 100
scripts/bag_sharpness.py lab_auto lab_short        # side-by-side table
```

Units: colour exposure in 100 us (50 = 5 ms, gain 0..128); IR exposure in microseconds (3000 = 3 ms,
gain 16..248).

## Reference points (ground truth for T4 / T5)

Tape crosses on the floor, number them 1..9, and measure the distances between them with the tape measure
(not every pair is needed; three or more per point is plenty). Put the measurements in a text file, one per
line: `point_a point_b distance_m`.

While recording with `slam_record.sh`, park the robot on a cross (same spot of the robot over the cross
every time: the middle of the wheel axle is `base_link`), wait a second, and press that point's number key.
The key stops the base and writes the moment into the bag (`/slam_test/mark`). Mark the start point when
you leave and again when you are back on it: the difference is the drift over the loop.

```bash
scripts/marks_report.py b512_map --truth config/truth_b512.txt        # wheel odometry against the tape
scripts/marks_report.py b512_loc1 --reference b512_map                # same points, another run
scripts/marks_report.py b512_loc1 --traj poses.tum --reference b512_map --reference-traj map_poses.tum --same-frame
```
The last form is the T5 score for a localiser: `--traj` takes any method's robot poses (TUM format, `base_link`
in the saved map's frame) and `--same-frame` compares positions directly, pass/fail at 0.25 m.

Suggested session (one visit to the robot):
`b512_map` slow, complete loop past all points -> `b512_loc1`, `b512_loc2`, `b512_loc3` normal driving,
stopping on every point. Every mapping and localisation method is then run offline on these four bags.

## ORB-SLAM3 modes (`orbslam_run.sh <run> [mode]`, default `ir-rgbd`)

Decision 2026-10-05: **`ir-rgbd` is the default.** On `basement_map` (two laps of a taped 5.4 x 1.7 m
rectangle) it tracked 100 % at full speed, closed the loop, and drifted 4-20 cm lap to lap at the taped points
(wheel odometry: 26-81 cm). `ir-stereo` was equivalent; colour `rgbd` is jittery; both inertial modes fail on
slow planar driving. Cold-start localisation against a saved map (`--save-map` on lap 1, `--load-map` from
three different start points on lap 2) merged every time and scored 4-20 cm, i.e. passes T5's 25 cm. See
`maps/logs/coldstart_basement_map.sh` for the exact commands.


| mode | images | shutter | notes |
|---|---|---|---|
| `rgbd` | colour + aligned depth (projector-on frames) | rolling | same sensors RTAB-Map uses today |
| `rgbd-inertial` | + `/camera/imu` | rolling | |
| `ir-stereo` | left + right IR (projector-off frames) | global | no depth image needed |
| `ir-stereo-inertial` | + `/camera/imu` | global | |
| `ir-rgbd` | left IR + native depth (projector-off frames) | global | depth is passive stereo in these frames |

Results land in `datasets/<run>/orbslam/<mode>/`; `summary.txt` explains itself, the last line is the
one-line `RESULT`. The settings file is generated from the bag (intrinsics, IR baseline, camera-IMU
extrinsics, frame rate), so there is no calibration file to maintain.

Useful variations:

```bash
# more / weaker features for blurry frames, kept next to the default result
scripts/orbslam_run.sh lab_loop_01 ir-stereo --tag lowfast --set ORBextractor.iniThFAST=12 --set ORBextractor.minThFAST=4

# map on one run, reuse the map on another (same mode)
scripts/orbslam_run.sh lab_map_slow ir-stereo --save-map lab
scripts/orbslam_run.sh lab_shaky    ir-stereo --load-map lab
```

RTAB-Map on the same bag: the replay recipe in `RECORDING.md` section 6 still applies (the extra IR
topics are simply ignored). With an `alternate` bag every second depth frame is passive stereo; record
with `--emitter on` if the run is meant for RTAB-Map only.

## Demo map viewer

`scripts/orbslam_viewer.py <run> <mode> [--truth FILE]` packs a run's coloured dense cloud, the odometry
version, the camera path, the taped points and the headline numbers into `viewer/<run>_<mode>/`;
`viewer/map_viewer.html` is the browser page (published 2026-10-06 for basement_map / ir-rgbd as
https://claude.ai/artifact/DDiFmp12df4dJG4Vtoof9V). The dense map now carries camera colour
(`orbslam_dense.py`, needs the colour stream in the bag).

## Results pages

* `~/orio/viewer/results.html` - mapping / localisation / navigation numbers for the basement course (published 2026-10-06).
* `scripts/nav_plot.py <nav csv ...> --map <dense_2d_orbslam.yaml> --waypoints <yaml>` - navigation runs drawn on the
  2D map; `nav_waypoints.py` now also logs the driven path (`<log>_path.csv`) so later runs show the trajectory.

## New site in one command

`scripts/site_build.sh <run> <map_name> [--truth FILE]` turns one recording (mapping lap with point 1 marked at
the start and again on return, then a localisation lap) into the saved map, live settings, map frame, 2D floor
plan, waypoint file and a cold-start localisation check, and prints the navigation command. Verified on the
basement recording 2026-10-25 (same numbers as the hand-run pipeline). Site truth files: `config/truth_<site>.txt`.

## Navigation with the IR camera as the localiser (T6 setup)

**Status 2026-10-06: drove the taped course live, camera only (no lidar).** 1->2->3->4->5->1, all six waypoints
reached, 6-12 cm from the goals by its own estimate, 12-30 s per leg. Two fixes came out of the first live try and are
in the launch: global planner = navfn (global_planner failed its path extraction on free ground) and the depth-cloud
staleness limit = 3 s (1 s blocked the base while ORB-SLAM3 loaded the CPU). The robot must move ~1-2 m before it
localises: run the keyboard teleop in a second terminal until the script prints LOCALISED.

Pieces:
* `launch/nav_orbslam.launch` - driver + lidar + D435i (IR, projector alternating) -> `orb_feed.py` -> ORB-SLAM3
  `ros_rgbd` with the saved atlas (`maps/orbslam/<map>.osa` + `<map>.live.yaml` from
  `orbslam_settings.py --load-atlas`) -> `scripts/orbslam_tf_bridge.py` (map->odom, like AMCL; holds the last fix
  when lost; ignores poses until ORB-SLAM3 has merged into the saved map) -> map_server with the run's
  `dense_2d_orbslam.yaml` -> move_base (static map + lidar layer + inflation) -> RViz.
* `scripts/nav_orbslam_start.sh <map> [--waypoints FILE --pause]` - the live bring-up: preflight, launch, camera
  settings, pose, waits for `/orbslam/localized`, then drives the waypoints or leaves RViz "2D Nav Goal".
* `scripts/nav_points.py FILE --from-run RUN --orbslam ir-rgbd_lap1 | --add NAME X Y YAW_DEG | --capture NAME`
  - waypoint files in the map frame (`config/waypoints_basement.yaml` = the five taped points).
* `scripts/nav_waypoints.py FILE [--pause]` - drives the list with move_base, logs the localiser's own
  arrival error per waypoint (the tape still measures the real one) to `datasets/nav_<date>.csv`.
* `scripts/nav_orbslam_selftest.sh [run] [start_s] [map] [run_dir] [waypoints] [start_at]` - any site; `start_at` seeds
  the start like `--start-at`; also reports live-costmap obstacles on floor the saved map has as free. Replays a recorded lap through the same nodes without
  hardware and checks map->base_link at every taped point against the waypoint file.

Frames: the saved atlas lives in ORB-SLAM3's camera-oriented world; `frame_upright.json` (written by
`marks_report.py --orbslam`) is the fixed rotation+shift to the upright map frame that the 2D map and the
waypoints use. Both come from the SAME run folder (`ir-rgbd_lap1`), which is why the launch takes `run_dir`.

## Where things are

* `~/orbslam_ws/` - separate catkin workspace: `src/orb_slam3_ros` (github.com/thien94/orb_slam3_ros,
  commit 5d4d506, with two small local patches marked `orio patch` in `src/custom_common.cc`: a
  `~tracking_state` topic, and a guard against a crash when saving an empty trajectory) and
  `deps/` (Pangolin 0.6 built into `deps/install`, no sudo used).
  Rebuild: `cd ~/orbslam_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$HOME/orbslam_ws/deps/install;/opt/ros/noetic"`
* `scripts/slam_record.sh`, `cam_setup.py` - recording
* `scripts/orbslam_run.sh`, `orbslam_settings.py`, `orb_feed.py`, `orbslam_log.py`, `orbslam_eval.py`, `slam_bag.py` - offline ORB-SLAM3
* `scripts/bag_sharpness.py` - blur report
* `maps/orbslam/*.osa` - saved ORB-SLAM3 maps

`orbslam_run.sh` uses its own ROS master on port 11511, so it can run while the robot stack is up.

## What was and was not tested (2026-10-05)

Tested on a 37 s scratch bag recorded from the camera alone with the robot standing still:
IR streams at 30 fps, manual IR exposure, alternate projector, `cam_setup.py`, settings generation for
all five modes, frame selection by projector state, `rgbd`, `ir-stereo`, `ir-rgbd` tracking (100 %),
map save, trajectory evaluation against wheel odometry, the sharpness report.

Not tested yet:

1. ~~`slam_record.sh` end to end~~ - worked on the first real run (`lab_loop_01`, 2026-10-05): all image
   streams at 30 fps, IMU at 199 Hz. If Ctrl-C does not end the teleop, run
   `pkill -INT -f stretch_twist_arm_teleop` in another terminal; the script then closes the bag normally.
2. **The inertial modes while moving.** ORB-SLAM3 needs acceleration to initialise the IMU; on the
   stationary test it correctly reported "never initialised". They may also struggle on a robot that
   only moves in a plane.
3. **`--load-map` merging.** The map loads; whether ORB-SLAM3 recognises the place and merges needs a
   real second run through the same space.
4. Frame drops while recording 30 fps with the whole robot stack running: `slam_record.sh` prints the
   achieved rates at the end and flags anything below 90 %. If it flags them, use `--fps 15`.

## Known limits

* No ground truth. "ATE vs wheels" compares against wheel odometry, which drifts. Drive a loop back to a
  floor mark and look at "start->end" as well.
* The final trajectory covers ORB-SLAM3's biggest map only. If it got lost and started new maps, the
  tracking numbers (from every frame) are the ones to trust.
* This D435i does not report the colour sensor's exposure per frame; with colour auto exposure the
  sharpness report shows it as "auto". The IR exposure is reported.
* ORB-SLAM3 has no localisation-only mode against a loaded map in this wrapper; `--load-map` runs full SLAM
  and merges.
* After a clean shutdown the ORB-SLAM3 node aborts in a ROS plugin destructor ("LibraryUnloadException" at
  the end of `orbslam.log`). Results and maps are already written at that point.
