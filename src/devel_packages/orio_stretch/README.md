# orio_stretch

The mobile half of ORIO: a Hello Robot Stretch RE1 (`stretch-re1-1076`) with an Impinj R700 RFID panel on the
lift. It maps a room from one recording, navigates it with the head camera only (ORB-SLAM3 localiser, depth
obstacle cloud, `move_base`), and drives a waypoint route on its own while the reader logs every tag read with
the antenna's map position.

ROS 1 Noetic. Nothing is compiled: every script runs from this folder.

This package is the clean copy of the code that grew in `~/orio` on the robot between September and November
2026. Only what the ORB-SLAM3 navigation and RFID path uses was moved; see [Not in this package](#not-in-this-package).

## Code here, data outside the repo

| What | Where |
| --- | --- |
| Code | this package: `scripts/`, `launch/`, `config/nav/`, `rviz/` |
| Reader tools | `../orio_rfid/RFID/` (`rfid_log.py`, `rfid_pose_join.py`, `tag_registry.csv`) |
| Data root | `$ORIO_DATA`, default `~/orio` |
| Recordings and runs | `$ORIO_DATA/datasets/<run>/` |
| Saved maps | `$ORIO_DATA/maps/orbslam/<map>.osa` and `<map>.live.yaml` |
| Launch logs | `$ORIO_DATA/maps/logs/` |
| Site files | `$ORIO_DATA/config/waypoints_<map>.yaml`, `truth_<site>.txt` |
| Per-boot state | `$ORIO_DATA/config/payload.env` (lift effort), `data_pose.env` (saved body pose) |

`scripts/orio_env.sh` defines these for the shell scripts and `scripts/orio_paths.py` for the Python ones.
A recording is 3 GB per minute, so none of it belongs in git. To use another data folder, export `ORIO_DATA`
before calling a script.

## What the robot needs

- `~/catkin_ws` with `stretch_ros` built (stock Stretch install) and `realsense2_camera`.
- `~/orbslam_ws` with `orb_slam3_ros` built, including the local patches described in
  [docs/SLAM_TESTS.md](docs/SLAM_TESTS.md) ("Where things are"). Another location: export `ORBSLAM_WS`.
- `ros-noetic-move-base`, `navfn`, `dwa-local-planner`, `map-server`.
- Python: `sllurp` for the reader, `stretch_body` (stock).
- The reader at `192.168.0.20` on the robot's Ethernet port.

No symlink into a catkin workspace is needed. `orio_env.sh` puts this package on `ROS_PACKAGE_PATH`, so the
scripts find `roslaunch orio_stretch ...` by themselves. To call `roslaunch` or `rosrun` by hand, run
`source scripts/orio_env.sh` in that terminal first.

## Demo run order

All commands are run from this folder.

**Every power cycle**

```bash
pkill -f robot_server.py; pkill -f camera_server.py   # ~/teleop_server starts at boot and holds the hardware
python3 scripts/home_payload.py                       # never stretch_robot_home.py: the payload spoils its lift zero
scripts/rfid_preflight.sh                             # disk, battery, runstop, homing, reader, lidar
```

**Once per site: record, then build the map**

```bash
scripts/slam_record.sh <run>              # drive a loop; press 1 on the start cross when leaving and when back
scripts/site_build.sh <run> <map_name>    # no robot needed; prints the navigation command for this site
```

**Navigation only**

```bash
scripts/nav_orbslam_start.sh hospital_corridor_v2 \
    --run-dir ~/orio/datasets/hospital_corridor_v2/orbslam/ir-rgbd_map \
    --waypoints ~/orio/config/waypoints_hospital_corridor_v2.yaml --start-at 1
```

`--start-at 1` means the robot is parked on waypoint 1, facing as when it was marked; it then starts without
teleop. Without it, drive 1 to 2 m by hand until the script prints `LOCALISED`. In a second terminal:

```bash
source scripts/orio_env.sh
python3 scripts/stretch_twist_arm_teleop.py _cmd_vel_topic:=/stretch/cmd_vel    # Shift+Q quits
```

(`scripts/drive.sh` is for driving with nothing else running: it starts its own driver.)

**Autonomous RFID run** (navigation, lift wave, reader log, posed reads)

```bash
scripts/rfid_nav_run.sh hospital_corridor_v2 \
    --run-dir ~/orio/datasets/hospital_corridor_v2/orbslam/ir-rgbd_map \
    --waypoints ~/orio/config/waypoints_hospital_corridor_v2.yaml --start-at 1 --laps 2
```

Output: `$ORIO_DATA/datasets/rfid_auto_<date>/reads_posed.csv`, one row per read with the antenna's map pose.

**From the demo window** (the same run, started from a PC)

```bash
# on the PC, in orio_rfid:
demo/run_scan_demo.sh
```

It starts `scripts/demo_agent.py` on the robot over ssh. The agent runs the scripts of this folder on command
(preflight, homing, `rfid_nav_run.sh`, pause, stop, E-STOP, manual driving) and reports the run's state; it also
sends the reads to the PC while the run is going. See `../orio_rfid/demo/README.md`.

**Stopping**

```bash
pkill -INT -f rfid_nav_run.sh          # stops everything; the reads recorded so far are still posed
pkill -INT -f nav_orbslam_start        # navigation only
scripts/ros_clean.sh                   # leftovers of a crashed run (does not touch ~/teleop_server)
```

**Check without the robot**

```bash
scripts/nav_orbslam_selftest.sh hospital_corridor_v2 0 hospital_corridor_v2 \
    ~/orio/datasets/hospital_corridor_v2/orbslam/ir-rgbd_map ~/orio/config/waypoints_hospital_corridor_v2.yaml
```

It replays the recording through the same nodes on its own ROS master (port 11511) and checks the map pose at
every taped point. Called with no arguments it runs on the basement recording; `-h` is not handled.

## Scripts

Every script prints its options in the comment block at its top; most also with `--help`.

| Group | Scripts |
| --- | --- |
| Environment | `orio_env.sh`, `orio_paths.py`, `ros_clean.sh`, `in_dir.sh` |
| Body and lift | `home_payload.py`, `prepare_pose.py`, `set_head_pose.py`, `lift_motion.py` |
| Driving | `drive.sh`, `stretch_twist_arm_teleop.py` |
| Recording | `slam_record.sh`, `record_wrap.sh`, `cam_setup.py`, `launch/record.launch` |
| ORB-SLAM3 offline | `orbslam_run.sh`, `orbslam_settings.py`, `orb_feed.py`, `orbslam_log.py`, `orbslam_eval.py`, `orbslam_map.py`, `orbslam_dense.py`, `orbslam_video.py`, `slam_bag.py` |
| Site build | `site_build.sh`, `nav_points.py`, `marks_report.py` |
| Navigation | `nav_orbslam_start.sh`, `launch/nav_orbslam.launch`, `orbslam_tf_bridge.py`, `depth_to_cloud.py`, `nav_waypoints.py`, `nav_orbslam_selftest.sh` |
| RFID run | `rfid_nav_run.sh`, `rfid_preflight.sh`, `pass_sidecar.py`, `pose_logger.py` |
| Demo window, robot side | `demo_agent.py` (HTTP on port 8123, no ROS inside), `ros_bridge.py` (its ROS half: robot state out, base velocity and runstop in), `cam_stream.py` (head-camera picture for manual mode, no ROS) |

`config/nav/*.yaml` are the base `move_base` values from the earlier lidar setup. `nav_orbslam.launch` loads
them and then overrides the plugin lists, the depth layer and the global planner; the values in use are in the
launch file.

While a run is going, `nav_waypoints.py` writes its progress to `<run>/nav_events.jsonl` and holds the base
while `<run>/nav_control` says `pause` (`echo pause > .../nav_control`, `echo run > ...`). `drive.sh --hold` brings
up the driver without the keyboard teleop, for driving from the demo window.

More detail: [docs/RECORDING.md](docs/RECORDING.md) (what is in a bag, TF rules, replay) and
[docs/SLAM_TESTS.md](docs/SLAM_TESTS.md) (ORB-SLAM3 modes, reference points, navigation pieces, known limits).
The teleop-driven RFID recording session (passes P0 to P8) is run from `~/orio`; see `../orio_rfid/RFID/C2_SESSION.md`.

## Things that bite

- The lift does not hold itself with the payload: released at height it slides down. Keep it up only with
  `~/orio/scripts/lift_hold.py`. The payload touches the base at 0.39 m; `lift_motion.py` clamps at 0.42 m.
- The reader accepts one connection. `rfid_nav_run.sh`, `~/orio/scripts/rfid_pass.sh`, `register_tags.py` and
  `../orio_rfid/sync/robot_sync.py --reader` cannot run at the same time.
- The terminal often does not deliver Ctrl-C to the stack; use the `pkill -INT` lines above.
- Waypoint headings should face the next waypoint (`nav_points.py --face-next`): a heading into a wall 0.3 m
  away made the camera lose tracking.
- Record the map with nobody standing next to the robot: a person in the mapping run closed a passage in the
  2D map once.

## Not in this package

These stayed in `~/orio` on the robot and are not used by anything here:

- the RTAB-Map and AMCL pipeline (`map_*.sh`, `nav_start.sh`, `relocalize.py`, `navigation.launch`, `mapping.launch`)
- the blur and exposure sweep (`blur_record.sh`, `blur_run.py`)
- the Stretch move demos and the hand-written MoveIt config
- the slam_toolbox launch and its map
- the c2 recording session (`rfid_pass.sh`, `rfid_pass_check.py`, `rfid_session_finish.sh`, `lift_measure.py`)
- the blur and video reports (`bag_sharpness.py`, `bag_video.py`)
- the result pages (`nav_plot.py`, `orbslam_viewer.py`, `viewer/map_viewer.html`)
- `lift_hold.py`

The last four groups were copied here on 2026-11-13 and taken out again the same day: no demo command reaches
them. Run them from `~/orio/scripts/` as before.

`~/orio` also keeps its own copy of the scripts moved here (package name `orio`, local git snapshot
`538cf5a`). The two copies read and write the same data folder. Edit this package from now on.

## Verified on 2026-11-13

Against the `~/orio` copy, on the robot, without moving it:

- both launch files resolve to the same parameters (`roslaunch --dump-params`; the only difference is the
  spelling of one path to the same file)
- every script passes a syntax check; the argparse tools start and print the same help (one docstring was reworded)
- `nav_points.py --list` and run lookup give identical output on existing runs
- the replay self-test on the corridor recording (`hospital_corridor_v2` from 0 s, run from this package): localised,
  all 8 taped checkpoints within 25 cm, and each within 6 mm of the 2026-11-06 run of the `~/orio` copy
  (0.000 to 0.005 m on the way out, 0.162 m and 0.199 m on the way back; log `maps/logs/nav_selftest_20261113_191020`)

Not yet run from this package: anything on the hardware (recording, live navigation, an RFID run).

Added the same day for the demo window (`demo_agent.py`, `ros_bridge.py`, pause and progress in `nav_waypoints.py`,
`drive.sh --hold`): tested with a replayed run and against a pretend `move_base` on a private ROS master, not on
the hardware. What was and was not tested is listed in `../orio_rfid/demo/README.md`.
