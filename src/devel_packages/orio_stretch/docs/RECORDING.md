# Recording SLAM / navigation datasets on Stretch RE1 (`stretch-re1-1076`)

> Moved here from `~/orio/RECORD_README.md` on 2026-11-13. `scripts/`, `launch/` and `config/nav/` are relative to this
> package (`orio_stretch`); `datasets/`, `maps/`, `viewer/` and `config/waypoints_*`, `config/truth_*` are relative
> to the data root `$ORIO_DATA` (default `~/orio`). See `../README.md` for the layout and the demo run order.

Written for whoever (human or agent) next evaluates SLAM / localization /
navigation algorithms on bags recorded with `launch/record.launch`.
Read the whole thing once. Section 7 is the list of things the recording
setup does **not** solve for you.

Robot: Stretch RE1, Ubuntu 20.04, **ROS 1 Noetic only** (no ROS 2 on this PC).
Sensors: Intel RealSense D435i on the head (pan/tilt), RPLidar A1 on the base,
wheel odometry + base IMU + wrist IMU from `stretch_driver`.

---

## 1. Quick start

```bash
source scripts/orio_env.sh             # ROS + ~/catkin_ws + this package on ROS_PACKAGE_PATH; ORIO_DATA=~/orio
export DISPLAY=:0                      # only needed over SSH (teleop opens a window on the robot's screen)

roslaunch orio_stretch record.launch run_name:=lab_loop_01
#   ...drive with the teleop window (w/s/a/d, r/f + t/g for speed, SPACE stops)...
#   Ctrl-C IN THE TELEOP WINDOW to stop everything and close the bag cleanly.
```

Output: `~/orio/datasets/<run_name>/` (see section 3). `run_name` empty gives
`run_YYYYMMDD_HHMMSS`. Recording into an existing folder is refused.

Useful args (`roslaunch orio_stretch record.launch --ros-args` lists them all):

| arg | default | notes |
|---|---|---|
| `run_name` | auto | folder name under `datasets_dir` |
| `width` `height` `fps` | 640 480 15 | matches the stretch_rtabmap config. 1280x720 triples the bag size |
| `unite_imu_method` | linear_interpolation | also publishes `/camera/imu` (single IMU topic for VIO). `none` disables |
| `record_depth_raw` | true | also record un-aligned `/camera/depth/image_rect_raw` (+9 MB/s) |
| `head_pan` `head_tilt` | 0.0  -0.30 | rad, set once before the bag opens; `set_head:=false` skips |
| `max_linear` `max_angular` | 0.30  0.80 | teleop caps. Fast spins smear lidar + blur RGB |
| `split` | false | see 7.4 before turning on |
| `teleop` | true | `false` = start the teleop yourself: `rosrun orio_stretch stretch_twist_arm_teleop.py _lock_head:=true` |
| `skip_preflight` | false | see 2.1 |
| `extra_topics` | "" | space-separated extra topics |

## 2. Preconditions (each one has already cost a run)

### 2.1 Nothing else may own the robot
A cron `@reboot` job starts `~/teleop_server/start_teleop.sh` on every boot. It
runs `robot_server.py` (owns **all** `/dev/ttyACM*` = every motor, the pimu and
wacc) and `camera_server.py` (owns the D435i `/dev/video*`) plus a cloudflared
tunnel. While it runs:
* `stretch_driver` exits with code 1 about one second after start, silently (no
  `/odom`, no `/joint_states`, no trajectory action, no odom TF).
* the RealSense node logs `xioctl(VIDIOC_S_FMT) failed, errno=16 Device or resource busy`
  and publishes no images.

`record_wrap.sh` runs a **preflight** that lists any process holding
`/dev/ttyACM*` or `/dev/video*` and aborts the launch. To free the robot:

```bash
pkill -f "bash start_teleop.sh"        # its trap kills robot_server, camera_server, cloudflared
sleep 2; pgrep -af "robot_server|camera_server|cloudflared" || echo free
```
It comes back at next reboot (or `cd ~/teleop_server && TUNNEL_NAME=re1-1076-teleop bash start_teleop.sh &`).
The same applies to any `stretch_body` script (`stretch_robot_*`, the
`scripts/stretch_body_move_demo.py` in this repo, etc.).

### 2.2 Robot state
* **Runstop released** (button on the head glows steady, not blinking). With it engaged the
  driver runs but the base ignores `cmd_vel`, and the head move at start fails.
* Robot **homed** (`stretch_robot_home.py` after a power cycle) so joint values are meaningful.
* Arm retracted, lift low, wrist tucked: the arm is NOT masked out of the depth image. A raised
  gripper close to the camera produces bad depth (D435i min-Z at 640x480 is ~0.28 m).
* Battery: `/battery` is in the bag; below ~12.5 V wheel odometry gets sloppy.

### 2.3 Environment
* The driver is launched in **navigation mode** and publishes the **only** `odom -> base_link` TF.
  The laser scan matcher (which also publishes that TF in the stock stretch_rtabmap `robot.launch`)
  is deliberately NOT started. Do not add it back with `publish_tf` on.
* Head pose is set once (default pan 0, tilt -0.30 rad = ~17 deg down) and the teleop's head keys
  are locked (`~lock_head`). The actual reached pose is in `meta.txt` (`HEAD_POSE pan=... tilt=...`).
  If `set_head_pose` failed, `meta.txt` says so and the head pose is whatever it was.

## 3. What a run folder contains

```
datasets/<run_name>/
  run_<date>.bag                 lz4-compressed rosbag (one file unless split:=true)
  run_<date>.info.txt            `rosbag info` written at shutdown
  stretch.urdf                   the URDF robot_state_publisher used (calibrated one from stretch_description)
  controller_calibration_head.yaml   stretch_core head calibration used by the driver
  d435i_preset.json              RealSense visual preset loaded at start (HighAccuracyPreset)
  stretch_user_params.yaml, stretch_configuration_params.yaml   stretch_body params (wheel sep, etc.)
  params_at_start.yaml / params_20s.yaml / params_at_end.yaml   full parameter server dumps
  record.launch.used             copy of the launch file at the time of recording
  meta.txt                       robot, date, package versions, launch args, head pose, rosbag cmd,
                                 + three lines to FILL IN BY HAND (start/end floor mark, notes)
```
`*.bag.active` means the bag was not closed: `rosbag reindex file.bag.active` then rename.

## 4. Topics in the bag

| topic | type | rate | frame | notes |
|---|---|---|---|---|
| `/camera/color/image_raw` + `camera_info` | Image rgb8 640x480 | 15 | `camera_color_optical_frame` | RGB |
| `/camera/aligned_depth_to_color/image_raw` + `camera_info` | Image 16UC1 (mm) | 15 | `camera_color_optical_frame` | depth registered to color = what RTAB-Map consumes |
| `/camera/depth/image_rect_raw` + `/camera/depth/camera_info` | Image 16UC1 | 15 | `camera_depth_optical_frame` | native depth (if `record_depth_raw`) |
| `/camera/extrinsics/depth_to_color` | Extrinsics | latched | | for re-aligning offline |
| `/camera/gyro/sample` `/camera/accel/sample` | Imu | 200 / 63 | `camera_gyro_optical_frame`, `camera_accel_optical_frame` | raw, separate |
| `/camera/imu` | Imu | ~200 | `camera_imu_optical_frame` | gyro+accel united (`unite_imu_method`) for VIO |
| `/camera/accel/sample_corrected` | Imu | ~15 | | stock stretch node: accel re-stamped with NUC time, rate reduced |
| `/scan` | LaserScan | 10 | `laser` | raw RPLidar (Boost mode, 12 m) |
| `/scan_filtered` | LaserScan | 10 | `laser` | shadow-filtered (`laser_filters`). stretch_rtabmap feeds RTAB-Map the raw `/scan`; both recorded |
| `/odom` | Odometry | 30 | `odom` -> `base_link` | wheel odometry from stretch_driver |
| `/imu_mobile_base` `/magnetometer_mobile_base` | Imu, MagneticField | 30 | `base_link` | pimu |
| `/imu_wrist` | Imu | 30 | `wrist` accel only | |
| `/stretch/joint_states` | JointState | 30 | | raw from the driver (head, lift, arm, wrist, gripper, wheels) |
| `/joint_states` | JointState | 30 | | joint_state_publisher aggregate (same content) |
| `/tf` | TFMessage | 30 | | `odom->base_link` (driver) + all movable URDF joints (robot_state_publisher) |
| `/tf_static` | TFMessage | latched | | fixed URDF frames incl. camera optical frames |
| `/stretch/cmd_vel` | Twist | 20 | | teleop command (zeros while idle: dead-man publishes continuously) |
| `/battery` `/is_runstopped` `/mode` `/is_calibrated` `/tool` | misc | 1-30 | | run metadata |
| `/rosout_agg` | Log | | | driver / camera warnings during the run |

Size: about 32 MB/s raw before lz4 at 640x480@15 (color 14, aligned depth 9,
raw depth 9). Depth compresses well; expect roughly 1 GB/min on disk.
391 GB free on the NVMe at the time of writing.

### Not recorded, on purpose
* `/camera/depth/color/points` (~150 MB/s). Recreate offline from aligned depth + color +
  camera_info with `depth_image_proc/point_cloud_xyzrgb`, or let RTAB-Map do it.
* infra1/infra2 streams (disabled; enabling them also disables the IR projector pattern option).
* `/camera/*/compressed`, `/theora`, `/parameter_*`, `/metadata`.
* Point cloud filters / decimation: none applied. Depth is unfiltered except the HighAccuracy preset.

## 5. TF contract in the bag

```
map                      <- NOT in the bag. Your SLAM produces it.
 └─ odom                 <- stretch_driver only (30 Hz, from /odom)
     └─ base_link        <- robot_state_publisher from here down
         ├─ laser
         ├─ link_head -> link_head_pan -> link_head_tilt -> camera_bottom_screw_frame
         │      -> camera_link -> camera_{color,depth,gyro,accel,imu}_optical_frame
         └─ link_mast -> link_lift -> arm ... -> wrist -> gripper
```
* Exactly one publisher of `odom->base_link`. Its timestamps equal `/odom`.
* Head joints are in `/tf` (not `/tf_static`) because they are movable, even though they
  are held fixed during the run. Any camera-extrinsic assumption must use TF at the
  message stamp, not a constant.
* RTAB-Map on this robot is configured with `frame_id=base_link`, `odom_frame_id=odom`,
  `Reg/Force3DoF=true`, `Kp/MaxDepth=4.0`, `Grid/FromDepth=true` (see
  `~/catkin_ws/src/stretch_ros/stretch_rtabmap/launch/rtab.launch`). A comparison against the
  "live" map should use those same settings first.

## 6. Replay recipes (ROS 1)

Always:
```bash
rosparam set use_sim_time true
rosbag play --clock --pause ~/orio/datasets/<run>/run_*.bag      # press SPACE to start
```
Do **not** start any driver or `robot_state_publisher` alongside the bag: `/tf` and
`/tf_static` are already in it. Run only the algorithm under test.

**RTAB-Map mapping from a bag** (same parameters as live):
```bash
roslaunch stretch_rtabmap rtab.launch localization:=false args:="-d" database_path:=/path/new.db \
  wait_for_transform:=0.5 pointcloud_topic:=/camera/aligned_depth_to_color/image_raw \
  cmd_vel_topic:=/stretch/cmd_vel odom_topic:=/odom scan:=/scan \
  rgb_topic:=/camera/color/image_raw rgb_camera_info:=/camera/color/camera_info
```
(`-d` wipes `database_path`; never point it at a map you want to keep.)
`rosbag play --rate 0.5` if RTAB-Map drops frames (watch `rtabmap: Rtabmap/DetectionRate`, and
`/rtabmap/info` for `loop closure` ids).

**Localization against an existing map**: `localization:=true`, no `-d`, `database_path` = a
copy of the map (see `~/orio/rtabmap_commands.md` section 4; the RTAB-Map pipeline is retired and was not moved into this package). Compare `/rtabmap/localization_pose` over
the run.

**Testing a different odometry** (visual, ICP, EKF...): the recorded `odom->base_link` TF will
fight yours. Make a stripped bag once:
```bash
rosbag filter run.bag run_noodomtf.bag \
  "topic != '/tf' or all(tr.header.frame_id != 'odom' for tr in m.transforms)"
```
`/odom` (the topic) stays in and can still be fed to an EKF. If your odometry also wants the
robot TF tree but not the wheel odom, that stripped bag is what you play.

**Different depth alignment / point cloud**: play the bag and run `depth_image_proc` nodelets
(`register` to re-align from `/camera/depth/image_rect_raw` using
`/camera/extrinsics/depth_to_color`, `point_cloud_xyzrgb` to build clouds).

## 7. Things this setup does NOT take care of (check these when evaluating)

1. **No ground truth.** Nothing on the robot gives it. Per run the human is asked (in `meta.txt`)
   to note the start floor mark and whether the run ended on it, so the only free metrics are
   start-to-end loop error and map consistency. `stretch_aruco` + printed markers can add sparse
   anchors if you need more; that was not done.
2. **Nav2 is ROS 2; this robot is ROS 1.** Convert with the pure-python `rosbags` package
   (`pip install rosbags; rosbags-convert --src run.bag --dst run_ros2`). After conversion:
   `/tf_static` needs transient-local QoS when played (`ros2 bag play --qos-profile-overrides-path`),
   message type names map 1:1 for everything in this bag, and `realsense2_camera/Extrinsics`
   has no ROS 2 twin, so drop it (`--exclude` on the play side, or re-align in ROS 1 first).
   Closed-loop Nav2 (actually driving) can never be evaluated from a bag and needs the
   Ubuntu 22.04 / Humble stretch_ros2 install, which does not exist on this PC.
3. **Sensor clocks are not hardware-synchronised.** RealSense frames are stamped by the driver's
   global-time mapping (device clock -> host clock); lidar scans are stamped on receipt at 10 Hz;
   `/odom` at 30 Hz from the driver loop. Expect tens of ms of skew between color and scan.
   RTAB-Map's approximate sync (`approx_sync`, default queue 10) copes; tight VIO may not.
4. **`split:=true` drops latched topics from later files.** `/tf_static` and the extrinsics are
   published once; with splitting they exist only in the first file. If you split, always play all
   files of a run together (`rosbag play run_*.bag`), never a single later chunk.
5. **Wheel odometry covariance is a constant** from stretch_driver (not tuned). If an EKF is
   part of the algorithm under test, set covariances yourself.
6. **`/camera/accel/sample_corrected`** is the stock stretch hack that re-stamps accel with NUC
   time at a reduced rate. It is not gyro-synced. Use `/camera/imu` for VIO.
7. **Motion blur / rolling shutter**: D435i color is rolling shutter; angular cap 0.8 rad/s is a
   compromise, not a guarantee. Check RGB sharpness in a run before blaming the algorithm.
8. **Head pose is fixed by convention, not enforced by hardware.** The teleop locks its head
   keys, but nothing stops another node from moving the head. Verify `joint_head_tilt` in
   `/stretch/joint_states` is constant over the run.
9. **Lidar and camera fields of view differ.** The lidar sees 360 deg at ~0.17 m height; the camera
   sees ~69x42 deg tilted down. Obstacles seen by one and not the other are expected, not a bug.
10. **RealSense `initial_reset:=true`** adds ~10 s of startup and occasionally leaves the camera
    in a bad state on the *next* run (USB re-enumeration). If images are missing, check
    `rosout_agg` in the bag for `VIDIOC_S_FMT` / `busy`, and section 2.1.
11. **No CPU/latency record.** The NUC (8 cores, 15 GB) was near idle while recording (drivers
    only). A live SLAM run competes for CPU; offline results may be better than live ones.
12. **Base velocity limits**: the RE1 base caps at 0.30 m/s; the teleop default max matches.
    Keys are dead-man (1 s), so `/stretch/cmd_vel` contains a 20 Hz stream of zeros when idle.
13. **Both `/scan` and `/scan_filtered` are recorded**; stretch_rtabmap feeds RTAB-Map the raw
    `/scan`. Say which one you used.
14. **stretch_driver was launched with `fail_out_of_range_goal=false`**, so head goals outside
    the limits are clipped silently; that is why `meta.txt` records the *reached* head pose.

## 8. Per-run checklist

- [ ] `pgrep -af "robot_server|camera_server|stretch_body"` is empty (2.1)
- [ ] runstop released, robot homed, arm stowed, battery > 12.5 V
- [ ] tape / mark the start pose on the floor; write it in `meta.txt` afterwards
- [ ] `roslaunch orio_stretch record.launch run_name:=<place>_<pattern>_<nn>`
- [ ] wait for `[record_wrap] starting:` and `Recording to ...` in the launch terminal before driving
- [ ] drive at walking pace, slow through turns, revisit the start area at least once (loop closure)
- [ ] return to the start mark, stop, Ctrl-C in the teleop window
- [ ] check `run_*.info.txt`: every topic in section 4 present with a sensible count
  (images should be ~15 x duration; `/odom` ~30 x duration)
- [ ] fill in the three hand lines in `meta.txt`

## 9. Self-test status (2026-09-22)

Verified on the robot with the `teleop_server` service still running (it could not be stopped
by the agent that wrote this): launch, gnome-terminal teleop window (inherits ROS env, keys
publish `/stretch/cmd_vel`), lidar `/scan` + `/scan_filtered`, `/camera/imu` at ~195 Hz, `/tf`,
`/tf_static`, `/joint_states`, lz4 bag closed cleanly on Ctrl-C, `rosbag info`, param dumps,
URDF/calibration copies, preflight abort message.
**Not verified in that run** (blocked by 2.1): `/odom`, `/stretch/joint_states`, images, head
move. Rerun the self-test with the service stopped:
`roslaunch orio_stretch record.launch run_name:=selftest` for ~60 s, then check the `.info.txt`.
