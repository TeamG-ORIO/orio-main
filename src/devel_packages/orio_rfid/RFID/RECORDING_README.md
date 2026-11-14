# RFID corridor passes - recording run-book (orio, 2026-10-21)

What the plan (Corridor RFID Localization, Stage 0-3) consumes per read: EPC, antenna, channel -> frequency, phase,
RSSI, reader timestamp, and the ANTENNA POSE at that instant (base pose in the map + lift height + the measured
base->antenna offset). Everything below exists to produce exactly that, plus the calibrations that make the phase usable.

## 0. One-time calibrations
- Antenna offset: done (antenna_transform.yaml, rfid_antenna.urdf.snippet). Re-measure only if the bracket moves.
- Reader phase, Stage 0 (reader and tag static, panel facing a REFERENCE tag at a tape-measured distance, 1-2 m, nothing moving):
    python3 rfid_log.py scan stage0_s1bl_120cm --duration 120          # ~500 records/s, hop table saved next to the log
    python3 rfid_calib.py ambiguity pass_logs/stage0_s1bl_120cm_*.csv --tag "Ref S1 BL"
        -> "DOUBLE the phase" or "use the phase as reported": this decides the SAR phase model
    python3 rfid_calib.py offsets pass_logs/stage0_s1bl_120cm_*.csv --tag "Ref S1 BL" --distance-m 1.20 [--double] --out calib/offsets_a4.json
    power-cycle the reader, repeat the static log, then
    python3 rfid_calib.py offsets pass_logs/stage0_again_*.csv --tag "Ref S1 BL" --distance-m 1.20 [--double] --compare calib/offsets_a4.json
        -> repeatability should be a few tenths of a radian; if not, offsets must be re-fitted per session from the reference tags
- Room frame <-> map frame: park the base at 3 tape-measured spots (e.g. 1.0 m in front of the shelf-1 BL mark, parallel to
  the shelf; 1.0 m in front of the BR mark; 1.5 m out), note each pose from the localiser (rostopic echo -n1 /orbslam/pose
  or /amcl_pose) and write them in calib/room_to_map.txt. Three pairs fix the 2D similarity.
- Lift: the antenna back-face centre is 0.559 m + lift position; log /joint_states (record_pass.sh does).

## 1. Every session
1. Reader on (it takes ~60 s to boot) and reachable:  ping 192.168.0.20
2. Quick rate check (10 s) with the panel facing a shelf:  python3 rfid_log.py scan check --duration 10
   expect hundreds of records/s and the hop table line; if ~37/s the batching is off (--report-every must be 20)
3. Stop the teleop server (it owns the hardware):  pkill -f robot_server.py; pkill -f camera_server.py
4. Charge level, runstop released, lidar spinning.

## 2. Recording a pass - option A: live localiser (ORB-SLAM3 on the saved atlas), fewest moving parts
    scripts/nav_orbslam_start.sh basement_lap1          # driver + lidar + camera + ORB-SLAM3 + bridge; wait for "localised"
    (or AMCL: scripts/nav_start.sh <map>)
    cd ~/orio-main/src/devel_packages/orio_rfid/RFID
    ./record_pass.sh c1_lift030_fwd                     # rosbag(/tf, joint_states, odom, scan, /orbslam/pose...) + RFID log
    drive the corridor at ~0.10 m/s with the shelves on the robot's LEFT; Ctrl-C ends the pass and prints the read-rate report
Output: ~/orio/datasets/rfid_<label>_<time>/{pass.bag, <label>_<time>.csv, <label>_<time>_reader.json, antenna_transform.yaml, tag_registry.csv, meta.txt}

## 2'. Option B: record the cameras too, localise OFFLINE with ORB-SLAM3 (best trajectory, more disk)
    scripts/slam_record.sh c1_lift030_fwd --emitter alternate --pose keep     # terminal 1: drivers + keyboard teleop + bag with IR/depth
    python3 rfid_log.py scan c1_lift030_fwd --out-dir ~/orio/datasets/c1_lift030_fwd   # terminal 2, started right after the bag opens
    afterwards:
    scripts/orbslam_run.sh c1_lift030_fwd ir-rgbd --load-map basement_lap1     # trajectory in the saved map frame (final_cam_traj.txt)
    python3 rfid_log.py report ~/orio/datasets/c1_lift030_fwd/c1_lift030_fwd_*.csv
Both clocks are the robot's; reader_us in the CSV is the reader clock for cross-checking latency.

## 3. What to record (per corridor side)
- 3 lift heights per corridor: below the lowest level, mid-rack, above the top (e.g. lift 0.0, 0.3, 0.6 m), each forward AND backward
- speed ~0.10 m/s (median tag 5 reads/s -> a read every 2 cm; weak tags still ~1 read / 7 cm); slower (0.05) for the metal bay
- one SLOW coverage pass per corridor with --mode 2 (dense-reader mode: 5x fewer reads but ~40% more tags seen) for presence/removal
- a 30 s STATIC log at the start of each session facing a reference tag (per-session offset check), and one at the end
- label convention: <corridor>_lift<cm>_<fwd|bwd>[_mode2][_v<arrangement>]; log arrangement changes (items moved/removed) in meta.txt
- photos of each shelf after any rearrangement (the ground-truth pipeline needs them); registry CSV copied automatically

## 4. After recording
    python3 rfid_log.py report <log.csv>                 # rates, gaps, channels
    rosbag info <pass.bag>                                # /tf map->base_link present? joint_states? lidar?
    keep: bag + csv + _reader.json + meta.txt + calib/ + tag positions CSV from the mapper/picker (ground truth)
