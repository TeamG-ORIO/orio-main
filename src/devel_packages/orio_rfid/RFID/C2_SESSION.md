# c2 RFID session - code map and run order (prepared 2026-11-06)

The checklist page (claude.ai artifact "Robot session checklist") is the order of work. This file says which
command does each step. All scripts print `--help`.

| Step | Command | Notes |
| --- | --- | --- |
| Home with the payload | `python3 ~/orio/scripts/home_payload.py` | NOT `stretch_robot_home.py`: the gripper payload spoils its lift zero. Writes `config/payload.env` (LIFT_EFFORT), which every lift move below uses for upward strokes. Redo after every power cycle. |
| Preflight | `~/orio/scripts/rfid_preflight.sh` | teleop server, disk, battery, homed, runstop, payload homing this boot, reader 10 s read check, lidar alone, files. `--lidar-only` reruns the lidar test (c1's lidar was all `inf`; on 2026-11-06 it timed out at scan start). |
| Antenna height | `python3 ~/orio/scripts/lift_measure.py 0.40 0.80` | moves the lift, prints the reading, asks for the tape value; appends to `~/orio/datasets/c2_measurements.txt` |
| Drive between passes | `~/orio/scripts/drive.sh --no-pose` | `--no-pose` so the lift is not sent to 0.9 m |
| Each pass | `~/orio/scripts/rfid_pass.sh P0 ... P8` | one terminal: bag (as slam_record.sh) + RFID log + lift motion + sidecar logs + check |
| End | `~/orio/scripts/rfid_session_finish.sh` | table of passes, small archive without bags, rsync commands |

## Passes (`rfid_pass.sh --help` has every option)

| Preset | Label | What the script does |
| --- | --- | --- |
| P0 | c2_static_start | 2 min still in front of one shelf; `--tag NAME --distance-m D` for 2-4 of its reference tags (tape from the panel FRONT face; +0.019 m is added for the phase centre). Runs `phase_static_check.py`. |
| P8 | c2_static_backface | right after P0, same spot, robot turned 180 deg: what is read from behind (back lobe, reflections off metal). 60 s. Compared with P0. |
| P1 / P2 | c2_wave_fwd / c2_wave_back | lift triangle wave 0.45-0.90 m (0.30 is not reachable: the payload hits the base at 0.39 m, measured 2026-11-06), 11 s per up-and-down (`lift_motion.py wave`), teleop |
| P3 | c2_near | lift 0.60, lap ~0.3 m closer. Also the only pass that breaks the front/back mirror of a flat aperture (see below). |
| P4 | c2_checkpoints | lift 0.60; on each floor cross press its number, 1-9 and 0 = 10 (`/slam_test/mark`) |
| P5 | c2_bay_sweeps | lift rests at 0.60; pressing a number at a shelf stops the base and runs 0.45 -> 0.90 -> 0.60 |
| P6 | c2_static_end | repeat P0; channel offsets compared with P0 |
| P7 | c2_two_panel | `--antennas "4 <port>"`; measure the second panel like the first and write `antenna_transform_port<N>.yaml` |

A pass folder `~/orio/datasets/<label>/` holds: `run_*.bag` (+ info), `<label>_<time>.csv` + `_reader.json` (RFID),
`lift.csv`, `odom.csv`, `events.jsonl` (marks, lift strokes, runstop), `sidecar_summary.json` (lidar finite share),
`meta.txt` (profile, reader settings, note), `check.txt`, `antenna_transform.yaml`, `tag_registry.csv`, and for
static passes `channel_offsets.json`.

## New files and changes

- `~/orio/scripts/`: `rfid_pass.sh`, `lift_motion.py`, `pass_sidecar.py`, `rfid_pass_check.py`, `rfid_preflight.sh`,
  `lift_measure.py`, `rfid_session_finish.sh`
- here: `phase_static_check.py` (multi-tag split of phase into tag constants + channel offsets; compare; back face)
- `rfid_log.py`: SIGTERM stops it cleanly (same as Ctrl-C)
- `rfid_pose_join.py`: lift interpolated (it moves now), `--lift-csv lift.csv` instead of scanning the bag,
  `--antenna-port N=yaml` for a second panel
- `~/orio/stretch_twist_arm_teleop.py`: mark key `0` = point 10
- originals: `~/orio/backups/pre_c2_20261106/`

## Things the analysis side must handle (GPU box, loc/)

- The lift moves during P1, P2 and P5: use the per-read `lift_m` / `ant_z` from `rfid_pose_join.py`, never one lift per pass.
- Front/back mirror: a straight pass with the lift moving is a FLAT aperture (along x, up z). Phase alone cannot tell
  a tag at +y from its mirror at -y; only the panel pattern, the shelf map, or a second distance (P3) breaks it.
  P8 measures how much the panel pattern can be trusted for that.
- Two ports (P7): the `antenna` column selects the offset.
