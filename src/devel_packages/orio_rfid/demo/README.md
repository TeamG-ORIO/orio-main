# Scan demo: one command, one window

The robot drives the corridor by itself and reads the RFID tags; a PC shows the run, marks every item present or
missing in the inventory, and places each item on a shelf with one of the localisation methods F1 to F4.

```
PC (abuela today)                                       robot (stretch-re1-1076)
  demo/run_scan_demo.sh                                   orio_stretch/scripts/demo_agent.py     <- started over ssh
    sync/pc_receiver.py   <-- sightings, posed reads ---    scripts/rfid_nav_run.sh              (the run itself)
    loc/live/engine.py    --> item_locations table          RFID/rfid_pose_live.py               (posed reads, live)
    demo/scan_dashboard.py --- commands, status -------->   sync/robot_sync.py --follow          (sends them)
```

## Run it

On the PC:

```bash
demo/run_scan_demo.sh              # receiver + robot agent + item localiser + the window
demo/run_scan_demo.sh --stop       # stop everything, including the robot's agent
```

A PC without a checkout of orio-main gets the files from the robot with `demo/push_to_pc.sh <user@pc>` (it copies
`orio_rfid/{demo,sync,loc}` to `~/orio_scan/orio_rfid`), then runs `~/orio_scan/orio_rfid/demo/run_scan_demo.sh`.

Rehearsal without moving the robot: in the window choose **Replay an old run** and press START. The robot's agent
plays the reads of that run back at real speed; everything after the reader behaves as in a real run.
`demo/run_scan_demo.sh --local` runs both ends on one machine (for example on the robot itself).

The map (`demo/scene_view.py`) is the scene of the run-2 replay page drawn with numpy and Pillow instead of a browser: same room
frame, shelf colours and point cloud (`loc/site/scene_v2.json`, built by `loc/site/make_scene.py`). Without numpy or Pillow on the
PC the window still works and draws the route only.

## Settings: `demo/demo.conf`

`demo.conf` is not in git, because it holds this site's addresses. On a fresh checkout copy `demo.conf.example` to
`demo.conf` and fill in `ROBOT_HOST` and `PC_HOST`.

| Key | Meaning |
| --- | --- |
| `ROBOT_HOST`, `ROBOT_USER` | the robot as the PC reaches it (ssh and port `AGENT_PORT`, default 8123) |
| `PC_HOST` | this PC as the **robot** reaches it (the robot posts to port `RECEIVER_PORT`, default 8000) |
| `PYTHON` | Python on the PC: tkinter, and torch / numpy / pandas / scipy when `ENGINE=pc` |
| `DB` | SQLite inventory. If the file is missing a test copy is built (seed inventory + every item tag of `RFID/tag_registry.csv`) |
| `ENGINE` | `pc`: the robot sends its reads and this PC computes. `robot`: the robot computes. `off` |
| `METHOD` | default method; the window's F1..F4 buttons choose it per run |
| `CELLS` | `all` (default): the whole room grid, exactly as the offline method. `shelf`: only the cells in or near the shelf boxes, for F3 / F4; each item then gets a new position every 4 s instead of every 11 s, and 1 item in 130 moved by 2 cm on one of three recordings (`../loc/README.md`, "Speed and delay") |
| `MAP`, `RUN_DIR`, `WAYPOINTS`, `LAPS`, `START_AT` | the site, as paths **on the robot** |

Today both addresses are Tailscale addresses. On the team router (`TeamG_ORIO_5G`, no internet) only `ROBOT_HOST`
and `PC_HOST` change. The PC needs an ssh key on the robot (`ssh-copy-id hello-robot@<robot>`), and on the PC port
8000 must be open to the robot (`sudo ufw allow 8000/tcp` if ufw is on).

## The window

| Part | What it does |
| --- | --- |
| START | real run: asks once, stops the robot's teleop server, runs `rfid_nav_run.sh`. Replay: plays the chosen old run |
| PAUSE / RESUME | the base holds at the spot (the goal is cancelled and sent again on resume); lift and reader keep going |
| STOP | ends the run cleanly: everything is stopped and the reads so far are still posed |
| E-STOP (or Esc) | `/runstop` of the driver: all motion stops and stays stopped, then the run is stopped. Release with the button on the robot's head (hold 2 s) or RELEASE RUNSTOP while a driver is running |
| PREFLIGHT, HOME, CLEAN UP | `rfid_preflight.sh`, `home_payload.py` (asks first: the lift goes to the top), `ros_clean.sh` |
| Checks | one line per thing that must work: agent, receiver, teleop server, preflight, navigation, reader, antenna pose, reads at the PC, route, presence, localiser, locations. **Localiser** is the localiser program itself: it reports in every round, the line shows what it is doing, and turns red (`NOT RUNNING`, or the reason it stopped) when it has not been heard for 20 s. **Locations** turns red when reads are flowing but no position has arrived for 30 s |
| Map | the room in **2D** (from above) or **3D**, switched with the buttons above it: shelf boxes, the map's point cloud, waypoints, the driven path, the robot, and every item where the localiser has placed it so far. Drawn as on the run-2 replay page: a ball in the tag's own colour (its shelf's hue, its own shade) where it was placed in this scan, a diamond in the same colour at its tape-measured position, and a line between the two. Grey ring = last known place from an earlier scan, red cross = missing. Drag to move (3D: orbit, right-drag to move), wheel to zoom, double-click to reset. Point at an item for its name, shelf, uncertainty and read count; click it, or its row in the inventory, to mark it in both. Switches: point cloud, uncertainty (a translucent circle of the 90 % region's size), tape positions (on at the start; the 9 Nov measurement; with it on, the corner shows the median 3D error and an item's label says how far it is from its tape position) |
| Inventory | every item of the database: `on shelf`, `not seen yet`, or `MISSING` once the run is over; shelf from the localiser |
| Head camera | in manual mode, with no run going, the CAMERA button of the map panel shows the head camera (about 6 pictures a second, upright). `cam_stream.py` on the robot opens the camera once the driver is up and is stopped before a run, preflight or homing needs it. Not shown during a run |
| Manual control | MANUAL ON asks once, stops the teleop server, starts only the driver and then, by itself, goes to the run pose: lift up to the saved height first (with the payload effort from homing), then arm in, wrist turned in to 30 degrees short of stowed (2.88 rad; a run stows it fully, 3.40 rad, at its own start), head to its usual pan / tilt. POSE AGAIN repeats it. Everything on the pad is hold-to-move: the robot moves only while a button or key is held, with the pointer over the pad for keys, and stops 0.5 s after the last command at the latest. Base: the six pad buttons or W A S D / Q E, also while a run is paused. Lift: ▲ ▼ or R / F, up to 0.10 m/s, only with the arm in, over the lift's whole homed range (0 to 1.099 m; runs keep 0.42 to 0.90 m). Below 0.42 m watch the panel: on 6 Nov it touched the base at 0.39 m with the wrist stowed. If the lift meets something it stops and waits until the button is let go. Camera: the four arrows or I J K L, up to 0.5 rad/s; the middle button puts it back. The speed slider scales all three. MANUAL OFF puts the head back to its saved pan / tilt **before** the driver stops, so the camera looks the same way at the start of every run (a run also sets it at its start) |
| Head camera | in manual mode, with no run going, the CAMERA button of the map panel shows the head camera (about 6 pictures a second, upright); the view switches to it when manual mode is ready. `cam_stream.py` on the robot opens the camera once the driver is up and is stopped before a run, preflight or homing needs it. Not shown during a run |

The robot's agent accepts commands only with the token in `~/.orio_demo_token` on the PC (made on first start).

## What is stored

| Where | What |
| --- | --- |
| `DB`, table `medical_supplies` | unchanged layout; `is_on_shelf` and `last_seen` are set by the sightings. A new run sets every item to not on shelf first |
| `DB`, table `item_locations` | one row per tag: `shelf`, `shelf_prob`, `x_m y_m z_m` (room frame), `region90_m`, `method`, `n_reads`, `run`, `updated`. Not cleared by a new run: for a missing item it is where it was last |
| `<DB folder>/runs/<label>/` | `reads_posed_live.csv` (every read with the antenna pose), `run.json`, `locations.json` |
| `<DB folder>/logs/` | `pc_receiver.log`, `engine.log` |
| robot `~/orio/datasets/<label>/` | the run folder of `rfid_nav_run.sh`, plus `reads_posed_live.csv`, `nav_events.jsonl`, `robot_sync.out` |
| robot `~/orio/maps/logs/` | `demo_agent.log`, `demo_<label>.log` |

## Tested, and not

Tested 2026-11-13, with run 2 (7 Nov) replayed at real speed on the robot and abuela as the PC, over Tailscale:

- all 127,446 reads reached abuela and its copy of the read log is identical to the robot's, also after the
  receiver was killed and restarted mid-run
- presence: 149 of the 177 items of the test database marked on shelf
- F4 on abuela's GPU kept 2 to 4 s behind the reads for the whole run; result as good as abuela's offline one
  (see `../loc/README.md`)
- pause / resume in `nav_waypoints.py` against a pretend `move_base` on a private ROS master
- the window, on the robot's own screen at 1024 x 740, against a replay; the 2D and 3D map with the 138 item positions of the
  22:36 run (replayed), and the renderer alone on abuela (10 ms a picture). The mouse controls were exercised in code, not by hand

First try on the hardware (2026-11-13, 21:20): PREFLIGHT and HOME failed at once, because an agent started over
ssh had no ROS and no `HELLO_FLEET_PATH` (`~/.bashrc` sets them only in a terminal). Fixed in
`orio_stretch/scripts/orio_env.sh` and `demo_agent.py`, and checked in an empty environment; the two buttons have
not been pressed again since.

Not tested, because it needs the robot to move or its hardware to be free:

- START of a real run from the window, STOP and E-STOP during one
- PREFLIGHT, HOME and CLEAN UP from the window (the scripts themselves are the ones used on 7 Nov)
- manual mode on the real robot: the automatic pose move after MANUAL ON, driving, the lift and camera buttons, the head going back on
  MANUAL OFF, RELEASE RUNSTOP. (Tested 2026-11-14 only against a pretend driver on a private ROS master, and the order of the
  moves with stand-in processes)
- the window on abuela's screen, and anything on the team router
- `ENGINE=robot` during a real run (see the CPU numbers in `../loc/README.md`)

## When the localiser stops

At start the localiser waits (`ready, waiting for a run`). A run that was already over is not gone through again; its
positions stay in the database and on the map as last known places. `run_scan_demo.sh` runs the localiser in a loop: if it stops by itself it is started again after 5 s, picks the run up from
its beginning and catches up; the reason is in `logs/engine.log` and in the window's Localiser line. Closing the window (or
`--stop`) stops the loop, then the localiser, and waits until it is gone before anything new may start; a localiser left
over from an earlier session is stopped the same way at the next start. On 2026-11-13 this was missing: a new localiser
started while the old one was still finishing, the two did not fit on abuela's 8 GB GPU, the new one died, and nothing in
the window said so. Since then F3 and F4 also use 16 times less GPU memory (shelf cells only).
