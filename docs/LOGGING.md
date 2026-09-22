# Run logging with rerun

One `launch_demo.sh` session = one run folder `logging/rerun/<run_id>/` (git-ignored):

```
logging/rerun/20260914_153000/
  run.json          run id, git sha + dirty, args, env flags, configs, counts, per-state timing summary
  recorder.rrd      ROS-side streams + both arms in 3D (URDF)
  perception.rrd    per-call images, masks, grasp candidates
  configs/          grasp.yaml, motion.yaml, cell.yaml as used
```

```bash
bash launch_demo.sh [--no-logging] [--live]              # recorder pane starts with roscore
bash run_dexnet_pnp_single.sh [--no-logging] [--live]    # same, without tmux (recorder = bg process)
python3 src/devel_packages/orio_logging/runs.py list     # or: open latest | summary | compare A B | du
rerun logging/rerun/<run_id>/*.rrd                       # viewer (perception venv has `rerun`)
```

`--live` also streams to a `rerun` viewer already running on this machine. `--no-logging`
sets `ORIO_LOGGING=0` for every process: no recorder pane, no events, no perception writer.
The recorder refuses to start under 5 GB free and lists old runs to delete.

### Affordance panels

`run_dexnet_pnp_single.sh` writes the six-panel diagnostic per grasp to
`logging/dexnet_pnp/<stamp>.affordance/afford_<n>_<hhmmss>.png`: workspace, depth,
segmask, affordance map, affordance over scene, chosen grasp — the same figure
`orio_perception/test/live_affordance.py` renders offline, but for every call of a live
run. Rejected grasps get a panel too; a `q=0.01` result is exactly when the map and
segmask are worth looking at.

This is the **only** per-grasp image the launcher writes. An earlier single-frame PNG
drawn in the perception node was removed in favour of it: the panel is a superset (its
bottom-right cell is that same annotated close-up), and keeping both meant two images per
pick from two containers.

On by default. `--no-affordance` skips the panels; `--no-logging` turns them off along
with everything else.

This one lives in **`dexnet_grasp_planner.py`**, not the perception node, and that is
forced: the dense affordance tensor exists only inside the policy object, while
`PlanDexnetGrasp` returns just the chosen grasp. `_affordance_map` reaches into
`policy._unpack_state` / `_gen_images_and_depths` / `_grasp_quality_fn.quality` exactly as
`live_affordance.py` does, so it tracks those gqcnn internals and is exception-wrapped.

Cost, for the record: a **second forward pass** per call plus a ~0.3 s matplotlib
render (measured on the RTX 4090 against a 178x172 input). The panels show the
**post-processing** depth and segmask taken from the policy state — after inpainting and
the valid-pixel intersection — since that is what the network ran on; the raw service
inputs would overstate the mask and hide inpainting artefacts.

`run_dexnet.sh` passes `ORIO_AFFORDANCE_DIR` and mounts `logging/` into the dexnet
container (the planner had no such mount before this). The launcher gates the panels on
`LOGGING_REQUESTED` rather than `ORIO_LOGGING`: the latter gets zeroed when the host venv
lacks rospy/rerun, which says nothing about the dexnet container, where these are drawn.

### Host requirement: rospy *and* rerun in one interpreter

The recorder is the only piece that needs both, and they pull in opposite directions:
rerun 0.37.2 needs Python ≥ 3.9, while `rospy` ships with ROS1 noetic (Python 3.8, Ubuntu
20.04). On a workstation that is 22.04 with no host ROS1 — the stack is containerised
precisely because noetic is not installable there — `/opt/ros/noetic` may exist but be
empty, and the recorder dies at `import rospy`.

Fix: add the pure-python ROS1 packages to the perception venv (py3.10). They are the
community [rospypi/simple](https://github.com/rospypi/simple) builds, pip-installable and
independent of a system ROS:

```bash
src/devel_packages/orio_perception/venv/bin/pip install \
    --extra-index-url https://rospypi.github.io/simple/ \
    rospy rosgraph_msgs std_msgs geometry_msgs sensor_msgs actionlib_msgs
```

`actionlib_msgs` is needed for `franka_interface_msgs/RobotState` (the arm streams).
`smach_msgs` is not on that index; without it the recorder logs one warning and records
everything except smach transitions — which the DexNet path does not use anyway.
`run_dexnet_pnp_single.sh` checks for both imports up front and, if either is missing,
prints this command and continues with logging off rather than failing the run.

## Architecture

```
control PCs ──robot_state (100 Hz)──┐
pneumatics  ──has_item, rosout──────┤
dexnet      ──rosout, /orio/events──┤        ┌──────────────┐
state mach. ──/orio/events, rosout──┤──────▶ │  recorder.py │──▶ recorder.rrd
dexnet_pnp  ──/orio/events, rosout──┤        │  (host venv) │──▶ run.json
smach       ──container_status──────┘        └──────┬───────┘
perception  ──rerun SDK, worker thread──────────────┼──────▶ perception.rrd
                                                    └─ --live: gRPC ─▶ rerun viewer
```

Two writers, one `recording_id` (the run id); the viewer merges the files.

| Piece | Where |
|---|---|
| Recorder, run CLI, cell.yaml, meshes | `src/devel_packages/orio_logging/` |
| Event encoding (pure, no ROS) | `orio_core/orio_core/events.py` |
| Perception writer (worker thread) | `orio_perception/scripts/rerun_writer.py` |
| Affordance panels | `orio_bringup/docker/dexnet_grasp_planner.py` (`_save_affordance_panel`) |
| Event hooks | `state_machine.py`, `dexnet_pnp.py`, `pneumatic_control_recovery.py`, `dexnet_grasp_planner.py` |
| Bring-up | `launch_demo.sh`, `orio.session.sh`, `stop_demo.sh`, `orio_run_docker.sh`, `run_dexnet.sh`, `run_dexnet_pnp_single.sh` |

Why this shape: the state machine runs on Python 3.8 in the container, where rerun stops
at 0.22.1 and `.rrd` files are not portable across versions. So nothing in a container
imports rerun; nodes publish small JSON events on `/orio/events` (`std_msgs/String`, no new
msg types, no image rebuild) and the host recorder (perception venv, rerun 0.37.2) turns them
into rerun entries. Arm state comes straight from the control PCs' 100 Hz `robot_state`
topics; state transitions from the smach introspection topic; every node's log lines from
`/rosout_agg`. Only perception logs directly, because it already holds the images.

### Events

`{"t", "src", "kind", "name", "data"}` on `/orio/events`. Emitted by:

| src | kind | data |
|---|---|---|
| state_machine | `pick` | n |
| state_machine | `cmd` (every `goto_joints`) | joints, duration, dt_s, ok |
| state_machine | `ik` | target, pre, final, attempt, err_pre, err_final, label_zone |
| state_machine | `service` (every Trigger call) | dt_s, ok |
| state_machine | `zone`, `recovery`, `error`, `rfid`, `operator`, `shutdown`, `summary` | see code |
| pneumatics | `vacuum_cmd` | (name = serial command) |
| dexnet | `dexnet` | q, px, depth, plan_time_s, ok |
| dexnet_pnp | `pick` | n (attempt), placed |
| dexnet_pnp | `cmd` (every `_move`, name=`arm1`) | desc, joints, duration, timeout, dt_s, faults, ok |
| dexnet_pnp | `ik` (name=`arm1`) | target, pre, final, err_pre, err_final, ok |
| dexnet_pnp | `grasp` (`accepted` / `declined`) | pos, tilt_deg, reason, n_poses, ok |
| dexnet_pnp | `service` (every Trigger call) | dt_s, ok |
| dexnet_pnp | `operator`, `error`, `shutdown`, `summary` | see code |

### Entity layout

| Path | Archetype | Source |
|---|---|---|
| `/log/<node>` | TextLog | `/rosout_agg` |
| `/events/<kind>` (+ `/events/<kind>/<name>/<field>` for numeric fields) | TextLog, Scalars | `/orio/events` |
| `/sm/ORIO_ROOT/ARM{1,2}` | TextLog (transitions) | smach container_status |
| `/arm{1,2}/q`, `dq`, `tau_J`, `tau_ext`, `f_ext`, `success_rate` | Scalars, columnar | robot_state, 20 Hz |
| `/arm{1,2}/joints/panda_joint{1..7}` | Transform3D | `q` via `rerun.urdf` |
| `/arm{1,2}/panda/...`, `/tf_static` | Asset3D + static transforms | URDF loader |
| `/arm{1,2}/ee` | Transform3D + axes | `O_T_EE` |
| `/arm{1,2}/mode` | TextLog on change | robot_mode, current_errors |
| `/arm{1,2}/cmd/target_joints`, `/arm{1,2}/ik/{pre,final,target}` | Scalars, Points3D | events |
| `/vacuum/{pnp,lbl}/has_item` | Scalars 0/1 | has_item topics |
| `/world/arm2_base`, `/world/targets` | Transform3D, Points3D | cell.yaml, Target_Task_Poses.json |
| `/perception/pnp/image` (+ `/boxes`, `/masks`), `depth` | Image (JPEG), Boxes2D, SegmentationImage, DepthImage | per /compute_grasps |
| `/perception/pnp/grasp/{points,normals}` | Points3D, Arrows3D in `arm1/panda_link0` | planned grasps |
| `/perception/pnp/{q_value,tilt_deg,plan_time,n_grasps,center}`, `result` | Scalars, TextLog | replaces grasp_attempts.csv |
| `/perception/lbl/image` (+ `/boxes`, `/masks`, `/targets`), `z{1,2}/*` | same set | per labelling call |
| `/proc/{recorder,system,state_machine,perception,pneumatics,dexnet,cameras}/{cpu,rss_mb}` | Scalars, 1 Hz | psutil |

Timelines: `ros_time` (host receive time), `sensor_time` (robot_state header stamp), `pick`
(state machine pick counter), `label` (labelling call counter).

### 3D arms

`rerun.urdf.UrdfTree` loads `orio/panda_arm_hand.urdf` once per arm with `frame_prefix`
`arm1/` / `arm2/`; each 20 Hz sample logs seven joint transforms per arm (columnar, one
`send_columns` per joint per 0.5 s window). Arm 2 sits at the static transform from
`orio_logging/config/cell.yaml` (arm 2 base in arm 1's frame, world = arm 1 base). The
`ee` axes come from the reported `O_T_EE`; if they drift from the URDF hand, `cell.yaml`
or the tool length is wrong. Meshes: `orio_logging/franka_description/` (visual 11 MB +
collision 0.1 MB), resolved through `ROS_PACKAGE_PATH` which the recorder sets itself.

## Non-interference

- The recorder is its own process; absent, stalled or crashed, nothing else notices.
- Producers pay only one small `std_msgs/String` publish per event. No new camera subscribers.
- Perception handlers only enqueue to a bounded worker queue (32 items, drop-on-full,
  `dropped` counter); JPEG encoding and rerun conversion happen on the worker thread.
- Every hook is fire-and-forget; `EventSink.emit` swallows exceptions. Missing `orio_core`
  on a node's path degrades to no events, not a crash.
- `ORIO_LOGGING=0` disables everything in one switch.
- `stop_demo.sh` sends Ctrl-C to the recorder and perception panes and waits up to 5 s
  before killing the session; `.rrd` is append-only, so a hard kill loses at most rerun's
  batcher window (~200 ms) and the file still opens.

## Overhead

Microbenchmarks on iam-abuela (`orio_logging/test/bench_events.py`, 2026-09-14):

| Call | Where | Median | Max |
|---|---|---|---|
| `events.emit` (ik payload, 1 subscriber) | state machine, per event | 72 µs | 490 µs |
| log line forwarded to rosout | state machine, per line | 0.61 ms | 1.8 ms |
| perception enqueue (`rlog.pnp`) | service handler, per call | 14 µs | — |
| JPEG 400x365 crop | perception worker thread | 0.9 ms | 1.3 ms |
| JPEG 1920x1080 (`ORIO_LOG_FULL_FRAMES=1`) | perception worker thread | 12 ms | 16 ms |

A pick emits roughly 10 events and 30 log lines: about 20 ms of producer-side time per
pick, none of it inside a `goto_joints` motion. Recorder-side, a 6 s fake run (2 arms,
20 Hz, `test/fake_publishers.py`) wrote 108 samples/arm; the file is 14 MB of static
meshes plus ~1 MB/min of streams.

**End-to-end A/B (to run on hardware, not yet done):** same scripted dry run
(`--no-vacuum`, N ≥ 10 picks) with `--no-logging` and with logging, then a third run
without. Budget: +2 % cycle time, 0 added on `goto_joints`, recorder < 5 % of a core.
Every run records its own per-state durations, service latencies and `goto_joints` wall
times in `run.json` (from smach transitions and `cmd`/`service` events), so the comparison
is `runs.py compare <off> <on>` plus the `/proc/*/cpu` series. For the `--no-logging`
baseline, which has no recorder, time the picks from the state machine's terminal log.

## Open items

- Hardware A/B above.
- `rospy` on Python 3.11 is unsupported upstream; perception already depends on it and the
  recorder shares that path.
- Control-PC clocks vs host clock: `sensor_time` may be offset; `ros_time` is authoritative.
- Perception's debug PNG dumps (`Xtion_imgs/`, `ZED_imgs/`, `Segmented_imgs/`) still exist
  alongside the recording.
