# Commands Reference

Commonly used commands with usage examples. Run all commands from the repo root.

---

## `run_dexnet_pnp_single.sh`

For the state of the DexNet integration as a whole (architecture, conventions, what has
been measured, open items) see `docs/DEXNET.md`.

Brings up the whole DexNet pick-and-place stack in a **single terminal**. Every
service runs in the background with its output merged into your terminal (each line tagged
`[module]`), while the pick loop runs in the foreground so its "Press Enter to pick" prompt
can read the keyboard. Press **Ctrl-C** to stop everything (services are killed and
containers removed). A full log of each run is saved to `logging/dexnet_pnp/<timestamp>.log`.

When the bin runs out (five scans with no valid grasp), the loop does not stop: it parks
the arm at home and asks you to add more items and press Enter. Press `q` or Ctrl-C at
that prompt to finish the run. This happens in `--auto` runs too.

Each run is also recorded for the `rerun` viewer in `logging/rerun/<timestamp>/` — arm
motion, every service's log lines, and the pick loop's own grasps, moves and IK solves.
Open it with `rerun logging/rerun/<timestamp>/recorder.rrd`. Recording needs `rospy` and
`rerun` in the perception venv; if they are missing the script says so, tells you what to
install, and carries on without recording (see `docs/LOGGING.md`).

Every grasp also saves a six-panel figure to
`logging/dexnet_pnp/<timestamp>.affordance/`, showing the workspace, the depth map, the
mask of what counted as the bin, a heat map of how good the network thought every spot
was, that heat map laid over the scene, and a close-up of the point it picked. This is
the same figure `live_affordance.py` makes offline, and it is what to look at when you
want to know *why* a grasp scored the way it did. Scans that were rejected get one too,
so you can see what the camera was looking at when nothing was picked. The script prints
the folder and the picture count when it stops.

You get these even when the message above says run logging is off: that message is about
the `rerun` recording, which needs the perception venv, while these are drawn inside the
DexNet container and need nothing extra.

They do make each pick slower, because the network runs a second time for every grasp.
To turn them off:

```bash
bash run_dexnet_pnp_single.sh --no-affordance
```

**Base command:**

```bash
bash run_dexnet_pnp_single.sh
```

| Option | Description |
| --- | --- |
| `--vacuum` | Start the pneumatics for real suction. |
| `--no-vacuum` | Dry-run with no suction; move the cup by hand (default). |
| `--auto` | Auto-pick — no per-pick Enter prompt. |
| `--confirm` | Before each pick, wait for Enter to execute it, `r` to regenerate the grasp, or Ctrl-C to abort (default). |
| `--straight-down` | Ignore grasp tilt and approach the object vertically. |
| `--offset-x M`, `--offset-y M` | Shift every pick by this much (metres, robot base frame) after DexNet has chosen it. Defaults come from `ORIO_PNP_OFFSET_X` / `ORIO_PNP_OFFSET_Y`, else 0. |
| `--min-q Q` | Lowest DexNet confidence (0 to 1) at which a grasp is accepted. Default 0.30, or `DEXNET_MIN_Q_VALUE`. Lower it to attempt low-confidence grasps on hard piles; grasps below it are rejected and the loop re-scans. |
| `--no-logging` | Do not record this run. |
| `--live` | Also stream the recording to a `rerun` viewer already open on this machine. |
| `-h`, `--help` | Print the script's usage header and exit. |

**Example:**

```bash
bash run_dexnet_pnp_single.sh --vacuum --auto --straight-down
```

**Correcting a constant landing error.** If the cup lands a fixed distance away from the
grasp drawn in the affordance figure, that is the camera calibration, not the planner.
Pass the correction in metres and the pick loop adds it to every grasp before moving the
arm; DexNet, the perception node and the figure are unchanged. In the figure, up is +Y and
right is +X in the robot frame, so an arm that lands above the drawn grasp needs a
negative `--offset-y`. The chosen value is printed on the `[launcher] pnp flags:` line
and in the `[Grasp]` log lines for each pick.

```bash
bash run_dexnet_pnp_single.sh --vacuum --offset-y -0.015
export ORIO_PNP_OFFSET_Y=-0.015    # or set it once for every run
```

---

## `snap_xtion.sh`

Saves one picture of what the Xtion camera sees right now. If a camera stack is already
running (for example a `run_dexnet_pnp_single.sh` or `launch_demo.sh` session in another
terminal) it uses that. Otherwise it starts only what the picture needs: the ROS 1 master
container, `orio_docker_container` and the Xtion driver. It takes the picture and then
stops **what it started**, leaving anything that was already running alone. A cold start
takes about ten seconds. The grab itself runs inside the perception container because the
host has no ROS 1 Python, and the files come out owned by you.

**Base command:**

```bash
bash snap_xtion.sh
```

| Option | Description |
| --- | --- |
| `[outdir]` | Where to save. Default: `logging/xtion/<timestamp>/`. |
| `--keep` | Leave the master, container and camera running afterwards (e.g. to take several pictures in a row). |
| `-h`, `--help` | Print the usage header and exit. |

**Examples:**

```bash
bash snap_xtion.sh                  # -> logging/xtion/20261005_192832/color.png
bash snap_xtion.sh ~/scenes/bin_a   # pick the folder yourself
bash snap_xtion.sh --keep           # first of several: later runs reuse the camera
```

Each run writes `color.png` (the RGB snapshot) and, next to it, `color.npy`, `depth.npy`
and `K.npy`, which is the scene format `live_affordance.py` and `compare_dexnet_methods.py`
read. It prints how much of the depth image was valid and the median range, which is a
quick way to tell whether the camera is actually seeing the bin. If the Xtion does not
start after five tries it says so and stops; see `docs/TROUBLESHOOTING.md` for the
camera's known start-up failure.

---

## `reset_robot.sh`

Resets one arm to its home joints from a single terminal. Brings up only what the reset
needs (the ROS1 master container, that robot's control PC, `orio_docker_container`), runs
`reset_joints.py` inside the container, then tears down **what it started**; anything
already running is reused and left alone. Ctrl-C at any point runs the teardown.

**Base command:**

```bash
bash reset_robot.sh
```

| Option | Description |
| --- | --- |
| `--robot N` | Which arm: `1` = iam-doc (default), `2` = iam-luisa. |
| `--unlock` | Open the brakes first via `lock_arms.py --unlock N` (arm moves slightly). |
| `--pose` | Use `reset_pose()` instead of `reset_joints()`. |
| `--keep` | Leave roscore, the container and the control PC up afterwards (e.g. before a demo run). |
| `--open-gripper` / `--close-gripper` | Also open/close the Franka Hand (robot 1 with a hand only; skipped by default). |
| `-h`, `--help` | Print the usage header and exit. |

**Examples:**

```bash
bash reset_robot.sh --unlock            # locked robot 1: unlock, then reset
bash reset_robot.sh --robot 2 --keep    # reset robot 2, leave the stack running
```

The control PC's raw output goes to `logging/reset/<timestamp>.robotN.control_pc.log`.

---

## `lock_arms.py`

Locks or unlocks the Franka joint brakes via the Desk web API. Runs over ssh on each
control PC. **Locks both robots by default**; pass `--unlock` to open the brakes instead
(the arm will move slightly when unlocked).

Before unlocking, it checks Desk's Settings -> End-Effector and switches it to "Other"
if it is anything else (for example "Franka Hand"). That switch keeps the current mass,
centre of mass, inertia and transform, and it restarts the robot, so the unlock takes a
minute or two longer that one time. Already "Other": nothing changes.

Desk credentials load automatically from `.env.local` (git-ignored) — see the script's
docstring if it's missing on a fresh clone.

**Base command:**

```bash
python3 src/devel_packages/orio_bringup/lock_arms.py
```

| Argument / Option | Description |
| --- | --- |
| `[robots ...]` | Robot numbers to act on (`1`, `2`); default is all robots. |
| `--unlock` | Open the brakes instead of closing them (arm moves slightly). Sets the end effector to "Other" first if needed. |
| `--end-effector-status` | Only print which end effector Desk has selected. Touches nothing. |
| `-h`, `--help` | Show help and exit. |

**Examples:**

```bash
python3 src/devel_packages/orio_bringup/lock_arms.py 1          # lock robot 1
python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1 # unlock robot 1
python3 src/devel_packages/orio_bringup/lock_arms.py --end-effector-status 1   # just report it
```
