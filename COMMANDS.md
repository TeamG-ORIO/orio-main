# Commands Reference

Commonly used commands with usage examples. Run all commands from the repo root.

---

## `run_dexnet_pnp_single.sh`

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
| `--no-logging` | Do not record this run. |
| `--live` | Also stream the recording to a `rerun` viewer already open on this machine. |
| `-h`, `--help` | Print the script's usage header and exit. |

**Example:**

```bash
bash run_dexnet_pnp_single.sh --vacuum --auto --straight-down
```

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
