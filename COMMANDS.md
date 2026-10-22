<!--
  NOTE FOR ANYONE EDITING THIS FILE (people and AI assistants):
  Keep every command description very short and in simple, plain language.
  Say what the command does for the user. Use a few short sentences or bullets.
  Do not explain internals, and do not copy wording from the script's comments.
  Put details in the options table or in docs/, not here.
-->
# Commands Reference

Commonly used commands with usage examples. Run all commands from the repo root.

---

## `run_dexnet_pnp_single.sh`

Starts the suction pick-and-place system in one terminal and picks items from the bin.
Press **Ctrl-C** to stop picking. The cameras, planner and robot connection keep running,
so the next run starts fast. To stop everything, run `bash stop_dexnet_pnp.sh`.
Each run saves a log to `logging/dexnet_pnp/<timestamp>.log`.

Before the first run, build the planner image:

```bash
docker build -t orio/suction:latest - < src/devel_packages/orio_bringup/docker/Dockerfile.suction
```

- When the bin is empty, the arm goes home and asks you to refill it. Press Enter to go on, or `q` to finish.
- Each grasp saves a picture to `logging/dexnet_pnp/<timestamp>.affordance/` showing where it chose to pick and why.
- Each run is also recorded for the `rerun` viewer in `logging/rerun/<timestamp>/` (see `docs/LOGGING.md`).

**Base command:**

```bash
bash run_dexnet_pnp_single.sh
```

| Option | Description |
| --- | --- |
| `--vacuum` | Turn on real suction. |
| `--no-vacuum` | No suction; test run only (default). |
| `--auto` | Pick without asking each time. |
| `--confirm` | Ask before each pick: Enter to pick, `r` to try another grasp (default). |
| `--straight-down` | Always move straight down to the item. |
| `--offset-x M`, `--offset-y M` | Move every pick by this many metres. Use it if the cup always lands off by the same amount. |
| `--min-q Q` | Lowest grasp score (0 to 1) to accept. Default 0.30. |
| `--planner NAME` | `suction` (default) or `dexnet` (see `docs/DEXNET.md`). |
| `--no-affordance` | Do not save the grasp pictures (a bit faster). |
| `--no-logging` | Do not record this run. |
| `--live` | Also show the recording in an open `rerun` viewer. |
| `--fresh` | Stop everything first and start from scratch. |
| `-h`, `--help` | Show help and exit. |

**Examples:**

```bash
bash run_dexnet_pnp_single.sh --vacuum --auto --straight-down
bash run_dexnet_pnp_single.sh --vacuum --offset-y -0.015   # cup lands 1.5 cm too high
```

---

## `stop_dexnet_pnp.sh`

Stops everything `run_dexnet_pnp_single.sh` started: cameras, planner, robot connection
and containers. Turns the suction off first if a run is still going.

```bash
bash stop_dexnet_pnp.sh
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
