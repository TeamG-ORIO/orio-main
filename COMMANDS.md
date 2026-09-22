# Commands Reference

Commonly used commands with usage examples. Run all commands from the repo root.

---

## `run_dexnet_pnp_single.sh`

Brings up the whole DexNet pick-and-place stack in a **single terminal**. Every
service runs in the background with its output merged into your terminal (each line tagged
`[module]`), while the pick loop runs in the foreground so its "Press Enter to pick" prompt
can read the keyboard. Press **Ctrl-C** to stop everything (services are killed and
containers removed). A full log of each run is saved to `logging/dexnet_pnp/<timestamp>.log`.

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

Desk credentials load automatically from `.env.local` (git-ignored) — see the script's
docstring if it's missing on a fresh clone.

**Base command:**

```bash
python3 src/devel_packages/orio_bringup/lock_arms.py
```

| Argument / Option | Description |
| --- | --- |
| `[robots ...]` | Robot numbers to act on (`1`, `2`); default is all robots. |
| `--unlock` | Open the brakes instead of closing them (arm moves slightly). |
| `-h`, `--help` | Show help and exit. |

**Examples:**

```bash
python3 src/devel_packages/orio_bringup/lock_arms.py 1          # lock robot 1
python3 src/devel_packages/orio_bringup/lock_arms.py --unlock 1 # unlock robot 1
```
