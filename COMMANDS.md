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

## `lock_arms.py`

Locks or unlocks the Franka joint brakes via the Desk web API. Runs over ssh on each
control PC. **Locks both robots by default**; pass `--unlock` to open the brakes instead
(the arm will move slightly when unlocked). Credentials are read from the
`ORIO_DESK_USER` / `ORIO_DESK_PASSWORD` environment variables, otherwise you are prompted.

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
