# Troubleshooting log

A running list of environment / hardware / setup issues that are **not** self-evident
from the code or a git commit — machine-specific gotchas, USB/driver quirks, robot
states, and the like. Ordinary code fixes live in git history; this file is for the
"why did the machine behave that way, and what fixed it" knowledge that would otherwise
be lost.

Newest entries on top. Keep each entry short: **symptom → cause → fix**.

---

## Xtion camera flaps / disconnects at bring-up (USB autosuspend)

**Symptom.** Cameras fail to start intermittently. `[cameras]` logs show
`Couldn't create IR video stream`, then `Device "1d27/0600@1/NN" disconnected`, with the
device number `@1/NN` incrementing each run. Perception then hangs waiting for
`/camera/rgb/image_raw`. Re-running usually works.

**Cause.** Linux USB autosuspend (2s idle timer on this host) suspends the Xtion
*mid-startup*, before the driver has opened all its video streams. The half-open device
then errors and re-enumerates. Re-running works only because a freshly enumerated device
sometimes wins the race against the timer.

**Fix.** Disable autosuspend for the Xtion (`1d27:0600`). Permanent, one-time:

```bash
bash src/devel_packages/orio_bringup/udev/install_xtion_udev.sh   # udev rule, needs sudo
```

`run_dexnet_pnp_single.sh` also retries the camera launch a few times as a fallback
(`ORIO_CAM_ATTEMPTS`), but the udev rule is the real fix.

---

## DexNet planner container: `No module named 'rospy'`

**Symptom.** `orio_dexnet` crash-loops with `ModuleNotFoundError: No module named 'rospy'`
(and later `sensor_msgs`). Perception then never gets `/dexnet_grasp_planner/plan_grasp`.

**Cause.** `run_dexnet.sh` historically mounted the host's `/opt/ros/noetic` into the
container for rospy. This host is 22.04 / ROS 2 with an **empty** `/opt/ros/noetic`, so
nothing was mounted.

**Fix.** rospy + std/sensor/geometry_msgs are now installed **into** the `orio/dexnet`
image (`Dockerfile.dexnet`), and the host mount was dropped from `run_dexnet.sh`. Rebuild
the image if you ever see this again:
`docker build -f src/devel_packages/orio_bringup/docker/Dockerfile.dexnet -t orio/dexnet:latest src/devel_packages`.

---

## Robot hangs forever on a motion (control PC crash)

**Symptom.** The pick loop reaches `ros initialized` and then hangs indefinitely on
`reset_joints()` / a `goto_joints`, with no error. `/robot_state_publisher_node_1/robot_state`
shows `no new messages`.

**Cause.** The control PC (iam-doc) crashed / franka-interface stopped executing, so the
robot never reports the skill as done. frankapy's `wait_for_skill()` busy-loops with **no
timeout**, so a blocking motion never returns. (Note: unlocking the arm's brakes is not
enough — the **FCI must be active** and franka-interface running.)

**Fix.**
- Restart the control PC / franka-interface, and confirm FCI is active in Franka Desk
  (blue state), not merely brakes-off.
- `dexnet_pnp.py` now wraps every motion in `_move()` with a hard timeout, so a dead
  control PC raises a clear error instead of hanging.

---

## Perception crashes on startup with `ORIO_LOGGING=1` (rerun on Python 3.8)

**Symptom.** Perception exits during import with
`TypeError: 'type' object is not subscriptable` from `rerun_bindings/types.py`
(`VectorLike = Union[..., list[float]]`).

**Cause.** The `rerun` library uses 3.9+ typing syntax (`list[float]`), but the perception
image runs Python 3.8. The import only happens when logging is enabled.

**Fix (workaround).** Run with `ORIO_LOGGING=0` (default in the `orio_dexnet_pnp` layout
and in `run_dexnet_pnp_single.sh`). Proper fix pending: pin/patch a py38-compatible rerun
in the perception image, then logging can be turned back on.
