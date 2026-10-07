# Troubleshooting log

A running list of environment / hardware / setup issues that are **not** self-evident
from the code or a git commit — machine-specific gotchas, USB/driver quirks, robot
states, and the like. Ordinary code fixes live in git history; this file is for the
"why did the machine behave that way, and what fixed it" knowledge that would otherwise
be lost.

Newest entries on top. Keep each entry short: **symptom → cause → fix**.

---

## Diagnosing an iam-doc freeze / crash (where the evidence lives)

**Two different failures, don't confuse them:**

- **Whole-machine freeze** — iam-doc answers ping but ssh times out at banner exchange, and
  only a power cycle recovers it. Evidence: the journal for that boot **stops mid-line with
  no shutdown sequence**, and sshd logs *nothing at all* during the incident (it never got
  CPU to respond). This is the system hanging, not an app crashing, so **core dumps cannot
  catch it** — nothing can dump core when the kernel can't schedule. Suspicion falls on the
  PREEMPT_RT kernel (`5.4.3-rt1`, `rtprio 99` allowed) plus franka-interface being the only
  realtime workload, i.e. an RT-priority stall starving userspace. Not yet confirmed.
- **franka-interface crashing on its own** — e.g. the observed `std::bad_cast`. The machine
  stays up; only the controller dies. *This* is what core dumps catch.

franka-interface crashes had also been leaving **no trace on iam-doc**: core dumps were off
(`ulimit -c` = 0), apport ignores it (locally built, non-packaged binary), and it runs from
an interactive ssh shell rather than a service, so nothing captured its output. These places
now record things:

| Where | What it holds | Notes |
|---|---|---|
| `logging/dexnet_pnp/stack/control_pc.log` (workstation) | franka-interface + action-server output, one banner per start | written by the launcher's ssh tap; the control PC stays up between runs |
| `~/franka_logs/*.log` (**on iam-doc**) | the same output, saved locally | survives the launcher dying / terminal closing; last 50 runs kept |
| `coredumpctl` (**on iam-doc**) | core dump + backtrace of a franka-interface crash | needs the one-time setup below |
| `journalctl -b -1 -k` (**on iam-doc**) | kernel hung-task / lockup traces from a **freeze** | needs the setup below; look at the *previous* boot |

**One-time setup on iam-doc** (kernel hang detection, core dumps, persistent journald):

```bash
ssh student@iam-doc
bash enable_crash_logging.sh     # from orio_bringup/control_pc/, needs sudo
sudo reboot                      # cleanest way to have it all active
```

It lowers `hung_task_timeout_secs` 120 → 30 (catch stalls sooner), raises
`hung_task_warnings` 10 → unlimited (a cascade otherwise truncates at ten, losing the
cause), enables the lockup watchdogs, fully enables SysRq, and makes journald sync every
10s so the last lines survive the power cycle.

**After a freeze** (once it's back up — note `-b -1` = the boot that died):

```bash
journalctl -b -1 -k --no-pager | grep -iE "hung task|blocked for more than|watchdog|BUG|Call Trace" -A 25
journalctl -b -1 --no-pager | tail -50
```

At a frozen console, SysRq still works: **Alt+SysRq+w** dumps blocked tasks,
**Alt+SysRq+l** dumps CPU backtraces, **Alt+SysRq+b** reboots immediately.

**After a franka-interface crash** (machine still up):

```bash
coredumpctl list                     # crashes, newest last
coredumpctl info franka_interface    # signal, faulting address, backtrace
coredumpctl gdb franka_interface     # full interactive backtrace (needs gdb)
tail -50 ~/franka_logs/$(ls -t ~/franka_logs | head -1)
```

---

## Bring-up hangs at pre-flight; control_pc.log shows ssh "banner exchange" timeout

**Symptom.** The launcher stops at `pre-flight: checking robot 1 is ready…` and hangs (had
to Ctrl-C). `control_pc.log` shows `Connection timed out during banner exchange` /
`Connection to <iam-doc ip> port 22 timed out`. `ping iam-doc` **succeeds** (host is up),
but ssh never completes.

**Cause.** iam-doc's **sshd is wedged**: TCP to port 22 connects, but the daemon never
sends its version banner, so no ssh session (and thus no franka-interface / action server)
can start. The pre-flight then blocks constructing `FrankaArm` (frankapy's
`wait_for_franka_interface` waits on a controller that was never launched). Typically
follows a reboot / power event (e.g. moving a USB cable) that left iam-doc half-booted, or
sshd exhausted after many rapid ssh sessions.

**Diagnose.** `ping iam-doc` (up?) + `nc -vz iam-doc 22` / a raw connect that opens but
returns an empty banner ⇒ sshd wedged, not a network problem.

**Fix.** Recover ssh on iam-doc: reboot it, or at its console `sudo systemctl restart ssh`.
`run_dexnet_pnp_single.sh` now probes ssh first and **aborts fast** with this guidance
instead of hanging at pre-flight.

---

## Arm hangs after "Descending to contact" (motion fault silently swallowed)

**Symptom.** The pick loop prints `[Pick] Descending to contact` and the arm then just sits
there — no motion, no error, no timeout — until you Ctrl-C. The `control_pc.log` shows the
descent skill hit `Caught Franka Exception` /
`Motion finished commanded, but the robot is still moving!
["joint_motion_generator_acceleration_discontinuity"]`, then `Performing automatic error
recovery` / `franka_interface status is not ready`.

**Cause.** Two layers:
1. *Why it faulted:* the descent `goto_joints` was issued ~4 ms after the previous move
   reported "Succeeded", while the arm was likely still settling — the trajectory
   generator saw an acceleration jump beyond limits and tripped the reflex.
2. *Why it hung instead of erroring:* `dexnet_pnp.py`'s `_move()` polled
   `is_skill_done()` with the frankapy default `ignore_errors=True`, which **swallows** a
   controller fault (it just waits for franka-interface to become ready again) and then
   reports the skill "done". So the pick continued as if the descent succeeded; the next
   command landed while the controller was still "not ready" and was silently dropped →
   the arm sat idle. The `_move()` timeout never fired because the skill *did* report done.

**Fix.** `_move()` now calls `is_skill_done(ignore_errors=False)`, which **raises** on a
fault; `_move()` re-raises a clear `RuntimeError`, and the loop's per-pick `try/except`
logs it, drops the cup, and re-scans instead of wedging. If the acceleration-discontinuity
fault recurs often, the deeper fix is to add a small settle delay between consecutive
`goto_joints` (or use a spline/continuous motion) so the trajectory start isn't
discontinuous — not yet done.

---

## ZED launch dies at bring-up: `zedm_state_publisher has died` (takes cameras down)

**Symptom.** Cameras appear to start — ZED `Camera successfully opened`, Xtion publishing
`/camera/rgb/image_raw` — then `REQUIRED process [zedm/zedm_state_publisher-1] has died!` /
`process has finished cleanly`, and roslaunch tears down the **whole** `cameras.launch`
(both ZED and Xtion). `/zedm/zed_node/rgb/image_rect_color` never publishes; the launcher
`GAVE UP`. Intermittent — "rerunning usually works."

**Cause.** `zed_wrapper`'s `zed_camera.launch.xml` includes a `robot_state_publisher`
node (`<camera>_state_publisher`) marked **`required="true"`**, gated on `publish_urdf`
(default true). It needs the ZED `robot_description` URDF, which races to be set; if it
comes up with nothing to publish it exits cleanly (exit 0), and because it is `required`,
roslaunch kills every other node in the launch — including the perfectly healthy cameras.
Racy, so a retry sometimes wins. This is **not** a camera-hardware or autosuspend fault.

**Fix.** We don't use the ZED's URDF/TF for grasping (only its image/depth topics), so
`manipulation/cameras.launch` now passes `publish_urdf:=false` to the `zedm.launch`
include. That drops the whole `<group if="publish_urdf">` — URDF param **and** the
required state_publisher — so nothing in it can die and take the launch down.

---

## Xtion flaps at bring-up: USB bus contention with the ZED (primary cause)

**Symptom.** Cameras fail to start intermittently. `[cameras]` logs show
`Couldn't create IR video stream`, then `Device "1d27/0600@1/NN" disconnected`, with the
device number `@1/NN` incrementing each run, and often the openni2 warning
`Reconnect has been enabled, only one camera should be plugged into each bus`. The failure
lands *while the ZED is initialising* (e.g. downloading its calibration). Re-running works.

**Cause.** The ZED-M's HID interface (`2b03:f681`) shares **USB Bus 001** with the Xtion
(`1d27:0600`). When `cameras.launch` starts both cameras at once, they contend for Bus 001
bandwidth; the Xtion's IR stream (the most bandwidth-hungry) loses, errors, and the openni2
driver marks the device disconnected and re-enumerates. Re-running works because by then the
ZED is already up and idle, so the bandwidth burst is gone. Verified: the Xtion's
`power/control` is `on` and `runtime_suspended_time` is `0` when this happens — i.e. it is
**not** being autosuspended, so this is contention, not autosuspend (see next entry).

**Fix.**
- **Software (in place):** `run_dexnet_pnp_single.sh` now **staggers** startup — brings the
  ZED up first (`zed_only.launch`), lets the bus settle, then starts the Xtion
  (`xtion_only.launch`) on a quiet bus, with a retry (`ORIO_CAM_ATTEMPTS`) as fallback.
- **Hardware (permanent root fix):** plug the ZED-M into a port on a **different USB
  controller/bus** than the Xtion, so nothing shares Bus 001. Check with `lsusb` (compare
  the `Bus` numbers) and `lsusb -t` (tree). Then contention can't happen at all.

---

## Xtion autosuspend (secondary / historical cause)

**Symptom.** Same flap signature as above, but occurring even with the cameras staggered,
and `cat /sys/bus/usb/devices/<X>/power/runtime_suspended_time` is **non-zero** for the
Xtion (it was actually suspended).

**Cause.** Linux USB autosuspend (2s idle timer on this host) suspends the Xtion
*mid-startup*, before the driver has opened all its video streams; the half-open device
errors and re-enumerates.

**Fix.** Disable autosuspend for the Xtion (`1d27:0600`), permanent + one-time:

```bash
bash src/devel_packages/orio_bringup/udev/install_xtion_udev.sh   # udev rule, needs sudo
```

Verify it took: `power/control` should read `on` and `runtime_suspended_time` stay `0`.
(As of this writing the rule is installed and the Xtion is not being suspended — the
bus-contention entry above is the active cause of the flap.)

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
