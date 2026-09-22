# Open issues

Things we've hit and deliberately parked — known problems that still need investigation.

This list is **manually curated**. Entries are added only when someone explicitly asks for
one; nothing is auto-logged here. When an issue is resolved, move the knowledge to
`TROUBLESHOOTING.md` (symptom → cause → fix) and delete the entry here.

Format per entry: a short plain-language description, then possible causes (may be empty
if we don't know yet).

---

## iam-doc control PC keeps crashing — WATCH OUT FOR

**Status: likely fixed, NOT confirmed.** A probable cause was found and fixed on
2026-09-21, but only a handful of runs have happened since. Keep an eye on it; if it
recurs, the fix below was not the whole story.

**What happens.** The machine still responds to ping, but SSH stops working — the connection
opens and then times out during "banner exchange", so nothing can be started on it. (In the
worst instance it stopped answering ping too.) Restarting iam-doc clears it.

**Likely cause (found 2026-09-21).** Our teardown never actually stopped franka-interface,
and then deleted its shared memory while it was still running.

`franka_interface` is 16 characters long, and Linux truncates a process's `comm` name to 15
(`franka_interfac`). The cleanup used `pkill -x franka_interface` and
`ps -eo comm | grep -x franka_interface`, so **both silently matched nothing** — the process
was never killed, and the "wait for it to exit" loop exited immediately believing it was
already gone. The cleanup then removed `/dev/shm/run_loop_*` out from under the still-live
process. The next run's franka-interface recreated those segments while the old one still
held them, and hung on the stale lock
(`Will try to acquire lock while setting franka_interface status`).

Verified on the real binary: `comm` is `franka_interfac`, the old matchers return 0, and
franka-interface never removes its own `/dev/shm` segments even on a clean exit.

**Fixed by.** Matching the full command line (`pgrep -f '[f]ranka_interface --robot_ip'`)
in both the pre-run cleanup and the teardown, confirming the process is really gone, and
removing `/dev/shm` **only** once nothing is running.

**Still unexplained.** This does not account for the machine itself going down. The boot
that died logged only 2 kernel lines and no hung-task traces, despite hang detection being
active — so something killed it faster than it could log. A userspace mutex deadlock should
not take the kernel with it. If the freezes continue, look at the PREEMPT_RT kernel
(`5.4.3-rt1`, `rtprio 99` allowed for `@realtime`) or hardware, not the teardown.

---

## Xtion camera often fails to connect on the first try

**What happens.** At bring-up the Xtion frequently fails to start and `/camera/rgb/image_raw`
never publishes. The driver logs `Can't initialize stream of type 1` (the IR stream) and then
`Device "1d27/0600@1/NN" disconnected`, with `NN` incrementing each attempt. The launcher
retries and it usually succeeds within a few attempts, so runs still proceed.

**Possible causes.** Not established. Two theories were investigated and ruled out: USB
autosuspend (the device reports `power/control=on` and `runtime_suspended_time=0`, so it is
never suspended) and USB bandwidth contention with the ZED (the Xtion is the only high-speed
device on its bus, and staggering the two cameras did not stop it). The failure is always the
IR stream specifically, and a retry usually works, which points at the cable, power delivery,
or the camera itself rather than software — untested.
