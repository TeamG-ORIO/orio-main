#!/usr/bin/env bash
# Enable crash + freeze capture on the iam-doc control PC.
#
# WHAT WE'RE CHASING
# iam-doc intermittently becomes unusable: it still answers ping, but ssh connects and then
# times out during banner exchange, and only a power cycle recovers it. The journal from
# such a boot just STOPS mid-line with no shutdown sequence, and sshd logs nothing at all
# during the incident — so this is the WHOLE MACHINE freezing, not an application crash.
# iam-doc runs a PREEMPT_RT kernel (5.4.3-rt1) with rtprio 99 allowed for @realtime, and
# franka-interface is the only realtime workload on it, which makes an RT-priority stall
# the leading suspect.
#
# WHAT THIS SETS UP
#   A. Kernel hung-task + watchdog detection — so a stall is DETECTED and logged (with
#      backtraces of the stuck tasks) instead of passing silently.
#   B. Core dumps for franka-interface — for the separate case where it crashes outright
#      (we saw a std::bad_cast) rather than taking the box down.
#   C. Persistent, generous journald — so evidence survives the power cycle.
#
# RUN THIS ON iam-doc (needs sudo):
#     ssh student@iam-doc
#     bash enable_crash_logging.sh
#     sudo reboot            # cleanest way to have everything active
#
# AFTER THE NEXT FREEZE (once it's back up):
#     journalctl -b -1 -k --no-pager | grep -iE "hung task|blocked for more than|watchdog|BUG|Call Trace" -A 25
#     journalctl -b -1 --no-pager | tail -50      # what it was doing at the end
#     coredumpctl list                            # if franka-interface crashed instead
# NOT `set -e`: several steps here are best-effort (an optional package that may not be
# installable, apt repos with expired keys). One of those failing must not skip the parts
# that actually matter — kernel hang detection and persistent logging. Each step reports
# its own success or failure instead.
set -uo pipefail

echo "=== Enabling crash + freeze capture on $(hostname) ==="

# ── A. Detect a system freeze ────────────────────────────────────────────────
# hung_task_timeout_secs : log a backtrace for any task blocked in D state this long.
#     This is the one most likely to name the culprit in an RT stall.
# nmi_watchdog / soft+hard lockup: catch a CPU spinning without yielding.
# panic_on_* = 0: we LOG rather than auto-reboot, so the evidence stays readable on the
#     console and (if anything is still schedulable) reaches the journal. Set these to 1
#     only if you'd rather have the machine recover itself unattended.
echo "--- configuring kernel hang/lockup detection ---"
sudo mkdir -p /etc/sysctl.d
sudo tee /etc/sysctl.d/90-orio-hangdetect.conf >/dev/null <<'EOF'
# Log a stack trace for tasks stuck in uninterruptible sleep for 30s+.
kernel.hung_task_timeout_secs = 30
kernel.hung_task_warnings = 99999
# Watchdogs for a CPU that stops yielding (the RT-starvation case).
kernel.nmi_watchdog = 1
kernel.watchdog = 1
kernel.softlockup_panic = 0
kernel.hardlockup_panic = 0
# Allow SysRq so a wedged console can still force output/reboot:
#   Alt+SysRq+w  = dump blocked tasks   Alt+SysRq+l = dump CPU backtraces
#   Alt+SysRq+t  = dump all tasks       Alt+SysRq+b = immediate reboot
kernel.sysrq = 1
EOF
sudo sysctl --system >/dev/null 2>&1 || true

# ── B. Core dumps for an outright crash ──────────────────────────────────────
# franka_interface is a locally BUILT binary, so apport ignores it by default; and the
# core limit is 0, so nothing is written. systemd-coredump handles non-packaged binaries
# and gives us coredumpctl. (This does NOT help with a full system freeze — nothing can
# dump core then — it is for the std::bad_cast style crash.)
# NOTE: this step is OPTIONAL and is allowed to fail. It cannot catch a whole-system
# freeze (nothing dumps core when the kernel can't schedule) — it only helps for an
# outright franka-interface crash. On iam-doc it currently fails anyway: systemd-coredump
# depends on an exact systemd version that is held back, and some apt repos have expired
# GPG keys. So we try, shrug, and carry on to the parts that matter.
if ! command -v coredumpctl >/dev/null 2>&1; then
    echo "--- trying to install systemd-coredump (optional) ---"
    if sudo apt-get update -qq 2>/dev/null && sudo apt-get install -y systemd-coredump 2>/dev/null; then
        echo "    installed."
    else
        echo "    SKIPPED: could not install (version conflict / apt repo issues)."
        echo "    Not fatal — core dumps only help for an outright crash, not a freeze."
        echo "    Fallback: franka-interface output is still saved on this host in"
        echo "    ~/franka_logs/, and kernel hang traces go to the journal."
    fi
else
    echo "--- systemd-coredump already installed ---"
fi

echo "--- raising core dump limit ---"
if ! grep -q "^\* soft core unlimited" /etc/security/limits.conf 2>/dev/null; then
    echo '* soft core unlimited' | sudo tee -a /etc/security/limits.conf >/dev/null
fi
sudo mkdir -p /etc/systemd/system.conf.d
printf '[Manager]\nDefaultLimitCORE=infinity\n' \
    | sudo tee /etc/systemd/system.conf.d/90-core.conf >/dev/null

sudo mkdir -p /etc/systemd/coredump.conf.d
sudo tee /etc/systemd/coredump.conf.d/90-orio.conf >/dev/null <<'EOF'
[Coredump]
Storage=external
Compress=yes
MaxUse=5G
KeepFree=20G
EOF

# ── C. Keep the evidence across the power cycle ──────────────────────────────
# A freeze is only diagnosable if the log survives the hard reset that follows it.
echo "--- configuring journald (persistent, flush often) ---"
sudo mkdir -p /etc/systemd/journald.conf.d
sudo tee /etc/systemd/journald.conf.d/90-orio.conf >/dev/null <<'EOF'
[Journal]
Storage=persistent
SystemMaxUse=2G
MaxRetentionSec=1month
# Write through rather than batching: a frozen machine never gets to flush, so buffered
# lines would be lost on the power cycle. Costs a little I/O, buys us the last words.
SyncIntervalSec=10s
EOF
sudo systemctl restart systemd-journald

echo
echo "=== Summary of what is now active ==="
printf '  hung_task_timeout_secs : %s  (want 30)\n'    "$(sysctl -n kernel.hung_task_timeout_secs 2>/dev/null)"
printf '  hung_task_warnings     : %s  (want 99999)\n' "$(sysctl -n kernel.hung_task_warnings 2>/dev/null)"
printf '  nmi_watchdog           : %s  (want 1)\n'     "$(sysctl -n kernel.nmi_watchdog 2>/dev/null)"
printf '  sysrq                  : %s  (want 1)\n'     "$(sysctl -n kernel.sysrq 2>/dev/null)"
printf '  journald storage       : %s\n' \
    "$([ -f /etc/systemd/journald.conf.d/90-orio.conf ] && echo 'persistent (configured)' || echo 'NOT configured')"
printf '  coredumpctl            : %s\n' \
    "$(command -v coredumpctl >/dev/null 2>&1 && echo available || echo 'not installed (optional)')"
echo
echo "REBOOT to have everything active:  sudo reboot"
echo
echo "After the next freeze, once it is back up:"
echo "  journalctl -b -1 -k --no-pager | grep -iE 'hung task|blocked for more than|watchdog|BUG|Call Trace' -A 25"
echo "  journalctl -b -1 --no-pager | tail -50"
