#!/usr/bin/env bash
# Install the Xtion no-autosuspend udev rule (fixes the intermittent camera flapping at
# bring-up — see 90-xtion-no-autosuspend.rules). Needs sudo. Run once per machine.
#
#   bash install_xtion_udev.sh
#
# After this, unplug/replug the Xtion (or reboot) so the rule takes effect, or the script
# also applies it to the currently-connected device immediately.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE="90-xtion-no-autosuspend.rules"
DEST="/etc/udev/rules.d/$RULE"

echo "Installing $RULE to $DEST (sudo)…"
sudo cp "$HERE/$RULE" "$DEST"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=1d27

# Also apply immediately to any already-connected Xtion (udevadm trigger covers this, but
# set power/control directly too in case the device was mid-suspend).
for d in /sys/bus/usb/devices/*/idVendor; do
    if [ "$(cat "$d" 2>/dev/null)" = "1d27" ]; then
        dev="$(dirname "$d")"
        echo "on" | sudo tee "$dev/power/control" >/dev/null 2>&1 || true
        echo "Applied to connected device: $dev"
    fi
done

echo "Done. If the camera was mid-flap, unplug/replug it once."
