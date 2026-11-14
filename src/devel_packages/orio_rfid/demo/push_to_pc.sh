#!/usr/bin/env bash
# push_to_pc.sh - copy what the PC side of the scan demo needs from this checkout to a PC that has no checkout.
#
#   demo/push_to_pc.sh <user@host> [folder on the PC, default ~/orio_scan]
#
# Copies orio_rfid/{demo,sync,loc} and the few files of orio_rfid/RFID the PC uses (tag registry, the pose-log
# readers) into <folder>/orio_rfid, and nothing else. A demo.conf that is already on the PC is kept, because it
# holds that PC's own settings (PYTHON, DB). Then, on the PC:  <folder>/orio_rfid/demo/run_scan_demo.sh
# A PC with a checkout of orio-main does not need this: run demo/run_scan_demo.sh from the checkout.
set -euo pipefail
[ $# -ge 1 ] || { sed -n '2,9p' "$0"; exit 1; }
PC="$1"; DIR="${2:-orio_scan}"; DIR="${DIR/#\~\//}"
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ssh -o BatchMode=yes "$PC" "mkdir -p $DIR/orio_rfid/RFID"
rsync -a --delete --exclude __pycache__ --exclude demo.conf "$PKG/demo" "$PKG/sync" "$PKG/loc" "$PC:$DIR/orio_rfid/" \
      --exclude 'sync/*.db' --exclude 'sync/runs' --exclude 'sync/HANDOFF.md'
rsync -a --ignore-existing "$PKG/demo/demo.conf" "$PC:$DIR/orio_rfid/demo/demo.conf"
rsync -a "$PKG/RFID/tag_registry.csv" "$PKG/RFID/rfid_pose_live.py" "$PKG/RFID/rfid_pose_join.py" "$PC:$DIR/orio_rfid/RFID/"
rsync -a "$PKG/seed_medical_inventory.db" "$PC:$DIR/orio_rfid/"
echo "copied to $PC:$DIR/orio_rfid  ($(ssh -o BatchMode=yes "$PC" "du -sh $DIR/orio_rfid | cut -f1"))"
echo "on the PC: edit $DIR/orio_rfid/demo/demo.conf if needed (PYTHON, DB, addresses), then run $DIR/orio_rfid/demo/run_scan_demo.sh"
