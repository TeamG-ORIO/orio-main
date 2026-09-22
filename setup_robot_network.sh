#!/usr/bin/env bash
# Configure this workstation's two robot-network NICs to reach the Franka control PCs.
#   USB dongle enx00e04c4d23a8 -> robot-1  (iam-doc = 192.168.1.1);  this PC = 192.168.1.2
#   eno1 (built-in)            -> robot-2  (iam-luisa = 192.168.2.3); this PC = 192.168.2.1 (ROS master)
# Run once with sudo:  sudo bash setup_robot_network.sh
set -euo pipefail

USB_IF="enx00e04c4d23a8"
ENO1_CON="Wired connection 1"   # bound to eno1

echo "== robot-2 on eno1 (iam-luisa): 192.168.2.1/24 =="
nmcli con mod "$ENO1_CON" ipv4.method manual ipv4.addresses 192.168.2.1/24 ipv4.gateway "" ipv4.never-default yes
nmcli con up "$ENO1_CON"

echo "== robot-1 on $USB_IF (iam-doc): 192.168.1.2/24 =="
# Remove any stale/auto profile on the dongle so ours is the only one.
while IFS= read -r c; do [ -n "$c" ] && nmcli con delete "$c" || true; done < <(
  nmcli -t -f NAME,DEVICE con show 2>/dev/null | awk -F: -v d="$USB_IF" '$2==d{print $1}'
)
nmcli con delete robot1-doc >/dev/null 2>&1 || true
nmcli con add type ethernet ifname "$USB_IF" con-name robot1-doc \
    ipv4.method manual ipv4.addresses 192.168.1.2/24 ipv4.never-default yes
nmcli con up robot1-doc

echo "== /etc/hosts (name resolution to the control PCs) =="
grep -qE '^[^#]*\biam-doc\b'   /etc/hosts || printf '192.168.1.1\tiam-doc\n'   >> /etc/hosts
grep -qE '^[^#]*\biam-luisa\b' /etc/hosts || printf '192.168.2.3\tiam-luisa\n' >> /etc/hosts

echo
echo "== resulting interfaces =="
ip -br addr show eno1
ip -br addr show "$USB_IF"
echo
echo "== reachability test =="
ping -c1 -W2 192.168.1.1 >/dev/null 2>&1 && echo "iam-doc  (192.168.1.1): REACHABLE" || echo "iam-doc  (192.168.1.1): NO REPLY (is iam-doc powered on?)"
ping -c1 -W2 192.168.2.3 >/dev/null 2>&1 && echo "iam-luisa (192.168.2.3): REACHABLE" || echo "iam-luisa (192.168.2.3): NO REPLY"
