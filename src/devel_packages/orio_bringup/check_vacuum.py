#!/usr/bin/env python3
# Verify the ClearCore answers over serial before pneumatic_control starts.
# Run with no node holding the port; exits 0 only if the board acks a pulse.
#   python3 check_vacuum.py [--port /dev/ttyACM0] [--cup pnp|lbl]
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("FAIL: no 'serial' module - run with the interpreter that has pyserial "
             "(anaconda base: /home/student/anaconda3/bin/python3)")

# Firmware echoes "CMD: ..." for every command it accepts.
ACKS = {
    "disable": "ALL Cups DISABLED",
    "pnp_on": "PNP Cup ON",
    "pnp_off": "PNP Cup OFF",
    "lbl_on": "LBL Cup ON",
    "lbl_off": "LBL Cup OFF",
}


def drain(conn, seconds):
    """Collect lines for a fixed window; returns them and prints EVENT//vac lines."""
    lines = []
    end = time.time() + seconds
    while time.time() < end:
        if conn.in_waiting > 0:
            line = conn.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        else:
            time.sleep(0.01)
    return lines


def expect(conn, command, timeout=2.0):
    """Send a command and wait for its CMD: ack. Returns (ok, stray_lines)."""
    want = ACKS[command]
    conn.reset_input_buffer()
    conn.write((command + "\n").encode("utf-8"))
    stray = []
    end = time.time() + timeout
    while time.time() < end:
        if conn.in_waiting > 0:
            line = conn.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line == "CMD: " + want:
                print(f"  {command:8s} -> {line}")
                return True, stray
            stray.append(line)
        else:
            time.sleep(0.01)
    print(f"  {command:8s} -> NO ACK (expected 'CMD: {want}')")
    return False, stray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--cup", default="pnp", choices=["pnp", "lbl"])
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to keep the cup on")
    args = ap.parse_args()

    print(f"[check_vacuum] opening {args.port}")
    try:
        conn = serial.Serial(args.port, 115200, timeout=1)
    except serial.SerialException as e:
        print(f"FAIL: cannot open {args.port}: {e}")
        print("  Another process may hold it (pneumatic_control already running?),")
        print("  or the ClearCore is unplugged.")
        return 1

    with conn:
        time.sleep(2.0)  # ClearCore reboots when the port opens
        conn.reset_input_buffer()

        ok, _ = expect(conn, "disable")
        if not ok:
            print("FAIL: board did not ack 'disable'. Is the firmware flashed?")
            return 1

        # Raw sensor values, for spotting an unplugged/miswired vacuum line.
        conn.write(b"read_vac\n")
        for line in drain(conn, 0.5):
            if line.startswith("LBL_VAC:"):
                print(f"  sensors  -> {line}")

        print(f"[check_vacuum] pulsing {args.cup} cup for {args.hold}s")
        ok_on, _ = expect(conn, f"{args.cup}_on")
        events = drain(conn, args.hold)
        ok_off, _ = expect(conn, f"{args.cup}_off")

        for line in events:
            if line.startswith("EVENT:"):
                print(f"  event    -> {line}")

        expect(conn, "disable")  # leave the cups off

        if not (ok_on and ok_off):
            print("FAIL: board did not ack the pulse.")
            return 1

    print("[check_vacuum] OK - board acked every command.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
