#!/usr/bin/env python3
import datetime
import os
import re
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, Int32, String
import serial
import time
import threading

# The firmware's reply to "read_vac" (pneumatic_control/pneumatic_control.ino).
VAC_LINE = re.compile(r"LBL_VAC: (\d+) \| PNP_VAC: (\d+)")
VAC_POLL_PERIOD = 0.1  # s; ~1.5 MB of CSV per hour

try:
    from orio_core.events import TOPIC as EVENTS_TOPIC, EventSink
except ImportError:  # orio_core not on this python's path: no run events, node still works
    EVENTS_TOPIC, EventSink = None, None

class VacuumControlNode:
    def __init__(self):
        rospy.init_node('vacuum_control')

        # Setup serial connection to the ClearCore vacuum controller. Its firmware is
        # flashed once and persists, so no manual re-flash is needed; we just open the
        # port. If the device is missing (unplugged, wrong port, or firmware lost so it
        # never enumerates), fail loudly with an actionable message instead of a bare
        # SerialException.
        port = os.environ.get('ORIO_VACUUM_PORT', '/dev/ttyACM0')
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=1)
        except serial.SerialException as exc:
            rospy.logerr(
                "Cannot open ClearCore vacuum controller on %s: %s. "
                "Check it is plugged in (ls -l %s), on the right port "
                "(set ORIO_VACUUM_PORT), and still flashed with "
                "pneumatic_control/pneumatic_control.ino.", port, exc, port)
            raise
        time.sleep(2.0) # Wait for ClearCore reboot

        # Publishers for modular boolean state tracking
        self.lbl_state_pub = rospy.Publisher('orio/vacuum/lbl_has_item', Bool, queue_size=10)
        self.pnp_state_pub = rospy.Publisher('orio/vacuum/pnp_has_item', Bool, queue_size=10)

        # Raw sensor readings (0-4095), polled with "read_vac". They go to their own CSV
        # (ORIO_VACUUM_LOG, one per run) and these topics, not to the run log: at 10 Hz
        # they would bury it.
        self.lbl_raw_pub = rospy.Publisher('orio/vacuum/lbl_raw', Int32, queue_size=10)
        self.pnp_raw_pub = rospy.Publisher('orio/vacuum/pnp_raw', Int32, queue_size=10)
        self.vac_csv = None
        vac_log = os.environ.get('ORIO_VACUUM_LOG', '')
        if vac_log:
            try:
                self.vac_csv = open(vac_log, 'a', buffering=1)
                if self.vac_csv.tell() == 0:
                    self.vac_csv.write("unix_time,local_time,lbl_raw,pnp_raw\n")
                rospy.loginfo("Raw vacuum readings -> %s", vac_log)
            except OSError as exc:
                rospy.logwarn("Cannot open %s (%s): raw vacuum readings not saved", vac_log, exc)
        # Events are logged on the reading that follows them, so the log shows the value.
        self.pending_events = []
        self.serial_lock = threading.Lock()  # the poll thread and service callbacks both write

        # Run events for the rerun recorder (docs/LOGGING.md); no-op when unavailable or ORIO_LOGGING=0.
        if EventSink and os.environ.get('ORIO_LOGGING', '1') != '0':
            ev_pub = rospy.Publisher(EVENTS_TOPIC, String, queue_size=20)
            self.events = EventSink(lambda s: ev_pub.publish(String(data=s)), src='pneumatics')
        else:
            self.events = EventSink(lambda s: None, src='pneumatics', enabled=False) if EventSink else None

        # Services for Labelling Cup
        rospy.Service('orio/lbl_cup/on', Trigger, self.lbl_on_cb)
        rospy.Service('orio/lbl_cup/off', Trigger, self.lbl_off_cb)

        # Services for PnP Cup
        rospy.Service('orio/pnp_cup/on', Trigger, self.pnp_on_cb)
        rospy.Service('orio/pnp_cup/off', Trigger, self.pnp_off_cb)

        # Global Disable
        rospy.Service('orio/vacuum/disable_all', Trigger, self.disable_all_cb)

        # Start a background thread to continuously read serial data
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        self.send_command("disable")

        self.poll_thread = threading.Thread(target=self.vac_poll_loop, daemon=True)
        self.poll_thread.start()
        rospy.loginfo("Vacuum Control Node Ready")

    def vac_poll_loop(self):
        """Asks the ClearCore for both raw readings every VAC_POLL_PERIOD s."""
        while not rospy.is_shutdown():
            try:
                with self.serial_lock:
                    self.serial_conn.write(b"read_vac\n")
            except Exception as e:
                rospy.logerr(f"Serial write error (read_vac): {e}")
            time.sleep(VAC_POLL_PERIOD)

    def on_vac_reading(self, lbl, pnp):
        self.lbl_raw_pub.publish(lbl)
        self.pnp_raw_pub.publish(pnp)
        if self.vac_csv:
            now = time.time()
            local = datetime.datetime.fromtimestamp(now).strftime('%H:%M:%S.%f')[:-3]
            self.vac_csv.write(f"{now:.3f},{local},{lbl},{pnp}\n")
        while self.pending_events:
            log_fn, text = self.pending_events.pop(0)
            log_fn(f"{text} (raw lbl={lbl} pnp={pnp})")

    def serial_read_loop(self):
        """Continuously monitors the serial port for incoming events."""
        while not rospy.is_shutdown():
            if self.serial_conn.in_waiting > 0:
                try:
                    # Read and decode the line from the ClearCore
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    vac = VAC_LINE.fullmatch(line) if line else None
                    if vac:
                        self.on_vac_reading(int(vac.group(1)), int(vac.group(2)))
                    elif line:
                        # Route events to the correct boolean publisher
                        rospy.loginfo(f"RAW SERIAL READ: '{line}'")
                        if line == "EVENT: LBL Cup Picked Up Item":
                            self.lbl_state_pub.publish(True)
                            self.pending_events.append((rospy.loginfo, "Hardware Event: LBL Cup Picked Up Item"))
                        elif line == "EVENT: LBL Cup Dropped Item":
                            self.lbl_state_pub.publish(False)
                            self.pending_events.append((rospy.logwarn, "Hardware Event: LBL Cup Dropped Item!"))

                        elif line == "EVENT: PNP Cup Picked Up Item":
                            self.pnp_state_pub.publish(True)
                            self.pending_events.append((rospy.loginfo, "Hardware Event: PNP Cup Picked Up Item"))
                        elif line == "EVENT: PNP Cup Dropped Item":
                            self.pnp_state_pub.publish(False)
                            self.pending_events.append((rospy.logwarn, "Hardware Event: PNP Cup Dropped Item!"))

                        # Optionally log command confirmations
                        elif line.startswith("CMD:"):
                            rospy.loginfo(line)

                except Exception as e:
                    rospy.logerr(f"Serial read error: {e}")
            else:
                # Small sleep to prevent this thread from pegging the CPU to 100%
                time.sleep(0.01)

    def send_command(self, command):
        with self.serial_lock:
            self.serial_conn.write((command + '\n').encode('utf-8'))
        if self.events:
            self.events.emit('vacuum_cmd', command)

    def lbl_on_cb(self, req):
        self.send_command("lbl_on")
        return TriggerResponse(success=True, message="Labelling Cup ON command sent")

    def lbl_off_cb(self, req):
        self.send_command("lbl_off")
        return TriggerResponse(success=True, message="Labelling Cup OFF command sent")

    def pnp_on_cb(self, req):
        self.send_command("pnp_on")
        return TriggerResponse(success=True, message="PnP Cup ON command sent")

    def pnp_off_cb(self, req):
        self.send_command("pnp_off")
        return TriggerResponse(success=True, message="PnP Cup OFF command sent")

    def disable_all_cb(self, req):
        self.send_command("disable")
        return TriggerResponse(success=True, message="All Cups OFF command sent")

if __name__ == '__main__':
    try:
        node = VacuumControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
