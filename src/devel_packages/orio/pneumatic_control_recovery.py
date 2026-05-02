#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
import serial
import time
import threading

class VacuumControlNode:
    def __init__(self):
        rospy.init_node('vacuum_control')
        
        # Setup serial connection
        self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2.0) # Wait for ClearCore reboot

        # Publishers for modular boolean state tracking
        self.lbl_state_pub = rospy.Publisher('orio/vacuum/lbl_has_item', Bool, queue_size=10)
        self.pnp_state_pub = rospy.Publisher('orio/vacuum/pnp_has_item', Bool, queue_size=10)

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
        rospy.loginfo("Vacuum Control Node Ready")
        
    def serial_read_loop(self):
        """Continuously monitors the serial port for incoming events."""
        while not rospy.is_shutdown():
            if self.serial_conn.in_waiting > 0:
                try:
                    # Read and decode the line from the ClearCore
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        # Route events to the correct boolean publisher
                        rospy.loginfo(f"RAW SERIAL READ: '{line}'")
                        if line == "EVENT: LBL Cup Picked Up Item":
                            self.lbl_state_pub.publish(True)
                        elif line == "EVENT: LBL Cup Dropped Item":
                            self.lbl_state_pub.publish(False)
                            rospy.logwarn("Hardware Event: LBL Cup Dropped Item!")

                        elif line == "EVENT: PNP Cup Picked Up Item":
                            self.pnp_state_pub.publish(True)
                        elif line == "EVENT: PNP Cup Dropped Item":
                            self.pnp_state_pub.publish(False)
                            rospy.logwarn("Hardware Event: PNP Cup Dropped Item!")

                        # Optionally log command confirmations
                        elif line.startswith("CMD:"):
                            rospy.loginfo(line)
                            
                except Exception as e:
                    rospy.logerr(f"Serial read error: {e}")
            else:
                # Small sleep to prevent this thread from pegging the CPU to 100%
                time.sleep(0.01)

    def send_command(self, command):
        self.serial_conn.write((command + '\n').encode('utf-8'))

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