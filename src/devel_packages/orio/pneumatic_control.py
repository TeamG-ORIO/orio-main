#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import serial
import time

class VacuumControlNode:
    def __init__(self):
        rospy.init_node('vacuum_control')
        
        # Setup serial connection
        self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2.0) # Wait for ClearCore reboot

        # Services for Labelling Cup
        rospy.Service('orio/lbl_cup/on', Trigger, self.lbl_on_cb)
        rospy.Service('orio/lbl_cup/off', Trigger, self.lbl_off_cb)

        # Services for PnP Cup
        rospy.Service('orio/pnp_cup/on', Trigger, self.pnp_on_cb)
        rospy.Service('orio/pnp_cup/off', Trigger, self.pnp_off_cb)

        # Global Disable
        rospy.Service('orio/vacuum/disable_all', Trigger, self.disable_all_cb)

        self.send_command("disable")
        rospy.loginfo("Vacuum Control Node Ready")
        
    def send_command(self, command):
        self.serial_conn.write((command + '\n').encode('utf-8'))

    def lbl_on_cb(self, req):
        self.send_command("lbl_on")
        return TriggerResponse(success=True, message="Labelling Cup ON")

    def lbl_off_cb(self, req):
        self.send_command("lbl_off")
        return TriggerResponse(success=True, message="Labelling Cup OFF")

    def pnp_on_cb(self, req):
        self.send_command("pnp_on")
        return TriggerResponse(success=True, message="PnP Cup ON")

    def pnp_off_cb(self, req):
        self.send_command("pnp_off")
        return TriggerResponse(success=True, message="PnP Cup OFF")

    def disable_all_cb(self, req):
        self.send_command("disable")
        return TriggerResponse(success=True, message="All Cups OFF")

if __name__ == '__main__':
    try:
        node = VacuumControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass