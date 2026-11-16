#!/usr/bin/env python3
# RFID label node: /inventory/add_item reads the new label's tag with the Mercury reader and
# saves it on this PC only (master_sync.py: master_sync/labels.db, as Item 1001, 1002, ...).
# The inventory window (inventory_dashboard.py) pushes saved labels to abuela's master inventory.
#   python3 db_manager.py
import rospy
import threading

import mercury

from custom_msgs.srv import AddLabeledItem, AddLabeledItemResponse

import master_sync

# --- CONFIGURATION ---
MERCURY_PORT = "tmr:///dev/ttyUSB0"
MERCURY_REGION = "NA"
MERCURY_POWER = 1000   # centi-dBm (10 dBm); master_sync.REG_POWER_DBM records the same in the registry
# ---------------------


class LabelNode:
    def __init__(self):
        self.request = None                     # (item_name, expiry) waiting for the main loop
        self.done = threading.Event()
        self.result = (False, "")

        rospy.init_node('rfid_db_manager')
        rospy.Service('/inventory/add_item', AddLabeledItem, self.handle_add_item)
        rospy.loginfo("RFID label node ready: labels are saved in %s", master_sync.LABELS_DB)
        self.run_main_loop()

    def handle_add_item(self, req):
        """Service callback: hand the request to the main loop (it owns the reader) and wait."""
        rospy.loginfo("ROS Service Call received for: %s", req.item_name)
        self.done.clear()
        self.request = (req.item_name, req.expiration_date)
        self.done.wait()
        ok, msg = self.result
        return AddLabeledItemResponse(success=ok, message=msg)

    def read_tag(self):
        """The strongest tag the Mercury reader sees in 3 s -> (epc, rssi, reads), or None."""
        reader = mercury.Reader(MERCURY_PORT)
        reader.set_region(MERCURY_REGION)
        reader.set_read_powers([(1, MERCURY_POWER)])
        reader.set_read_plan([1], "GEN2")
        tags = reader.read(3000)
        if not tags:
            return None
        tag = max(tags, key=lambda x: x.read_count)
        epc_bytes = bytes(tag.epc)
        if len(epc_bytes) > 16:
            try:
                epc = epc_bytes.decode('utf-8').upper()
            except UnicodeDecodeError:
                epc = epc_bytes.hex().upper()
        else:
            epc = epc_bytes.hex().upper()
        return epc, getattr(tag, 'rssi', None), tag.read_count

    def label(self, item_name, expiry):
        try:
            found = self.read_tag()
        except Exception as e:  # noqa: BLE001 — any reader fault is a failed registration
            return False, "Hardware Error: %s" % e
        if found is None:
            return False, "No tag detected by Mercury reader."
        epc, rssi, reads = found
        rospy.loginfo("Scanned new label: %s", epc)
        try:
            ok, msg, _ = master_sync.add_label(epc, item_name, expiry, rssi, reads)
        except Exception as e:  # noqa: BLE001
            return False, "Could not save the label: %s" % e
        return ok, msg

    def run_main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.request is not None:
                item_name, expiry = self.request
                self.result = self.label(item_name, expiry)
                (rospy.loginfo if self.result[0] else rospy.logwarn)(self.result[1])
                self.request = None
                self.done.set()
            rate.sleep()


if __name__ == '__main__':
    try:
        LabelNode()
    except rospy.ROSInterruptException:
        pass
