#!/usr/bin/env python3
import rospy
import threading
from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT
from custom_msgs.msg import ScannedTags

READER_IP = "192.168.0.20"

class ShelfScannerNode:
    def __init__(self):
        rospy.init_node('shelf_scanner_node')
        
        # Publisher for the heartbeat database updates
        self.scan_pub = rospy.Publisher('/inventory/shelf_scans', ScannedTags, queue_size=5)
        
        self.scanned_tags = set()
        self.lock = threading.Lock()
        
        # Publish batches every 2 seconds
        rospy.Timer(rospy.Duration(2.0), self.publish_batch)
        
        rospy.loginfo("Starting Impinj Shelf Scanner...")
        self.start_reader()

    def tag_report_cb(self, reader, tag_reports):
        """Background callback triggered by sllurp when tags are seen."""
        with self.lock:
            for tag in tag_reports:
                epc = tag.get('EPC-96', tag.get('EPCData', b'Unknown'))
                if isinstance(epc, bytes):
                    epc_string = str(epc).upper()[2:-1]
                    self.scanned_tags.add(epc_string)

    def publish_batch(self, event):
        """Publishes accumulated tags to the database manager."""
        with self.lock:
            if self.scanned_tags:
                msg = ScannedTags()
                msg.rfid_uids = list(self.scanned_tags)
                self.scan_pub.publish(msg)
                
                rospy.logdebug(f"Published heartbeat for {len(self.scanned_tags)} tags.")
                self.scanned_tags.clear() # Reset for the next batch

    def start_reader(self):
        config_dict = {
            'antennas': [1],
            'tx_power_dbm': 10.0, 
            'tag_content_selector': {
                'EnablePeakRSSI': True,
                'EnableAntennaID': True,
                'EnableTagSeenCount': True,
            }
        }
        config = LLRPReaderConfig(config_dict)
        reader = LLRPReaderClient(READER_IP, LLRP_DEFAULT_PORT, config)
        reader.add_tag_report_callback(self.tag_report_cb)
        
        try:
            reader.connect()
            rospy.spin() # Keep the ROS node alive
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.loginfo("Shutting down Impinj connection.")
            reader.disconnect()

if __name__ == '__main__':
    ShelfScannerNode()