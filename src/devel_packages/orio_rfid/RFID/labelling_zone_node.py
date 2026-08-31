#!/usr/bin/env python3
import rospy
import mercury
from custom_msgs.srv import AddLabeledItem, AddLabeledItemRequest
# Assuming you create a custom TriggerLabeling.srv with string item_name, string expiration_date
from custom_msgs.srv import TriggerLabeling, TriggerLabelingResponse 

PORT = "tmr:///dev/ttyUSB0"
REGION = "NA"
SAFE_POWER = 1000

class LabelingStationNode:
    def __init__(self):
        rospy.init_node('labeling_station_node')
        
        # Wait for the database manager to be ready
        rospy.loginfo("Waiting for database manager service...")
        rospy.wait_for_service('/inventory/add_item')
        self.db_client = rospy.ServiceProxy('/inventory/add_item', AddLabeledItem)
        rospy.loginfo("Connected to database manager.")

        # Expose a service to trigger the hardware scan
        self.trigger_srv = rospy.Service('/inventory/trigger_labeling', TriggerLabeling, self.handle_trigger)
        rospy.loginfo("Labeling Station Ready. Waiting for triggers...")

    def handle_trigger(self, req):
        """Triggered when a new label needs to be scanned and registered."""
        rospy.loginfo(f"Scanning new label for item: {req.item_name}")
        
        epc_string = self.scan_single_tag()
        
        if not epc_string:
            return TriggerLabelingResponse(success=False, message="Hardware failed to detect any tag.")
            
        rospy.loginfo(f"Tag {epc_string} detected. Sending to database...")
        
        # Act as a client to the db_manager
        db_request = AddLabeledItemRequest(
            rfid_uid=epc_string,
            item_name=req.item_name,
            expiration_date=req.expiration_date
        )
        
        try:
            db_response = self.db_client(db_request)
            return TriggerLabelingResponse(
                success=db_response.success, 
                message=db_response.error if not db_response.success else "Registered successfully."
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to communicate with DB: {e}")
            return TriggerLabelingResponse(success=False, message="Database service failed.")

    def scan_single_tag(self):
        """Handles the Mercury hardware initialization and read cycle."""
        try:
            reader = mercury.Reader(PORT)
            reader.set_region(REGION)
            reader.set_read_powers([(1, SAFE_POWER)])
            reader.set_read_plan([1], "GEN2")
            
            tags = reader.read(3000) # 3 second scan window
            
            if not tags:
                return None
                
            # Return the first detected tag
            return str(tags[0].epc)[2:-1]
            
        except Exception as e:
            rospy.logerr(f"Mercury Reader Error: {e}")
            return None

if __name__ == '__main__':
    LabelingStationNode()
    rospy.spin()