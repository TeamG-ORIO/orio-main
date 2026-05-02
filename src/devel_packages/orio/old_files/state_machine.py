#!/usr/bin/env python
# ********************************************************************************
# DESCRIPTION: SMACH Finite State Machine for a dual-arm labeling robot system
#               with two shared labeling zones (A and B).
#
# DESIGN PATTERN:
#   - Concurrent State Machines running in parallel.
#   - Pipelined logic for Arm 1 (Logistics) to fill/empty zones opportunistically.
#   - Shared "ZoneManager" object acting as a thread-safe Blackboard/Mutex Hub.
# ********************************************************************************

import rospy
import smach
import smach_ros
from threading import Lock

# ==============================================================================
# SECTION 1: SYSTEM SETUP & DATA MANAGEMENT
# ==============================================================================

# --- MOCK / PLACEHOLDER IMPORTS ---
# When running on real hardware, you will uncomment these:
from frankapy import FrankaArm
from std_srvs.srv import Trigger
# from std_srvs.srv import SetBool # Common for suction
# from perception_msgs.srv import GetGraspPose # Example custom service for vision

def call_trigger_service(service_name):
    """Helper function to call ROS Trigger services (vacuum OR vision)."""
    try:
        rospy.loginfo(f"Calling service: {service_name}")
        rospy.wait_for_service(service_name, timeout=5)
        service_proxy = rospy.ServiceProxy(service_name, Trigger)
        response = service_proxy()
        rospy.loginfo(f"Service Response: {response.message}")
        return response.success
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available (timeout).")
    except Exception as e:
        rospy.logerr(f"Service call failed: {e}")
    return False

class ZoneManager:
    """
    SHARED STATUS BOARD (Blackboard / Mutex Hub)
    This object is shared by both arms. It is crucial that it is thread-safe
    because both state machines will be reading/writing to it simultaneously.
    """
    def __init__(self):
        # The 'lock' ensures only one arm can update the board at a time.
        self.lock = Lock()
        
        # ZONE STATUS (Variables tracking the item lifecycle)
        # States: 
        #   'EMPTY' - No item present, ready for Arm 1 to bring one.
        #   'NEEDS_LABEL' - Item present, waiting for Arm 2 to apply label.
        #   'READY_FOR_PICKUP' - Item is labeled, ready for Arm 1 to take to Output.
        #   'ERROR' - Something went wrong in this zone (failed pick/place).
        self.states = {'A': 'EMPTY', 'B': 'EMPTY'}
        
        # ZONE BUSY LOCKS (Variables tracking physical collisions)
        #   True - An arm is physically inside the zone volume.
        #   False - The zone is clear and safe to enter.
        self.busy = {'A': False, 'B': False}

# Global manager instance. Keep it global so both sub-state machines can see it.
manager = ZoneManager()

# ==============================================================================
# SECTION 2: ARM 1 (LOGISTICS SM) STATES
# ==============================================================================
# The Logistics SM handles "moving boxes." It finds EMPTY zones and fills them
# from Input, and finds READY_FOR_PICKUP zones and takes them to Output.

class TaskSelector(smach.State):
    """
    The "Decision Hub" for Arm 1.
    Instead of a fixed loop, Arm 1 constantly polls the ZoneManager to see
    what the most productive thing it can do next is. This is key for pipelining.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['do_load', 'do_unload', 'wait'])

    def execute(self, userdata):
        # 1. ALWAYS GET THE LOCK FIRST when talking to the Status Board
        with manager.lock:
            rospy.loginfo("Arm 1 (Hub): Scanning Zone Manager...")
            
            # --- STRATEGY: Prioritize unblocking the exit ---
            # Rule 1: Prioritize Clearing Labeled Items
            # (Check if ANY zone is finished AND physically clear)
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    rospy.loginfo(f"Arm 1 (Hub): Zone {z} is finished. Executing Retrieval.")
                    return 'do_unload'
            
            # Rule 2: Prioritize Filling Empty Zones
            # (Check if ANY zone is empty AND physically clear)
            # *Optional*: You could add a check here for the 'Input Sensor' topic.
            for z in ['A', 'B']:
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    rospy.loginfo(f"Arm 1 (Hub): Zone {z} is empty. Executing Loading.")
                    return 'do_load'
            
            # Rule 3: Wait
            # (Zones are full but Arm 2 is still working)
            rospy.loginfo("Arm 1 (Hub): Both zones are full/busy. Waiting...")
            return 'wait'

class FetchInput(smach.State):
    """Moves from Home -> Input Zone -> Perceive -> Pick (Suction ON)."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        # Initialization (Wraps frankapy and Service Clients)
        # When running real hardware, initialize your objects here:
        # self.fa = FrankaArm()
        self.fa1 = FrankaArm(with_gripper=False, old_gripper=False, robot_num=1) 
        # rospy.wait_for_service('//get_input_pose')
        # self.vision_srv = rospy.ServiceProxy('/get_input_pose', GetGraspPose)
        # rospy.wait_for_service('/suction_control')
        # self.suction_srv = rospy.ServiceProxy('/suction_control', SetBool)

    def execute(self, userdata):
        rospy.loginfo("Arm 1 (Fetch): Moving to Input Zone...")
        # =========================================
        # --- PLACE YOUR HARDWARE CODE HERE ---
        # 1. PERCEPTION CALL: response = self.vision_srv()
        # 2. MOVEMENT: self.fa.goto_pose(response.grasp_pose)
        # 3. ACTION: self.suction_srv(True)
        # =========================================
        
        rospy.loginfo("Arm 1 (Fetch): Item picked. Proceeding to find zone.")
        return 'succeeded'

class DropToZone(smach.State):
    """Waits for an EMPTY/CLEAR zone -> Moves -> Drops (Suction OFF) -> Updates Status."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropped', 'wait'])

    def execute(self, userdata):
        target_zone = None
        
        # 1. Look for a valid zone and write the "ENTRY" Busy Lock
        with manager.lock:
            for z in ['A', 'B']:
                # The crucial check: status is correct AND lock is CLEAR
                if manager.states[z] == 'EMPTY' and not manager.busy[z]:
                    target_zone = z
                    # WRITE THE MUTEX LOCK before any movement
                    manager.busy[z] = True 
                    rospy.loginfo(f"Arm 1 (Drop): Physical lock acquired for Zone {z}")
                    break
        
        # 2. If a zone was found, execute physical move
        if target_zone:
            rospy.loginfo(f"Arm 1 (Drop): Placing item in Zone {target_zone}")
            # =========================================
            # --- PLACE YOUR HARDWARE CODE HERE ---
            # 1. MOVEMENT: self.fa.goto_pose(pose_for_zone[target_zone])
            # 2. ACTION: self.suction_srv(False)
            # =========================================
            
            # 3. Perform the HANDSHAKE: Update Status and RELEASE the Busy Lock
            with manager.lock:
                manager.states[target_zone] = 'NEEDS_LABEL'
                manager.busy[target_zone] = False # Safe to enter again
                rospy.loginfo(f"Arm 1 (Drop): Update successful: Zone {target_zone} is now NEEDS_LABEL.")
            return 'dropped'
        
        # 4. If no zone was empty (this shouldn't happen with the Pipelined structure), wait.
        rospy.sleep(0.5)
        return 'wait'

class RetrieveFromZone(smach.State):
    """Waits for a READY/CLEAR zone -> Moves -> Picks (Suction ON) -> Updates Status."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['picked', 'no_items'])

    def execute(self, userdata):
        target_zone = None
        
        # 1. Find a valid zone and write the "ENTRY" Busy Lock
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'READY_FOR_PICKUP' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True # WRITE MUTEX
                    break
        
        # 2. Physical execution
        if target_zone:
            rospy.loginfo(f"Arm 1 (Retrieve): Retrieving labeled item from Zone {target_zone}")
            # =========================================
            # --- PLACE YOUR HARDWARE CODE HERE ---
            # 1. MOVEMENT: self.fa.goto_pose(pose_for_zone[target_zone])
            # 2. ACTION: self.suction_srv(True)
            # =========================================
            
            # 3. Perform HANDSHAKE: Update Status to EMPTY and RELEASE Busy Lock
            with manager.lock:
                manager.states[target_zone] = 'EMPTY'
                manager.busy[target_zone] = False
                rospy.loginfo(f"Arm 1 (Retrieve): Update successful: Zone {target_zone} is now EMPTY.")
            return 'picked'
        
        return 'no_items'

class PlaceOutput(smach.State):
    """Moves from Labelling Zone -> Output Zone -> Drops (Suction OFF)."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo("Arm 1 (Exit): Placing item in Output Zone...")
        # =========================================
        # --- PLACE YOUR HARDWARE CODE HERE ---
        # 1. MOVEMENT: self.fa.goto_pose(pose_output)
        # 2. ACTION: self.suction_srv(False)
        # =========================================
        return 'succeeded'

# ==============================================================================
# SECTION 3: ARM 2 (LABELING SM) STATES
# ==============================================================================
# The Labeling SM is a specialist. It only wakes up when the Status Board
# says a zone is NEEDS_LABEL.

class GetLabel(smach.State):
    """Home -> Label Dispenser -> Perceive -> Pick (Suction ON)."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo("Arm 2 (Prep): Fetching a new label from dispenser...")
        # =========================================
        # --- PLACE YOUR HARDWARE CODE HERE ---
        # (Moves to dispenser, suction ON)
        # =========================================
        return 'succeeded'

class ApplyLabel(smach.State):
    """
    Look for a zone marked NEEDS_LABEL -> (Wait for Lock) -> 
    Perceive Item Pose -> Move -> Apply -> Updates Status.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['labeled', 'wait'])

    def execute(self, userdata):
        target_zone = None
        
        # 1. Scan board for work order AND write the "ENTRY" Lock
        with manager.lock:
            for z in ['A', 'B']:
                if manager.states[z] == 'NEEDS_LABEL' and not manager.busy[z]:
                    target_zone = z
                    manager.busy[z] = True # WRITE MUTEX
                    rospy.loginfo(f"Arm 2 (Apply): Starting labeling sequence in Zone {z}")
                    break
        
        # 2. Execution
        if target_zone:
            # =========================================
            # --- PLACE YOUR HARDWARE CODE HERE ---
            # 1. PERCEPTION CALL: Ask where the item is sitting now
            # 2. MOVEMENT: Move precisely to apply label
            # 3. ACTION: Suction OFF
            # =========================================
            rospy.loginfo(f"Arm 2 (Apply): Label applied in Zone {target_zone}.")
            
            # 3. Perform HANDSHAKE: Update Status to READY and RELEASE Busy Lock
            with manager.lock:
                manager.states[target_zone] = 'READY_FOR_PICKUP'
                manager.busy[target_zone] = False
                rospy.loginfo(f"Arm 2 (Apply): Update successful: Zone {target_zone} is now READY.")
            return 'labeled'
        
        # 4. If nothing needs a label, just wait and re-check.
        rospy.sleep(0.5)
        return 'wait'

# ==============================================================================
# SECTION 4: MAIN EXECUTION FUNCTION
# ==============================================================================

def main():
    # 1. Always initialize your ROS node first
    rospy.init_node('dual_arm_labeling_fsm')

    # 2. DEFINE LOGISTICS SUB-STATEMACHINE (ARM 1)
    # outcomes=['finished'] allows it to shut down cleanly if needed
    sm_logistics = smach.StateMachine(outcomes=['finished'])
    with sm_logistics:
        # A: Central Decision Hub (The Pipelining logic)
        smach.StateMachine.add('DECIDE_HUB', TaskSelector(), 
                               transitions={'do_load':'FETCH_INPUT', 
                                            'do_unload':'RETRIEVE', 
                                            'wait':'DECIDE_HUB'})
        
        # B: Loading Sequence
        # If Drop succeeds, go back to DECIDE, not Retrieve (Pipelining!)
        smach.StateMachine.add('FETCH_INPUT', FetchInput(), 
                               transitions={'succeeded':'DROP_TO_ZONE', 'failed':'DECIDE_HUB'})
        smach.StateMachine.add('DROP_TO_ZONE', DropToZone(), 
                               transitions={'dropped':'DECIDE_HUB', 'wait':'DROP_TO_ZONE'})
        
        # C: Retrieval Sequence
        smach.StateMachine.add('RETRIEVE', RetrieveFromZone(), 
                               transitions={'picked':'PLACE_OUTPUT', 'no_items':'DECIDE_HUB'})
        smach.StateMachine.add('PLACE_OUTPUT', PlaceOutput(), 
                               transitions={'succeeded':'DECIDE_HUB'})

    # 3. DEFINE LABELING SUB-STATEMACHINE (ARM 2)
    # Simplified specialst loop
    sm_labeling = smach.StateMachine(outcomes=['finished'])
    with sm_labeling:
        # StateMachine re-uses logic automatically after transitions
        smach.StateMachine.add('GET_LABEL', GetLabel(), 
                               transitions={'succeeded':'APPLY'})
        smach.StateMachine.add('APPLY', ApplyLabel(), 
                               transitions={'labeled':'GET_LABEL', 'wait':'APPLY'})

    # 4. DEFINE THE TOP-LEVEL "CONCURRENCE" CONTAINER
    # This allows both sub-state machines to run concurrently.
    top_sm = smach.Concurrence(outcomes=['done', 'error'],
                               default_outcome='error', # Safety default
                               # Defines when the WHOLE system is considered finished
                               outcome_map={'done': {'ARM1':'finished', 'ARM2':'finished'}})

    # 5. ASSEMBLE EVERYTHING
    # The 'Concurrence' adds the two existing state machines.
    with top_sm:
        smach.Concurrence.add('ARM1', sm_logistics)
        smach.Concurrence.add('ARM2', sm_labeling)

    # 6. (Optional) Initialize and Start the SMACH Introspection Visualizer.
    # Highly recommended! Run `rosrun smach_viewer smach_viewer.py` to see it.
    sis = smach_ros.IntrospectionServer('orio_visualiser', top_sm, '/ORIO_ROOT')
    sis.start()

    # 7. EXECUTE the State Machine
    # This call is blocking; the loop runs here until finished.
    outcome = top_sm.execute()
    sis.stop() 
    # 8. Shutdown logic
    # sis.stop() # Stop visualizer if used
    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()