#!/usr/bin/env python3

import rospy
import ikpy.chain
import numpy as np
from frankapy import FrankaArm
from geometry_msgs.msg import PoseArray
from autolab_core import RigidTransform
from scipy.spatial.transform import Rotation as R_scipy
from std_srvs.srv import Trigger 

def call_trigger_service(service_name):
    """Helper function to call ROS Trigger services (vacuum OR vision)."""
    try:
        # We don't want to log this every 10 seconds when looping, so we keep it quiet
        rospy.wait_for_service(service_name, timeout=10.0)
        service_proxy = rospy.ServiceProxy(service_name, Trigger)
        response = service_proxy()
        return response.success
    except rospy.ROSException:
        return False
    except Exception as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

        
class VisionToIKController:
    def __init__(self):
        # 1. Initialize FrankaArm and IK Chain
        self.fa = FrankaArm(with_gripper=False, robot_num=1)
        self.my_chain = ikpy.chain.Chain.from_urdf_file(
            "panda_arm_hand.urdf", 
            base_elements=["panda_link0"] 
        )

        # 2. Configure IK Mask
        chain_length = len(self.my_chain.links)
        active_mask = [False] * chain_length
        for i in range(1, 8):
            if i < chain_length:
                active_mask[i] = True
        self.my_chain.active_links_mask = active_mask
        self.chain_length = chain_length
        self.latest_pose_msg = None
        self.sub = rospy.Subscriber("/grasp_poses", PoseArray, self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        """Quietly updates the latest pose in the background."""
        self.latest_pose_msg = msg
        
    def run_continuous_loop(self):
        """Main robot state machine loop."""
        rospy.loginfo("Starting Continuous Pick and Place Loop...")
        
        while not rospy.is_shutdown():
            # STATE 1: Move to home/reset position before taking a picture
            rospy.loginfo("Moving to home position for scanning...")
            self.fa.reset_joints()
            
            # STATE 2: Clear old data and trigger vision
            self.latest_pose_msg = None 
            success = call_trigger_service('/compute_grasps_output')
            
            # STATE 3: Check if an object was found
            if not success:
                rospy.loginfo("No object found. Waiting 3 seconds before looking again...")
                rospy.sleep(3.0)
                continue
            
            # STATE 4: Wait for the background subscriber to catch the message
            timeout_time = rospy.Time.now() + rospy.Duration(5.0)
            while self.latest_pose_msg is None and rospy.Time.now() < timeout_time:
                rospy.sleep(0.1) # Check every 0.1 seconds

            if self.latest_pose_msg is None:
                rospy.logerr("Vision succeeded but no poses were caught by the subscriber.")
                continue

            if not self.latest_pose_msg.poses:
                rospy.logwarn("Received empty PoseArray despite vision success.")
                continue
                
            rospy.loginfo("Object Pose received! Initiating pick sequence...")
            target_pose = self.latest_pose_msg.poses[0]
            
            # --- [Your IK and Motion Logic] ---
            target_pos_initial = [
                target_pose.position.x,         
                target_pose.position.y, 
                target_pose.position.z + 0.1 
            ]
            
            target_pose_final = [
                target_pose.position.x,         
                target_pose.position.y, 
                target_pose.position.z + 0.05
            ]
            # quat = [
            #     target_pose.orientation.x,
            #     target_pose.orientation.y,
            #     target_pose.orientation.z,
            #     target_pose.orientation.w
            # ]
           
            # target_ori = R_scipy.from_quat(quat).as_matrix()
            target_ori = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
            
            initial_guess = [0.0] * self.chain_length
            if 4 < self.chain_length: initial_guess[4] = -1.5 

            try:
                # 1. Move to pre-grasp
                joint_angles = self.my_chain.inverse_kinematics(
                    target_position=target_pos_initial,
                    target_orientation=target_ori,
                    orientation_mode="all",
                    initial_position=initial_guess
                )
                self.fa.goto_joints(joint_angles[1:8])

                # 2. Move down to final grasp
                joint_angles_final = self.my_chain.inverse_kinematics(
                    target_position=target_pose_final,
                    target_orientation=target_ori,
                    orientation_mode="all",
                    initial_position=joint_angles
                )
                self.fa.goto_joints(joint_angles_final[1:8])
                
                # 3. Turn vacuum ON
                call_trigger_service('/snaak/pnp_cup/on')
                rospy.sleep(1.0)
                
                # 4. Move to drop-off points
                self.fa.goto_joints([-0.742869, -0.306372, 0.145069, -2.528092, 0.034586, 2.215739, 0.758835])
                self.fa.goto_joints([0.97138892, -0.10126605, -0.18165077, -2.35042734, -0.01300324,  2.17649246,  1.52808765])
                
                # 5. Turn vacuum OFF
                call_trigger_service('/snaak/pnp_cup/off')
                
                rospy.loginfo("Drop-off complete! Ready for next cycle.")
                
            except Exception as e:
                rospy.logerr(f"IK solver or motion failed during cycle: {e}")
                
            # After this `try/except` block finishes, the while loop automatically 
            # goes back to the top, resets the joints, and looks for the next object!

if __name__ == "__main__":
    try:
        controller = VisionToIKController()
        # Instead of rospy.spin(), we run our continuous loop
        controller.run_continuous_loop() 
    except rospy.ROSInterruptException:
        pass