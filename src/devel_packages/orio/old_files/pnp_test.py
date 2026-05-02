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
        rospy.loginfo(f"Calling service: {service_name}")
        rospy.wait_for_service(service_name, timeout=10)
        service_proxy = rospy.ServiceProxy(service_name, Trigger)
        response = service_proxy()
        rospy.loginfo(f"Service Response: {response.message}")
        return response.success
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available (timeout).")
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
        
        # Move to home/reset position before taking a picture
        self.fa.reset_joints()
        print("reset done")

        # 3. Setup Subscriber FIRST (to catch the result)
        self.sub = rospy.Subscriber("/grasp_poses", PoseArray, self.grasp_callback, queue_size=1)
        
        # 4. TRIGGER THE VISION SERVICE
        # This tells your AI node to take a picture and publish to /grasp_poses
        success = call_trigger_service('/compute_grasps')
        
        if success:
            rospy.loginfo("Vision triggered. Waiting for poses on /grasp_poses...")
        else:
            rospy.logerr("Vision trigger failed. Is the vision node running?")

    def grasp_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("Received empty PoseArray")
            return
        
        # Unregister immediately so we don't process multiple frames
        self.sub.unregister() 
        rospy.loginfo("Unregistered subscriber. Moving to object...")
        
        target_pose = msg.poses[0]
        
        # --- [The rest of your IK and Motion logic remains the same] ---
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
        # print(target_ori)
        target_ori = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
        initial_guess = [0.0] * self.chain_length
        if 4 < self.chain_length: initial_guess[4] = -1.5 

        try:
            joint_angles = self.my_chain.inverse_kinematics(
                target_position=target_pos_initial,
                target_orientation=target_ori,
                orientation_mode="all",
                initial_position=initial_guess
            )

            self.fa.goto_joints(joint_angles[1:8])


            # delta_down = RigidTransform(
            #     translation=[0, 0, -0.05], 
            #     rotation=np.eye(3), 
            #     from_frame='world', to_frame='world'
            # )
            # self.fa.goto_pose_delta(delta_down, use_impedance=False)

            joint_angles_final = self.my_chain.inverse_kinematics(
                target_position=target_pose_final,
                target_orientation=target_ori,
                orientation_mode="all",
                initial_position=joint_angles
            )
            self.fa.goto_joints(joint_angles_final[1:8])
            # Turn vacuum ON
            call_trigger_service('/snaak/pnp_cup/on')
            rospy.sleep(1.0)
            
            # Move to drop-off points
            self.fa.goto_joints([-0.742869, -0.306372, 0.145069, -2.528092, 0.034586, 2.215739, 0.758835])
            #L2
            self.fa.goto_joints([ 0.16034816,  1.2326168,   0.37018591, -0.66147966, -0.37486378,  1.88602976, 1.15274049])
            #L1
            #self.fa.goto_joints([-0.171976, 1.207789, -0.260384, -0.733903, 0.281330, 1.963607, 0.521536])
            
            # Turn vacuum OFF
            call_trigger_service('/snaak/pnp_cup/off')
            self.fa.reset_joints()
            
        except Exception as e:
            rospy.logerr(f"IK solver or motion failed: {e}")

if __name__ == "__main__":
    try:
        controller = VisionToIKController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass