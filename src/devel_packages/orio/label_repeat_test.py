import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

import rospy
from std_srvs.srv import Trigger

def call_vacuum_service(service_name):
    """Helper function to call ROS Trigger services safely."""
    try:
        rospy.wait_for_service(service_name, timeout=2)
        service_proxy = rospy.ServiceProxy(service_name, Trigger)
        response = service_proxy()
        rospy.loginfo(f"Service Response: {response.message}")
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available. Is the node running?")
    except Exception as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
 #rospy.init_node('vacuum_client_interface', anonymous=True)
 fa = FrankaArm(with_gripper=False, old_gripper=False, robot_num=2)
#  fa.reset_joints()
 fa.goto_joints([ 1.31472703e+00, 1.22115404e-01, 9.75370435e-02, -2.53001883e+00, -2.44710884e-03, 2.61867466e+00, 2.15848474e+00])
 call_vacuum_service('/snaak/lbl_cup/on')
 rospy.sleep(1.0)
 fa.reset_joints()
 call_vacuum_service('/snaak/lbl_cup/off')

# Enter a number: 1
# pose: 
# Tra: [0.0714189  0.45083464 0.13961791]
#  Rot: [[ 0.99900712  0.04321068 -0.00991763]
#  [ 0.04295068 -0.99875267 -0.025082  ]
#  [-0.01098907  0.02463113 -0.9996362 ]]
#  Qtn: [ 0.01243233  0.99967445  0.02154736 -0.00522838]
#  from franka_tool to world
# Enter a number: 2
# joints: 
# [ 1.32810154  0.11605488  0.0824611  -2.53009525 -0.00753944  2.61939914
#   2.15901515]
