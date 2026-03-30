#!/usr/bin/env python3
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

def main():
    rospy.init_node('vacuum_client_interface', anonymous=True)
    
    print("\n--- ORIO Pneumatic Control Interface ---")
    print("Commands: L1 (LBL ON), L0 (LBL OFF), P1 (PNP ON), P0 (PNP OFF), OFF (Global), Q (Quit)")
    
    while not rospy.is_shutdown():
        user_input = input("\nEnter Command: ").strip().upper()

        if user_input == 'L1':
            call_vacuum_service('/snaak/lbl_cup/on') # Sets IO2 LOW
        elif user_input == 'L0':
            call_vacuum_service('/snaak/lbl_cup/off') # Sets IO2 HIGH
        elif user_input == 'P1':
            call_vacuum_service('/snaak/pnp_cup/on') # Sets IO4 LOW
        elif user_input == 'P0':
            call_vacuum_service('/snaak/pnp_cup/off') # Sets IO4 HIGH
        elif user_input == 'OFF':
            call_vacuum_service('/snaak/vacuum/disable_all') # Sets both HIGH
        elif user_input == 'Q':
            break
        else:
            print("Invalid command. Use L1/L0, P1/P0, OFF, or Q.")

if __name__ == "__main__":
    main()