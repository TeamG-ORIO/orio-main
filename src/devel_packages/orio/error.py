(base) student@iam-doc:~/Documents/franka-interface$ roslaunch franka_ros_interface franka_ros_interface.launch robot_num:=1
... logging to /home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/roslaunch-iam-doc-20597.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://iam-doc:39595/

SUMMARY
========

PARAMETERS
 * /execute_skill_action_server_node_1/publish_frequency: 10
 * /franka_interface_status_publisher_node_1/publish_frequency: 100
 * /get_current_franka_interface_status_server_node_1/franka_interface_status_topic_name: /franka_interface...
 * /get_current_robot_state_server_node_1/robot_state_topic_name: /robot_state_publ...
 * /robot_state_publisher_node_1/publish_frequency: 100
 * /rosdistro: melodic
 * /rosversion: 1.14.13
 * /run_loop_process_info_state_publisher_node_1/publish_frequency: 100

NODES
  /
    execute_skill_action_server_node_1 (franka_ros_interface/execute_skill_action_server_node)
    franka_interface_status_publisher_node_1 (franka_ros_interface/franka_interface_status_publisher_node)
    get_current_franka_interface_status_server_node_1 (franka_ros_interface/get_current_franka_interface_status_server_node)
    get_current_robot_state_server_node_1 (franka_ros_interface/get_current_robot_state_server_node)
    robot_state_publisher_node_1 (franka_ros_interface/robot_state_publisher_node)
    run_loop_process_info_state_publisher_node_1 (franka_ros_interface/run_loop_process_info_state_publisher_node)
    sensor_data_subscriber_node_1 (franka_ros_interface/sensor_data_subscriber_node)

ROS_MASTER_URI=http://iam-abuela:11311

process[execute_skill_action_server_node_1-1]: started with pid [20610]
process[robot_state_publisher_node_1-2]: started with pid [20611]
process[franka_interface_status_publisher_node_1-3]: started with pid [20612]
process[run_loop_process_info_state_publisher_node_1-4]: started with pid [20613]
terminate called after throwing an instance of 'boost::interprocess::interprocess_exception'
  what():  No such file or directory
process[get_current_robot_state_server_node_1-5]: started with pid [20620]
terminate called after throwing an instance of 'boost::interprocess::interprocess_exception'
  what():  No such file or directory
process[get_current_franka_interface_status_server_node_1-6]: started with pid [20629]
terminate called after throwing an instance of 'boost::interprocess::interprocess_exception'
  what():  No such file or directory
process[sensor_data_subscriber_node_1-7]: started with pid [20632]
terminate called after throwing an instance of 'boost::interprocess::interprocess_exception'
  what():  No such file or directory
[ INFO] [1772143324.241862632]: Get Current Robot State Server Started
[ INFO] [1772143324.247180667]: Get Current FrankaInterface Status Server Started
terminate called after throwing an instance of 'boost::interprocess::interprocess_exception'
  what():  No such file or directory
[execute_skill_action_server_node_1-1] process has died [pid 20610, exit code -6, cmd /home/student/Documents/franka-interface/catkin_ws/devel/lib/franka_ros_interface/execute_skill_action_server_node __name:=execute_skill_action_server_node_1 __log:=/home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/execute_skill_action_server_node_1-1.log].
log file: /home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/execute_skill_action_server_node_1-1*.log
[robot_state_publisher_node_1-2] process has died [pid 20611, exit code -6, cmd /home/student/Documents/franka-interface/catkin_ws/devel/lib/franka_ros_interface/robot_state_publisher_node __name:=robot_state_publisher_node_1 __log:=/home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/robot_state_publisher_node_1-2.log].
log file: /home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/robot_state_publisher_node_1-2*.log
[franka_interface_status_publisher_node_1-3] process has died [pid 20612, exit code -6, cmd /home/student/Documents/franka-interface/catkin_ws/devel/lib/franka_ros_interface/franka_interface_status_publisher_node __name:=franka_interface_status_publisher_node_1 __log:=/home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/franka_interface_status_publisher_node_1-3.log].
log file: /home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/franka_interface_status_publisher_node_1-3*.log
[run_loop_process_info_state_publisher_node_1-4] process has died [pid 20613, exit code -6, cmd /home/student/Documents/franka-interface/catkin_ws/devel/lib/franka_ros_interface/run_loop_process_info_state_publisher_node __name:=run_loop_process_info_state_publisher_node_1 __log:=/home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/run_loop_process_info_state_publisher_node_1-4.log].
log file: /home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/run_loop_process_info_state_publisher_node_1-4*.log
[sensor_data_subscriber_node_1-7] process has died [pid 20632, exit code -6, cmd /home/student/Documents/franka-interface/catkin_ws/devel/lib/franka_ros_interface/sensor_data_subscriber_node __name:=sensor_data_subscriber_node_1 __log:=/home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/sensor_data_subscriber_node_1-7.log].
log file: /home/student/.ros/log/ac5fd290-135d-11f1-b854-ed1aa1c8229a/sensor_data_subscriber_node_1-7*.log