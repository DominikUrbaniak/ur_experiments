## Real Robotic Push Actions and Teleoperation experiment

Implementation of linear push actions using ArUco detection and teleoperation using mediapipe hand-landmark detection with UR5e.

## External Packages
(to be included into the workspace along the packages from this repository)
- ROS2 UR driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
- IOC-UPC inverse kinematics library for UR robots (kinenik): https://gitioc.upc.edu/robots/kinenik

## Packages in this repository
- control_pkg: low-level control node
- experiment_pkg: Nodes for image publishing and processing of aruco and hand-landmark detections, as well as high-level control for push actions and teleoperation
- custom_interfaces: Includes the custom messages and services

## Prepare experiments

- *ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=x.x.x.x launch_rviz:=true initial_joint_controller:=forward_velocity_controller*
- *ros2 run control_pkg rcvc_sub*
- *ros2 run experiment_pkg a1_compressed_image_publisher* (optional arguments: see code) 
- *ros2 run experiment_pkg a2_compressed_aruco_detection* or *ros2 run experiment_pkg a2_hand_detection*
- *ros2 run experiment_pkg real_pose_real_push* or *ros2 run experiment_pkg real_hand_follow* (optional arguments: see code)

## Run push experiment
- ros2 service call /real_pose_real_push/set_goal std_srvs/srv/Empty
- ros2 service call /real_pose_real_push/start_experiment std_srvs/srv/Empty

optional:
- ros2 service call /real_pose_real_push/settings_update std_srvs/srv/Empty
- ros2 service call /real_pose_real_push/stop std_srvs/srv/Empty

## Distributed control via two networks
enter the IP addresses for peer discovery in the fastdds_pro.xml and export it as *export FASTRTPS_DEFAULT_PROFILES_FILE=path_to_file_folder/fastdds_pro.xml* for the three bridging nodes, e.g. image publisher (onboard), image processer (edge), high-level controller (onboard)

