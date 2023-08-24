# RLCar gazebo 3d simulation for ROS2 education
![20230507_102923](https://github.com/RLmodel/RLCar/assets/32663016/f811e6a3-6740-42c5-b99f-ff7326d057b0)
![Screenshot from 2023-06-26 19-38-09](https://github.com/RLmodel/RLCar/assets/32663016/11f6b22c-ea56-480b-9072-cc9c6d6f4657)
## Manaul
---------
### install def packages
------------------------
. setup_script.sh
### Clone repo & build   
--------------
colcon build --packages-select src_amcl   
colcon build --packages-select src_controller   
colcon build --packages-select src_demo   
colcon build --packages-select src_description   
colcon build --packages-select src_gazebo   
colcon build --packages-select src_gazebo_controller   
colcon build --packages-select src_nav   
colcon build --packages-select src_odometry   
colcon build --packages-select src_sensor_fusion   
colcon build --packages-select src_slam   

### Ubuntu ROS ver
Python manual, ROS2 manual
## Hex file for arduino nano
binary hex file for upload the board

![xloader](https://user-images.githubusercontent.com/32663016/227823399-03a04a84-f2a9-4a4b-a09e-c7dcb68fe870.jpg)


### simple manual
- docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
- ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0
- ros2 launch src_slam src_slam.launch.py open_rviz:=false
- ros2 launch src_demo joystick_control_foxy.launch.py
- rviz2
- 
- cd /dev/input
- ls | grep js
- js0
  
- ros2 run joy joy_node
- ros2 topic echo /joy
- ros2 launch rplidar_ros view_rplidar_a2m8_launch.py
- ros2 launch mw_ahrsv1_ros2 view_imu.launch.py
- cbp src_nav && rosfoxy
