# SRC-ROS-2

Environment Setup 

* Ubuntu 20.04
* ROS2 Foxy
* Install Gazebo
* Dependency Packages

```
sudo apt install ros-foxy-nav2* -y
sudo apt install ros-foxy-ros2_control* -y

cd ~/ros2_ws
rosdinstall foxy

src_control_message clone
cbp src_control_message && rosfoxy
```

## Simplified SRC Description

* robot state publisher & joint state publisher => robot description
* rviz launch

```
ros2 launch src_description src_description.launch.py   
```

![image](https://user-images.githubusercontent.com/12381733/164446136-6d672a84-7492-4b1e-980c-d7bd01c17c86.png)

## SRC Simulation 1 - SRC & Empty World

* Gazebo launch
* SRC spawn & ros2_control launch

```
ros2 launch src_gazebo empty_world.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164446956-0b621647-d80b-4c97-909c-9325354dd427.png)

## SRC Simulation 2 - SRC & MIT Racecourse

* Gazebo launch
* rqt_robot_steering launch
* rviz launch

```
ros2 launch src_gazebo racecourse.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164447235-754808f0-bf47-4b63-88f7-92846a81f026.png)

![image](https://user-images.githubusercontent.com/12381733/164447257-73ab6f38-aade-4d16-ae4b-35b3d4aff65c.png)

## Controller Pkg

* Move forward for 5 seconds

```
ros2 run src_gazebo_controller basic_control
```

* Ground Truth odometry Publisher utility

```
ros2 run src_gazebo_controller odom_utility_tools
```

![image](https://user-images.githubusercontent.com/12381733/164449881-4698ee8c-9185-4960-b453-120f7869efbc.png)

자체 제작 odom과의 비교

![image](https://user-images.githubusercontent.com/12381733/164452303-43e9c5e3-2a31-41e1-94ba-0d0e6cd96adb.png)

* src_gazebo_controller.py

ackermann steering을 위한 controller 수식은 `readme` 참고

## Sensor Fusion

```
ros2 launch src_sensor_fusion racecourse.launch.py
ros2 launch src_sensor_fusion robot_localization.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164702848-1e41dbc1-b5d5-4dca-b10c-0409ef716bf5.png)

## SLAM

```
ros2 launch src_gazebo racecourse.launch.py use_rviz:=false
ros2 launch src_slam src_slam_gazebo_slam_toolbox.launch.py 
ros2 launch src_slam src_slam_gazebo_cartographer.launch.py use_sim_time:=true
```

![image](https://user-images.githubusercontent.com/12381733/164704324-b26fb411-e78a-4c69-90b6-bceed81d3976.png)

## AMCL (Localization)

```
ros2 launch src_gazebo racecourse.launch.py use_rviz:=false
ros2 launch src_amcl amcl.launch.py
```

![image](https://user-images.githubusercontent.com/12381733/164706444-f65bbe6a-73aa-441f-abb7-a1ce057123d4.png)

## Navigation 

```
ros2 launch src_gazebo racecourse.launch.py use_rviz:=false
ros2 launch src_nav bringup_launch.py
```

obstable avoidance

```
ros2 launch src_gazebo caffee_world.launch.py use_rviz:=false
ros2 launch src_nav caffee_bringup_launch.py 
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap request:\ {}\
```

![image](https://user-images.githubusercontent.com/12381733/164715379-02655e8b-58b4-48e4-a09c-c5f4a97fdef4.png)

# Real Robot

joy control 

```
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch src_demo joystick_control_foxy.launch.py
```

PID Tuning

```

```

SLAM Toolbox

```
# LattePanda
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0
ros2 launch src_slam src_slam.launch.py open_rviz:=false

# remote rviz view
ros2 launch src_slam only_rviz_foxy.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard 
# or
ros2 launch src_demo joystick_control_foxy.launch.py
```

Navigation

```
ros2 launch src_demo src_bringup_nav.launch.py
ros2 launch src_nav bringup_real_launch.py

# remote version




# SRC launch
ros2 launch src_demo src_bringup_nav.launch.py
ros2 launch src_nav bringup_real_launch.py open_rviz:=false

# Remote launch
ros2 launch src_nav rviz_view_launch.py
ros2 launch src_nav rviz_view_foxy_launch.py
```

# TODO
- [] 우분투에서 캡쳐 다시하기 (그림자, 카메라 이미지)
- [] 빌드 종속성 모두 확인 - Docker에서
- [] 장애물 회피용 param 최적화
