rf2o_laser_odometry 실행 시 바퀴, 힌지 tf가 불안정한 이슈

odom pub rate가 낮아서 그런가?
적어도 20은 되어야 하나?

![image](https://user-images.githubusercontent.com/12381733/157614490-e2acd839-3113-4642-9272-ea0929fe1a12.png)

rviz 상에서 warning은 잡긴 함

laser odom이 워낙 느려서 old tf 에러가 발생했던 것임

CLaserOdometry2DNode.cpp
```c++
// odom.header.stamp = rf2o_ref.last_odom_time;
odom.header.stamp = this->get_clock()->now();
odom.header.frame_id = odom_frame_id;
```



```bash
ros2 run tf2_tools view_frames.py
```

Error, cannot vizualize laserScan data when fixed frame was odom, 

```
Message Filter dropping message: frame 'laser' at time 1647319692.498 for reason 'Unknown'
```

Edit timestamp in rf2o
```c++
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "[rf2o] Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    // odom_trans.header.stamp = rf2o_ref.last_odom_time;
    odom_trans.header.stamp = this->get_clock()->now();
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
```

Real Robot Slam
```
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch src_demo joystick_control_foxy.launch.py
# Accle => 10.0
# Max vel => 0.375

# lidar only slam
ros2 launch src_slam src_slam_lidar_odom.launch.py

# wheel odometry slam
ros2 launch src_slam src_slam.launch.py

# Remote view
ros2 launch src_slam only_rviz.launch.py
# eloquent
ros2 run nav2_map_server map_saver -f <map_dir>/<map_name>
# foxy
ros2 run nav2_map_server map_saver_cli -f <map_dir>/<map_name>
```

PID 

```
ros2 launch cmd_to_src cmd_to_src.launch.py
ros2 launch src_odometry src_bringup.launch.py
```

Gazebo SLAM

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py
ros2 launch src_slam src_slam_gazebo.launch.py
```

# TODO
- [] gmapping
- [] cartographer
- [] 3D SLAM 도 할 수 있을까??
- [] etc...

SLAM

```
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch src_slam src_slam.launch.py
```

Navigation 

```
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch src_demo src_bringup_nav.launch.py
ros2 launch src_nav bringup_real_launch.py
```

# Documentation

save map에다가 경로 넣으면 자동 저장

예를 들어, /home/kimsooyoung/ros2_ws/map/test를 넣으면, test.pgm/test.yaml이 생긴다.

![image](https://user-images.githubusercontent.com/12381733/164704324-b26fb411-e78a-4c69-90b6-bceed81d3976.png)

