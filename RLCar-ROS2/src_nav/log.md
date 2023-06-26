```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py use_rviz:=false

ros2 launch src_nav bringup_launch.py use_sim_time:=true open_rviz:=true
=> 이건 된다!! 이상하게 따로 하면 안됨...

# terminal 1
ros2 launch src_nav localization_launch.py use_sim_time:=true
# terminal 2 => 여기서 에러 
ros2 launch src_nav navigation_launch.py use_sim_time:=true
# terminal 3
ros2 launch src_nav rviz_view_launch.py use_sim_time:=true
```

```
[ERROR] [bt_navigator-4]: process has died [pid 4986, exit code -8, cmd '/opt/ros/foxy/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmpfe0pkevn -r /tf:=tf -r /tf_static:=tf_static'].
```

=> basic_mobile_robot에서도 그런다. 
이건 bt navigator 자체의 문제임 => 소스코드 빌드 시도 

```
[ERROR] [bt_navigator-12]: process has died [pid 3412, exit code -8, cmd '/opt/ros/foxy/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmpzglxw2pq -r /tf:=tf -r /tf_static:=tf_static'].
```

=> 소스코드 빌드해야 한다!!!

```
git clone -b foxy-devel https://github.com/ros-planning/navigation2.git
cba
```

src_dwb_params.yaml => 가긴 가는데 navigation fail이 종종 난다. 

```
[bt_navigator-7] [ERROR] [1650195121.345171135] [bt_navigator]: Navigation failed
[bt_navigator-7] [WARN] [1650195121.345286792] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
```

src_regulated_pure_pursuit.yaml => 이것도 이러네...

```
[controller_server-4] [ERROR] [1650196654.128657863] [controller_server]: Failed to make progress
[controller_server-4] [WARN] [1650196654.128999544] [controller_server_rclcpp_node]: [follow_path] [ActionServer] Aborting handle.
[bt_navigator-7] [ERROR] [1650196654.155309027] [bt_navigator]: Navigation failed
[bt_navigator-7] [WARN] [1650196654.155373578] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
```

자꾸 fail 나는 이유가 뭘까?
nav2 goal action 디버깅 방법

```
# action topic 조회
ros2 topic list --include-hidden-topics

# navigate_to_pose echo 
ros2 topic echo /navigate_to_pose/_action/status

# 이때 나오는 echo 중 제일 마지막 status code는 다음 링크에 해당됨
status => https://docs.ros.org/en/api/actionlib/html/index.html
```

angle이 잘 안맞네... 우선 goal tolerence를 좀 키웠음

```
goal_checker:
    plugin: "nav2_controller::SimpleGoalChecker"
    xy_goal_tolerance: 0.25
    yaw_goal_tolerance: 0.5 # 0.25 => 0.5
    stateful: True
```

what difference between voxel layer and obstacle layer ?

```
VoxelLayer tracks obstacles in 3d, whereas obstacle layer only does two. If you only have a planar laser for instance, then 2d is fine. If you have a 3d sensor or sensors at multiple levels, you may want VoxelLayer.
```

회전이 잘 안된다. 지지지직 거림

옆으로 가는 걸 거의 못하네, 스무스하게 해주는 걸 많이 해야 함

최소 속도를 높여야 그나마 가는데, 이 값이 무엇인지 공부하자

현재는 min_approach_linear_velocity로 추정중