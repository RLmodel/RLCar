우선은 불편하더라도 urdf 매번 생성해서 작업하자.

나중에 확실해지면 그때 또 바꾸면 됨

```
ros2 launch src_gazebo src_gazebo.launch.py
```

# odom 비교하기

/odom_rf2o/twist/twist/linear/x => 기본적으로 출렁인다.
cmd_vel하고 비교하려고 했는데 이건 뭐...

![image](https://user-images.githubusercontent.com/12381733/157583451-44f5861b-41ba-4504-a21e-3832a55183e8.png)

그런데, 가만히 있어도 tf가 마구 흔들리네

![image](https://user-images.githubusercontent.com/12381733/157586216-7c9e4a84-9b1a-4081-8c9b-43fd355b85f4.png)

pose를 비교해보자.

정확한 비교를 위해 plot의 크기와 높이를 맞춤

<p>
    <p align="center">
        <img src="https://user-images.githubusercontent.com/12381733/157586931-b348203c-6250-4267-9f8d-54e07bef3e5b.png" height="200">
        <img src="https://user-images.githubusercontent.com/12381733/157586971-979fbb12-d48b-49cc-8ff3-b2881c8fcb49.png" height="250">
    </p>
</p>

```
ros2 launch src_gazebo compare_odom.launch.py
```

이정도 차이난다.
계속 내비두면 laser odom은 낮아짐
![image](https://user-images.githubusercontent.com/12381733/157587300-ed5b3171-74fe-475d-b9d3-3310c795de90.png)

? 하다보니 또 맞을 때도 있네?

=> 일단 실제 로봇으로 가서 얼마나 잘되는지 실제 해보자.

# odom 제작

```
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
```

* ros2 topic echo /joint_states => 여기서의 vel은 각속도

예를 들어
* cmd_vel 0.1
* wheel_radius 0.05
일때, joint_states/velocity는 약 2가 된다.

* ros2 topic echo /joint_states => 여기서의 position은 radian이다.

5초간 0.1m/s 주면서 joint_state 모니터링

```
ros2 launch src_gazebo src_gazebo.launch.py
ros2 run src_gazebo_controller odom_test
ros2 topic echo /joint_states
=> 9.13정도 나옴
```

* ros2 topic echo /joint_states => 여기서의 position[4], position[5]는 radian이다.

```
ros2 topic echo /forward_position_controller/commands
- 0.36339788226173847
- 0.46082218765954414

ros2 topic echo /joint_states
position:
- 61.25404326045237
- 54.23001027119805
- 74.04593021112117
- 68.46704466435011
- 0.3621952714064074
- 0.46161041640775213
```

소수점 아래 2자리까지 믿을 만 하다.
대신 현재 좌우 angle이 다름에 주의

현재 사용하고자 하는 odom은 front angle을 좌우 구별하지 않는다.


```
$ ros2 topic info /joint_states
Type: sensor_msgs/msg/JointState
Publisher count: 2
Subscription count: 1

$ ros2 interface show sensor_msgs/msg/JointState
std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
```

sensor_msgs/msg/JointState => 이걸 받아서 odometry를 만들자.

실제 SRC는 servo 각도와 (피드백 없음) / 뒷바퀴 엔코더로 odom을 만든다.

# Ubuntu Desktop Error

```
[spawner.py-9] [ERROR] [1650724737.764791709] [spawner_velocity_controller]: Controller manager not available
```