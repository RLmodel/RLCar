base_link가 없는 문제 발생

![image](https://user-images.githubusercontent.com/12381733/163363578-ca77fc68-d754-4155-925c-1223a8a3580d.png)

일전 fusionbot은 이렇게 설정했었다.

![image](https://user-images.githubusercontent.com/12381733/163363714-0ced8491-afb9-41b4-8bc2-7dc4958f4abd.png)

urdf에 base_footprint 모두 추가
- src_description
- src_gazebo 

```
<link name="base_footprint" />

<link name="base_link" />

<joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

이제 odom -> base_footprint -> base_link 이렇게 간다. 
그리고 base_footprint는 뒷바퀴 중심임

```python
src_odometry_gazebo = Node(
    package='src_odometry',
    parameters=[{
        'base_frame_id' : "base_footprint",
        ...
```

![image](https://user-images.githubusercontent.com/12381733/163383859-90cee82e-0b18-468f-bc97-5d6279f761b7.png)


```
Waiting for the lifecycle_manager_navigation/is_active service...
```

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py use_rviz:=false
ros2 launch src_amcl amcl.launch.py
=> 되는데?
```
