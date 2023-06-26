주의사항 

모델링 파일에서 로봇 원점을 잘 잡아줘야 한다. 
x y z 좌표계도 잘 잡아야 함

![image](https://user-images.githubusercontent.com/12381733/164065909-3f7d109f-4d2b-4a75-ab7b-3a3ddd9c0011.png)


inertial는 mass가 반드시 있어야
visual / collision은 없어도 됨

결국 하나하나씩 보면서 joint 전부 잡아줬다. tq...



[] 라이다 / 카메라 / imu / gps 등등 달기
[] robot_description topic 살피기

Debug

description은 package://src_description 이렇게 해야 하고

가제보는 아래와 같이 $(find test_description) 형태로 바꿔줘야 제대로 나온다.

```
<geometry>
<mesh filename="$(find test_description)/meshes/right_rear_wheel.stl" scale="0.001 0.001 0.001"/>
</geometry>
```

color & physics

=> http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

가제보에서 잘 가다가 갑자기 탈조 나는 현상 발생

![image](https://user-images.githubusercontent.com/12381733/164165986-d22b2554-ce06-447b-8f7c-960e992104d0.png)

hinge에서 나는 것 같아서 racecar보고 질량 수정함

```
  <link name="left_steering_hinge">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.100"/>
```

가만히 있어도 막 움직이는 문제
=> 옆으로 움직인다.

아래와 같이 보이는 건 동그랗게, 실제는 원기둥으로 교체해보자.

![image](https://user-images.githubusercontent.com/12381733/164170370-01788062-713d-4e21-97fe-5b733a93b317.png)

=> 지혼자 춤을 춘다...

왜 혼자 도는 것일지 생각을 해보자.

