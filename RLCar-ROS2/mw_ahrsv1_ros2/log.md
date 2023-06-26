

ros2 run mw_ahrsv1_ros2 mw_ahrsv1_only_angle 

mw_ahrsv1으로 부터 roll pitch yaw 값을 받아오며, 콘솔 로그를 통해 디버깅이 가능하다.

```
[INFO] [1648380954.187335403] [mv_ahrsv1_node]: 
imu_raw_data.roll : 0.057072         
imu_raw_data.pitch : 0.006632         
imu_raw_data.yaw : 0.053756
[INFO] [1648380954.207261575] [mv_ahrsv1_node]: 
imu_raw_data.roll : 0.056898         
imu_raw_data.pitch : 0.006632         
imu_raw_data.yaw : 0.053756
...
```

ros2 launch mw_ahrsv1_ros2 view_imu.launch.py

rviz를 통해 imu tf frame의 변화를 확인할 수 있다.
현재 angle 값만 갖고 있기 때문에 imu 데이터 자체 시각화는 안되고 tf 시각화만 지원

