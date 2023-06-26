저속 모드를 위해 acc 값을 변경하는 매개변수를 joystick control foxy launch에 추가함 acc, deacc를 0.1로 해보라고 하네 

low speed test


docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch cmd_to_src cmd_to_src.launch.py 
ros2 topic echo /encoder_value

ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 topic echo /src_control
