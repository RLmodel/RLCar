import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class BasicControlNode(Node):

    runningTime = 5.0

    def __init__(self):
        super().__init__("basic_control_node")

        self.__pub_msg = Twist()
        self.__publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("Odom Test Node prepared")

    def pub_cmd_vel(self):
        self.__pub_msg.linear.x = 0.1
        self.__pub_msg.angular.z = 0.0
        self.__publisher.publish(self.__pub_msg)

    def stop_robot(self):
        self.__pub_msg.linear.x = 0.0
        self.__pub_msg.angular.z = 0.0
        self.__publisher.publish(self.__pub_msg)

def main(args=None):

    rclpy.init(args=args)
    racecar_controller = BasicControlNode()

    start = time.monotonic()
    print(time.monotonic() - start)

    while time.monotonic() - start < racecar_controller.runningTime:
        racecar_controller.pub_cmd_vel()
        print(time.monotonic() - start)
    racecar_controller.stop_robot()

    racecar_controller.get_logger().warn("Publish Done")
    racecar_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
