import math
import rclpy  # Import the ROS client library for Python
from rclpy.node import Node  # Enables the use of rclpy's Node class

from geometry_msgs.msg import Twist
from std_msgs.msg import (
    Float64,
    Float64MultiArray,
)  # Enable use of the std_msgs/Float64MultiArray message type

# Twist Sub => Float64MultiArray Pub
class SRCGazeboController(Node):
    def __init__(self):
        super().__init__("src_gazebo_controller")

        self.declare_parameter('verbose', True)
        self.__verbose = self.get_parameter('verbose').value

        # Distance from Front to Rear axel
        self.declare_parameter('car_wheel_base', 0.325)
        self.__L = self.get_parameter('car_wheel_base').value

        # Distance from left to right wheels
        self.declare_parameter('car_wheel_threat', 0.245)
        self.__T = self.get_parameter('car_wheel_threat').value

        # Calculated as the maximum steering angle the inner wheel can do
        # we want maxsteer to be that of the "inside" tire, and since it is 0.6 in gazebo, we
        # set our ideal steering angle max to be less than that, based on geometry
        # 0.7853 ~ about 45 degree
        self.declare_parameter('max_abs_steer', 0.7853)
        self.__maxsteer_inside = self.get_parameter('max_abs_steer').value

        # Wheel Radius (from urdf)
        self.declare_parameter('wheel_radius', 0.05)
        self.__wheel_radius = self.get_parameter('wheel_radius').value

        # Radians per second, that with the current wheel radius would make 44 Km/h max linear vel
        # 최대속도 30km/h 기준
        # 30 * 1000 / 3600 / 0.05 = 167
        # 30 * 1000 / 3600 / 0.0508 = 164
        self.declare_parameter('max_wheel_turn_speed', 167)
        self.__max_wheel_turn_speed = self.get_parameter('max_wheel_turn_speed').value

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # rad / sec
        self.max_steering_speed = 2.0
        self.acceptable_steer_error = 0.1

        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        # radius of inside tire is rMax, so radius of the ideal middle tire (R_MIN) is rMax+treadwidth/2
        R_Min_interior = self.__L / math.tan(self.__maxsteer_inside)
        self.__R_Min_baselink = R_Min_interior + (self.__T / 2.0)
        self.get_logger().info(
            "################ MINIMUM TURNING RADIUS ACKERMAN==="
            + str(self.__R_Min_baselink)
        )

        # ROS 2 Parts
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        self.throttling_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )

        # For Odometry Implementation
        self.steering_pub_middle = self.create_publisher(
            Float64, "/steering_angle_middle", 10
        )
        self.throttling_pub_middle = self.create_publisher(
            Float64, "/throttling_vel_middle", 10
        )

        self.lastMsg = self.get_clock().now().to_msg()

        self.steering_msg = Float64MultiArray()
        self.throttling_msg = Float64MultiArray()
        self.steering_msg_middle = Float64()
        self.throttling_msg_middle = Float64()

    def cmd_vel_callback(self, data):
        """
        We get the linear velocity and the desired Turning Angular velocity.
        We have to convert it to Turning Radius
        """
        self.process_cmd_vel_data(data)
        self.publish()

    def process_cmd_vel_data(self, data):
        self.linear_velocity = data.linear.x
        # We limit the minimum value of the Steering Radius
        # Todo Process negatives

        if self.__verbose:
            self.get_logger().info("self.linear_velocity=" + str(self.linear_velocity))
            self.get_logger().info("data.angular.z=" + str(data.angular.z))

        if data.angular.z != 0.0:
            steering_radius_raw = abs(self.linear_velocity / data.angular.z)
            if self.__verbose:
                self.get_logger().info("steering_radius_raw=" + str(steering_radius_raw))
                self.get_logger().info("R_Min_baselink=" + str(self.__R_Min_baselink))

            self.steering_radius = max(abs(steering_radius_raw), self.__R_Min_baselink)
            # We consider that turning left should be positive
            # Return the value of the first parameter and the sign of the second parameter
            self.turning_sign = -1 * math.copysign(1, data.angular.z)
            # Going Fowards is positive
            self.linear_sign = math.copysign(1, self.linear_velocity)
            self.omega_turning_speed = self.linear_velocity / self.steering_radius
        else:
            self.steering_radius = -1
            self.turning_sign = 0.0
            self.linear_sign = 0.0
            self.omega_turning_speed = 0.0

        self.lastMsg = self.get_clock().now().to_msg()

    def limit_wheel_speed(self, in_speed):

        if in_speed > self.__max_wheel_turn_speed:
            self.get_logger().warn("MAX Wheel Speed!")
            in_speed = self.__max_wheel_turn_speed
        elif in_speed < -1.0 * self.__max_wheel_turn_speed:
            self.get_logger().warn("MAX Wheel Speed!")
            in_speed = -1.0 * self.__max_wheel_turn_speed

        return in_speed

    def publish(self):

        # Step 1 Calculate the Wheel Turning Speed for the Rear Wheels
        # For that we have the following input data of the base_link
        # Filtered if it was impossible for the system to do it
        # self.turning_sign states which side system want to turn
        # and who is the exterior interior wheel ( default interior = right wheel. exterior = left wheel )
        vel_base_link = self.linear_velocity
        omega_base_link = self.omega_turning_speed
        turning_radius_base_link = self.steering_radius

        turning_radius_right_rear_wheel = None
        turning_radius_left_rear_wheel = None

        if self.steering_radius >= 0:
            # Default Interior = Right WHeel
            # Make this sign multiplication because when going backwards angular is inverted, so it ha sto invert the sign
            turning_radius_right_rear_wheel = turning_radius_base_link + (
                -1 * self.turning_sign * self.linear_sign
            ) * (self.__T / 2.0)
            vel_right_rear_wheel = omega_base_link * turning_radius_right_rear_wheel
            wheel_turnig_speed_right_rear_wheel = self.limit_wheel_speed(
                vel_right_rear_wheel / self.__wheel_radius
            )

            # Default Interior = Left WHeel
            # Make this sign multiplication because when going backwards angular is inverted, so it ha sto invert the sign
            turning_radius_left_rear_wheel = turning_radius_base_link + (
                1 * self.turning_sign * self.linear_sign
            ) * (self.__T / 2.0)
            vel_left_rear_wheel = omega_base_link * turning_radius_left_rear_wheel
            wheel_turnig_speed_left_rear_wheel = self.limit_wheel_speed(
                vel_left_rear_wheel / self.__wheel_radius
            )

            turning_radius_com = turning_radius_base_link
            vel_com = omega_base_link * turning_radius_com
            wheel_turnig_speed_com = self.limit_wheel_speed(
                vel_com / self.__wheel_radius
            )
        else:
            # Not turning , there fore they are all the same
            # Default Interior = Right WHeel
            wheel_turnig_speed_right_rear_wheel = self.limit_wheel_speed(
                vel_base_link / self.__wheel_radius
            )
            # Default Interior = Left WHeel
            wheel_turnig_speed_left_rear_wheel = self.limit_wheel_speed(
                vel_base_link / self.__wheel_radius
            )

            wheel_turnig_speed_com = self.limit_wheel_speed(
                vel_base_link / self.__wheel_radius
            )

        #### END REAR WHeel Calculations
        # Step 2: Calculate the Wheel Turning Speed for the Front wheels and the Steering angle
        if self.steering_radius >= 0 and vel_base_link != 0:

            turning_radius_right_front_wheel = turning_radius_right_rear_wheel
            distance_to_turning_point_right_front_wheel = math.sqrt(
                pow(self.__L, 2) + pow(turning_radius_right_front_wheel, 2)
            )
            vel_right_front_wheel = (
                omega_base_link * distance_to_turning_point_right_front_wheel
            )

            wheel_turnig_speed_right_front_wheel = self.limit_wheel_speed(
                vel_right_front_wheel / self.__wheel_radius
            )

            # alfa_right_front_wheel = math.atan(self.__L / turning_radius_right_front_wheel)
            # alfa_right_front_wheel = math.atan( self.__L / turning_radius_middle )
            # alfa_right_front_wheel = math.atan((omega_base_link * self.__L ) / vel_base_link)
            alfa_right_front_wheel = math.asin((omega_base_link * self.__L ) / vel_base_link)

            turning_radius_left_front_wheel = turning_radius_left_rear_wheel
            distance_to_turning_point_left_front_wheel = math.sqrt(
                pow(self.__L, 2) + pow(turning_radius_left_front_wheel, 2)
            )
            vel_left_front_wheel = (
                omega_base_link * distance_to_turning_point_left_front_wheel
            )
            wheel_turnig_speed_left_front_wheel = self.limit_wheel_speed(
                vel_left_front_wheel / self.__wheel_radius
            )
            # alfa_left_front_wheel = math.atan(self.__L / turning_radius_left_front_wheel)
            # alfa_left_front_wheel = math.atan( self.__L / turning_radius_middle )
            # alfa_left_front_wheel = math.atan((omega_base_link * self.__L ) / vel_base_link)
            alfa_left_front_wheel = math.asin((omega_base_link * self.__L ) / vel_base_link)

        else:
            wheel_turnig_speed_right_front_wheel = self.limit_wheel_speed(
                vel_base_link / self.__wheel_radius
            )
            alfa_right_front_wheel = 0.0

            wheel_turnig_speed_left_front_wheel = self.limit_wheel_speed(
                vel_base_link / self.__wheel_radius
            )
            alfa_left_front_wheel = 0.0

        #### END FRONT WHeel Calculations
        if self.__verbose:
            print("#####################")
            print("@ INPUT VALUES @")
            print("vel_base_link=" + str(vel_base_link))
            print("omega_base_link=" + str(omega_base_link))
            print("turning_radius_base_link=" + str(turning_radius_base_link))
            print("@ TURNING SPEEDS @")
            print(
                "wheel_turnig_speed_right_rear_wheel="
                + str(wheel_turnig_speed_right_rear_wheel)
            )
            print(
                "wheel_turnig_speed_left_rear_wheel="
                + str(wheel_turnig_speed_left_rear_wheel)
            )
            print(
                "wheel_turnig_speed_right_front_wheel="
                + str(wheel_turnig_speed_right_front_wheel)
            )
            print(
                "wheel_turnig_speed_left_front_wheel="
                + str(wheel_turnig_speed_left_front_wheel)
            )
            print("@ ANGLES @")
            print("alfa_right_front_wheel=" + str(alfa_right_front_wheel))
            print("alfa_left_front_wheel=" + str(alfa_left_front_wheel))
            print("####### END #########")

        # Step 3 Publish all the data in the corresponding topics
        right_steering = -1 * self.turning_sign * self.linear_sign * alfa_right_front_wheel
        left_steering = -1 * self.turning_sign * self.linear_sign * alfa_left_front_wheel
        self.steering_msg.data = [
            right_steering,
            left_steering,
        ]
        raw_wheel_speed = vel_base_link / self.__wheel_radius

        self.throttling_msg.data = [
            wheel_turnig_speed_left_rear_wheel,
            wheel_turnig_speed_right_rear_wheel,
            wheel_turnig_speed_left_front_wheel,
            wheel_turnig_speed_right_front_wheel,
        ]

        self.steering_msg_middle.data = (round(right_steering, 5) + round(left_steering, 5)) / 2
        self.throttling_msg_middle.data = wheel_turnig_speed_com

        if self.__verbose:
            self.get_logger().info(f"""
                raw_wheel_speed : {raw_wheel_speed:.2}
                alfa_left_front_wheel : {alfa_left_front_wheel:.6}
                alfa_right_front_wheel : {alfa_right_front_wheel:.6}
            """)
        
        self.steering_pub.publish(self.steering_msg)
        self.steering_pub_middle.publish(self.steering_msg_middle)
        self.throttling_pub.publish(self.throttling_msg)
        self.throttling_pub_middle.publish(self.throttling_msg_middle)

def main(args=None):

    rclpy.init(args=args)

    src_gazebo_controller = SRCGazeboController()

    rclpy.spin(src_gazebo_controller)
    src_gazebo_controller.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
