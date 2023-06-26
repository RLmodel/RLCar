#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "mw_ahrsv1_ros2/MW_AHRSv1.hpp"

using namespace std::chrono_literals;

tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw) {
  float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) -
             (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
  float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) +
             (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
  float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) -
             (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
  float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) +
             (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

  tf2::Quaternion q(qx, qy, qz, qw);
  return q;
}

class MW_AHRS : public rclcpp::Node {

private:
  // ROS Parts
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_,
      imu_data_pub_;

  // hold raw data
  IMUMsg imu_raw_data;

  std::string device_id_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string pub_topic_name_; 

  bool verbose_;
  int pub_rate_;
  bool pub_tf_;
  bool view_imu_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  MW_AHRSv1 mw_ahrs_;

public:
  MW_AHRS() : Node("mw_ahrsv1_node") {

    device_id_ = declare_parameter("device_id", "/dev/MWAHRs");
    RCLCPP_INFO(get_logger(), "device_id_ : %s", device_id_.c_str());

    frame_id_ = declare_parameter("frame_id", "base_link");
    RCLCPP_INFO(get_logger(), "frame_id_ : %s", frame_id_.c_str());

    child_frame_id_ = declare_parameter("child_frame_id", "imu_link");
    RCLCPP_INFO(get_logger(), "child_frame_id_ : %s", child_frame_id_.c_str());

    pub_topic_name_ = declare_parameter("pub_topic_name", "imu/data");
    RCLCPP_INFO(get_logger(), "pub_topic_name_ : %s", pub_topic_name_.c_str());

    pub_tf_ = declare_parameter("pub_tf", true);
    RCLCPP_INFO(get_logger(), "pub_tf : %s", pub_tf_ == true ? "true" : "false");

    verbose_ = declare_parameter("verbose", true);
    RCLCPP_INFO(get_logger(), "verbose : %s", verbose_ == true ? "true" : "false");

    view_imu_ = declare_parameter("view_imu", true);
    RCLCPP_INFO(get_logger(), "view_imu : %s", view_imu_ == true ? "true" : "false");

    pub_rate_ = declare_parameter("pub_rate", 50);
    RCLCPP_INFO(get_logger(), "pub_rate : %d", pub_rate_);

    /// TOOD : publish topic 이름 parameter로 변경
    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        pub_topic_name_, rclcpp::QoS(1));

    auto interval = (1000 / pub_rate_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval),
                                     std::bind(&MW_AHRS::timer_cb, this));

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    mw_ahrs_.set_device_name(device_id_);
    mw_ahrs_.start_serial();
    mw_ahrs_.reset_angle();

    rclcpp::sleep_for(1s);

    RCLCPP_WARN(get_logger(), "Ready to Parse MW-AHRs...");
  }

  ~MW_AHRS() {
    RCLCPP_INFO(get_logger(), "Closing MW-AHRs...");

    mw_ahrs_.reset_imu();

    rclcpp::sleep_for(1s);
  }

  void timer_cb() {
    auto imu_data_msg = sensor_msgs::msg::Imu();

    mw_ahrs_.get_angle_data(imu_raw_data);

    // Debugging Console
    if (verbose_)
      RCLCPP_INFO(get_logger(), "\nimu_raw_data.roll : %f \
        \nimu_raw_data.pitch : %f \
        \nimu_raw_data.yaw : %f",
                  imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    tf2::Quaternion orientation = Euler2Quaternion(
        imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw);

    tf2::Quaternion yaw_rotate;
    yaw_rotate.setRPY(0, 0, 90 * convertor_d2r);

    tf2::Quaternion q;
    q.setRPY(0, 0, -90 * convertor_d2r);

    // Apply rotation for 90 degree.
    tf2::Quaternion new_orientation = q * orientation * yaw_rotate;

    rclcpp::Time now = this->now();

    imu_data_msg.header.stamp = now;
    imu_data_msg.header.frame_id = frame_id_;

    // orientation
    imu_data_msg.orientation.x = new_orientation[0];
    imu_data_msg.orientation.y = new_orientation[1];
    imu_data_msg.orientation.z = new_orientation[2];
    imu_data_msg.orientation.w = -new_orientation[3];
    // imu_data_msg.orientation_covariance[0] = -1; // we don't have estimation for orientation

    // original data used the g unit, convert to m/s^2
    imu_data_msg.linear_acceleration.x =
        -imu_raw_data.linear_acc_y * convertor_g2a;
    imu_data_msg.linear_acceleration.y =
        imu_raw_data.linear_acc_x * convertor_g2a;
    imu_data_msg.linear_acceleration.z =
        imu_raw_data.linear_acc_z * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_msg.angular_velocity.x =
        imu_raw_data.angular_vel_x * convertor_d2r;
    imu_data_msg.angular_velocity.y =
        imu_raw_data.angular_vel_y * convertor_d2r;
    imu_data_msg.angular_velocity.z =
        imu_raw_data.angular_vel_z * convertor_d2r;

    imu_data_msg.linear_acceleration_covariance[0] =
        imu_data_msg.linear_acceleration_covariance[4] =
            imu_data_msg.linear_acceleration_covariance[8] = 1000;

    imu_data_msg.angular_velocity_covariance[0] =
        imu_data_msg.angular_velocity_covariance[4] =
            imu_data_msg.angular_velocity_covariance[8] = 1;

    imu_data_msg.orientation_covariance[0] =
        imu_data_msg.orientation_covariance[4] =
            imu_data_msg.orientation_covariance[8] = 0;

    if (pub_tf_) {
      geometry_msgs::msg::TransformStamped transform;

      transform.header.stamp = now;
      transform.header.frame_id = frame_id_;
      transform.child_frame_id = child_frame_id_;

      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;

      if (view_imu_) {
        transform.transform.rotation.x = new_orientation[0];
        transform.transform.rotation.y = new_orientation[1];
        transform.transform.rotation.z = new_orientation[2];
        transform.transform.rotation.w = -new_orientation[3];
      } else {
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
      }

      broadcaster_->sendTransform(transform);
    }

    imu_data_pub_->publish(imu_data_msg);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto imu_node = std::make_shared<MW_AHRS>();

  rclcpp::spin(imu_node);

  rclcpp::shutdown();

  return 0;
}