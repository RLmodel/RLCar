/// 각도 말고 다른 값들도 받기 
#include <string>
#include <memory>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "mw_ahrsv1_ros2/Serial.h"

using namespace std::chrono_literals;
typedef struct {
  // ang
  float roll;
  float pitch;
  float yaw;

  // acc
  float linear_acc_x;
  float linear_acc_y;
  float linear_acc_z;

  // gyr
  float angular_vel_x;
  float angular_vel_y;
  float angular_vel_z;

} IMUMsg;

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
  // Device Name
  int serial_id = 0;

  // Data buffer
  char buffer[120];
  char small_buffer[10];

  // utils for serial cmd
  unsigned char angle_cmd[5] = {0x61, 0x6E, 0x67, 0x0D, 0x0A}; // ang Enter
  unsigned char gyr_cmd[5] = {0x67, 0x79, 0x72, 0x0D, 0x0A};   // ang Enter
  unsigned char acc_cmd[5] = {0x61, 0x63, 0x63, 0x0D, 0x0A};   // ang Enter
  unsigned char reset_angle_cmd[5] = {0x7A, 0x72, 0x6F, 0x0D, 0x0A}; // zro Enter
  unsigned char reset_cmd[5] = {0x72, 0x73, 0x74, 0x0D, 0x0A}; // zro Enter
  unsigned char av_cmd[7] = {0x61, 0x76, 0x3D, 0x31,
                             0x30, 0x0D, 0x0A}; // av = 10
  unsigned char speed_cmd_slow[8] = {0x73, 0x70, 0x3D, 0x31,
                                0x30, 0x30, 0x0D, 0x0A}; // sp=100 Enter
  unsigned char speed_cmd[7] = {0x73, 0x70, 0x3D, 0x32,
                                0x30, 0x0D, 0x0A}; // sp=20 Enter
  unsigned char ros_data_cmd[6] = {0x73, 0x73, 0x3D,
                                   0x37, 0x0D, 0x0A}; // ss=7 Enter 가속도, 각속도, 각도 데이터

  // Serperate Euler Angle Variable
  int ang_count = 0;

  // ASCII CODE
  const unsigned char A = 0x61;
  const unsigned char N = 0x6E;
  const unsigned char G = 0x67;
  const unsigned char CR = 0x0D;
  const unsigned char LF = 0x0A;

  // Unit converting constants
  double convertor_g2a = 9.80665; // for linear_acceleration (g to m/s^2)
  double convertor_d2r =
      M_PI / 180.0; // for angular_velocity (degree to radian)
  double convertor_r2d =
      180.0 / M_PI;                // for easy understanding (radian to degree)
  double convertor_ut2t = 1000000; // for magnetic_field (uT to Tesla)
  double convertor_c = 1.0;        // for temperature (celsius)

  // ROS Parts
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_,
      imu_data_pub_;

  // hold raw data
  IMUMsg imu_raw_data;
  std::string device_id_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool verbose_;
  int pub_rate;
  bool pub_tf;
  bool view_imu_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

public:
  MW_AHRS() : Node("mv_ahrsv1_node") {

    this->declare_parameter("deviceID", "/dev/MWAHRs");
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("child_frame_id", "imu_link");
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("view_imu", false);
    this->declare_parameter("verbose", true);
    this->declare_parameter("publish_rate", 50);

    rclcpp::Parameter deviceID = this->get_parameter("deviceID");
    rclcpp::Parameter frame_id = this->get_parameter("frame_id");
    rclcpp::Parameter child_frame_id = this->get_parameter("child_frame_id");
    rclcpp::Parameter publish_tf = this->get_parameter("publish_tf");
    rclcpp::Parameter verbose = this->get_parameter("verbose");
    rclcpp::Parameter view_imu = this->get_parameter("view_imu");
    rclcpp::Parameter publish_rate = this->get_parameter("publish_rate");
    
    device_id_ = deviceID.as_string();
    frame_id_ = frame_id.as_string();
    child_frame_id_ = child_frame_id.as_string();
    pub_tf = publish_tf.as_bool();
    verbose_ = verbose.as_bool();
    view_imu_ = view_imu.as_bool();
    pub_rate = publish_rate.as_int();
    
    RCLCPP_INFO(this->get_logger(), "deviceID: %s, publish_tf: %s, verbose: %s, publish_rate: %s, frame_id: %s, child_frame_id: %s",
      deviceID.value_to_string().c_str(),
      publish_tf.value_to_string().c_str(),
      verbose.value_to_string().c_str(),
      publish_rate.value_to_string().c_str(),
      frame_id.value_to_string().c_str(),
      child_frame_id.value_to_string().c_str()
    );

    for (auto i = 0; i < sizeof(buffer); i++)
      buffer[i] = 0;

    // int open_serial(char *dev_name, int baud, int vtime, int vmin);
    serial_id = open_serial(const_cast<char*>(device_id_.c_str()), 115200, 0, 0);

    imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::QoS(1)
    );

    auto interval = (1000 / pub_rate);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval),
                                     std::bind(&MW_AHRS::timer_cb, this));

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    this->reset_angle();
    this->speed_setup();
    this->start_data_stream();
    rclcpp::sleep_for(1s);

    RCLCPP_WARN(get_logger(), "Ready to Parse MwAHRs...");
  }

  ~MW_AHRS() {
    RCLCPP_INFO(get_logger(), "Destructor...");
    
    this->reset_imu();
    close_serial(serial_id);
    
    rclcpp::sleep_for(1s);
  }

  void reset_angle() {
    write(serial_id, reset_angle_cmd, 5);
    read(serial_id, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    for(auto i=0 ; i < sizeof(buffer) ; i++)
      std::cout << (int)buffer[i] << " ";
    std::cout << std::endl;
    if( (int)buffer[0] != 0 )
      RCLCPP_INFO(get_logger(), "IMU Angle Reset Done...");

    // TODO: Initialization Test
    // if (buffer[0] != 'z' && buffer[1] == 'r' && buffer[2] == 'o')
    //   RCLCPP_INFO(get_logger(), "IMU Reset...");
  }

  void reset_imu() {
    write(serial_id, reset_cmd, 5);
    read(serial_id, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    for(auto i=0 ; i < sizeof(buffer) ; i++)
      std::cout << (int)buffer[i] << " ";
    std::cout << std::endl;
    if( (int)buffer[0] != 0 )
      RCLCPP_INFO(get_logger(), "IMU Reset Done...");

    // TODO: Initialization Test
    // if (buffer[0] != 'z' && buffer[1] == 'r' && buffer[2] == 'o')
    //   RCLCPP_INFO(get_logger(), "IMU Reset...");
  }

  void speed_setup() {
    write(serial_id, speed_cmd, 7);
    read(serial_id, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';

    for(auto i=0 ; i < sizeof(buffer) ; i++)
      std::cout << (int)buffer[i] << " ";
    std::cout << std::endl;
    if( (int)buffer[0] != 0 )
      RCLCPP_INFO(get_logger(), "Speed Setup Done...");

    // TODO: Initialization Test
    // Ex) sp=100 데이터 전송 주기를 100ms로 설정
    // if (buffer[0] == 's' && buffer[1] == 'p' && buffer[2] == '=' &&
    //     buffer[3] == '1' && buffer[4] == '0')
    //   RCLCPP_INFO(get_logger(), "Speed Setup Done...");
  }

  void start_data_stream() {
    write(serial_id, ros_data_cmd, 6);
    read(serial_id, &buffer, sizeof(buffer));

    buffer[sizeof(buffer)] = '\0';
  }

  void get_angle_data(IMUMsg &msg_in) {
    write(serial_id, angle_cmd, 5);
    read(serial_id, &buffer, sizeof(buffer));

    if (buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g') {
      char *ptr = strtok(buffer, " ");

      ang_count = 0;

      while (ptr != NULL) {
        ang_count++;

        ptr = strtok(NULL, " ");

        if (ang_count == 1) {
          msg_in.roll = atof(ptr);
        } else if (ang_count == 2) {
          msg_in.pitch = atof(ptr);
        } else if (ang_count == 3) {
          msg_in.yaw = atof(ptr);
        }
      }
    }
  }

  void parse_ss_data(IMUMsg &msg_in) {
    read(serial_id, &buffer, sizeof(buffer));

    if (int(buffer[0]) < 97) {
      char *rest;
      char *token;
      char *ptr = buffer;

      ang_count = 0;

      while (token = strtok_r(ptr, " ", &rest)) {
        ang_count++;

        if (ang_count == 1) {
          msg_in.linear_acc_x = atof(token);
        } else if (ang_count == 2) {
          msg_in.linear_acc_y = atof(token);
        } else if (ang_count == 3) {
          msg_in.linear_acc_z = atof(token);
        } else if (ang_count == 4) {
          msg_in.angular_vel_x = atof(token);
        } else if (ang_count == 5) {
          msg_in.angular_vel_y = atof(token);
        } else if (ang_count == 6) {
          msg_in.angular_vel_z = atof(token);
        } else if (ang_count == 7) {
          msg_in.roll = atof(token) * -convertor_d2r;
        } else if (ang_count == 8) {
          msg_in.pitch = atof(token) * -convertor_d2r;
        } else if (ang_count == 9) {
          msg_in.yaw = atof(token) * -convertor_d2r;
        }
        ptr = rest;
      }
    }
  }

  void timer_cb() {
    auto imu_data_msg = sensor_msgs::msg::Imu();

    parse_ss_data(imu_raw_data);

    tf2::Quaternion orientation = Euler2Quaternion(
      imu_raw_data.roll, imu_raw_data.pitch, imu_raw_data.yaw
    );

    tf2::Quaternion yaw_rotate;
    yaw_rotate.setRPY(0, 0, 90 * convertor_d2r);

    tf2::Quaternion q;
    q.setRPY(0, 0, -90 * convertor_d2r);

    // Apply rotation for 90 degree.
    tf2::Quaternion new_orientation = q * orientation * yaw_rotate;

    // Debugging Console
    if(verbose_){
      RCLCPP_INFO(get_logger(), "imu_raw_data.yaw : %f", imu_raw_data.yaw);
      // std::cout << "imu_raw_data.yaw : " << imu_raw_data.yaw << std::endl;
      // std::cout << new_orientation[0] << std::endl;
      // std::cout << new_orientation[1] << std::endl;
      // std::cout << new_orientation[2] << std::endl;
    }

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

    if (pub_tf) {
      geometry_msgs::msg::TransformStamped transform;

      transform.header.stamp = now;
      transform.header.frame_id = frame_id_;
      transform.child_frame_id = child_frame_id_;

      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;

      if(view_imu_){
        transform.transform.rotation.x = new_orientation[0];
        transform.transform.rotation.y = new_orientation[1];
        transform.transform.rotation.z = new_orientation[2];
        transform.transform.rotation.w = -new_orientation[3];
      }
      else {
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