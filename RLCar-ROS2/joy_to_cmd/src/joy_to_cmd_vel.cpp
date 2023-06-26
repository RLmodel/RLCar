#include <memory>
#include <string.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

using UInt8 = std_msgs::msg::UInt8;
using Float32 = std_msgs::msg::Float32;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;

inline bool isTrue(const int &val_in) { return (val_in == 1) ? true : false; }

class JoyToCmd : public rclcpp::Node {
public:
  struct XMode {
    float left_updown;
    float left_leftright;

    float right_updown;
    float right_leftright;

    int btn_updown;
    int btn_leftright;

    bool btn_a;
    bool btn_b;
    bool btn_x;
    bool btn_y;

    bool btn_LB;
    bool btn_RB;

    bool btn_back;
    bool btn_start;
  };

  enum class ControlMode {
    JOYCON_CONTROL,
    JOYBTN_CONTROL,
  };

private:
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<UInt8>::SharedPtr src_mode_pub;
  rclcpp::Publisher<Float32>::SharedPtr accel_pub;

  rclcpp::Subscription<Joy>::SharedPtr joy_sub;

  Twist twist;
  UInt8 src_mode;
  Float32 accel;

  XMode prev_joy_keys;
  XMode joy_keys;

  ControlMode control_mode_;

  float linear_speed_gain = 0.5;
  const float angular_pose_gain = 2.0;
  const float btn_control_gain = 0.0625f;

public:
  JoyToCmd() : Node("joy_to_cmd_vel_node") {
    // TODO : QOS Profile
    cmd_vel_pub = create_publisher<Twist>("cmd_vel", 5);
    src_mode_pub = create_publisher<UInt8>("src_mode", 5);
    accel_pub = create_publisher<Float32>("accel_vel", 5);

    joy_sub = create_subscription<Joy>(
        "joy", 10,
        std::bind(&JoyToCmd::sub_callback, this, std::placeholders::_1));

    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    accel.data = 5.0f;

    src_mode.data = 1;

    control_mode_ = ControlMode::JOYCON_CONTROL;
  }

  void sub_callback(const Joy::SharedPtr data) {
    // Assume JoyStick is on "X" mode
    joy_keys.left_updown = data->axes[1];
    joy_keys.left_leftright = data->axes[0];
    joy_keys.right_updown = data->axes[4];
    joy_keys.right_leftright = data->axes[3];

    // +1 btn up / -1 btn down
    joy_keys.btn_updown = data->axes[7];
    // +1 btn left / -1 btn right
    joy_keys.btn_leftright = data->axes[6];

    // buttons for mode change

    // A mode - joycon mode
    joy_keys.btn_a = isTrue(data->buttons[0]);

    // B mode - button mode
    joy_keys.btn_b = isTrue(data->buttons[1]);

    joy_keys.btn_x = isTrue(data->buttons[2]);
    joy_keys.btn_y = isTrue(data->buttons[3]);

    // Linear Speed UP/Down
    joy_keys.btn_LB = isTrue(data->buttons[4]);
    joy_keys.btn_RB = isTrue(data->buttons[5]);

    joy_keys.btn_back = isTrue(data->buttons[6]);
    joy_keys.btn_start = isTrue(data->buttons[7]);

    if (joy_keys.btn_a == 1.0 && prev_joy_keys.btn_a == 0.0)
      control_mode_ = ControlMode::JOYCON_CONTROL;
    if (joy_keys.btn_b == 1.0 && prev_joy_keys.btn_b == 0.0)
      control_mode_ = ControlMode::JOYBTN_CONTROL;

    if (control_mode_ == ControlMode::JOYCON_CONTROL)
      pub_twist_joy();
    else
      pub_twist_btn();

    pub_mode();
    pub_accel();

    prev_joy_keys = joy_keys;
  }

  void pub_twist_btn() {

    if (joy_keys.btn_updown == 1.0 && prev_joy_keys.btn_updown == 0.0)
      twist.linear.x += btn_control_gain;
    if (joy_keys.btn_updown == -1.0 && prev_joy_keys.btn_updown == 0.0)
      twist.linear.x -= btn_control_gain;

    if (joy_keys.btn_leftright == 1.0 && prev_joy_keys.btn_leftright == 0.0)
      twist.angular.z += btn_control_gain;
    if (joy_keys.btn_leftright == -1.0 && prev_joy_keys.btn_leftright == 0.0)
      twist.angular.z -= btn_control_gain;

    cmd_vel_pub->publish(twist);
  }

  void pub_twist_joy() {
    // floating point number considered
    if (joy_keys.btn_LB && !prev_joy_keys.btn_LB)
      linear_speed_gain -= 0.125f;
    if (joy_keys.btn_RB && !prev_joy_keys.btn_RB)
      linear_speed_gain += 0.125f;

    twist.linear.x = joy_keys.left_updown * linear_speed_gain;
    twist.linear.y = joy_keys.left_leftright;
    twist.linear.z = 0.0f;

    twist.angular.z = joy_keys.right_leftright * angular_pose_gain;

    cmd_vel_pub->publish(twist);
  }

  void pub_mode() {
    // Emergency Stop
    if (joy_keys.btn_x)
      src_mode.data = 1;
    if (joy_keys.btn_a)
      src_mode.data = 2;
    if (joy_keys.btn_b)
      src_mode.data = 3;
    if (joy_keys.btn_y)
      src_mode.data = 4;

    src_mode_pub->publish(src_mode);
  }

  void pub_accel() {
    // Tune acc / deacc values for your favorite

    // if (joy_keys.btn_updown == 1.0 && prev_joy_keys.btn_updown == 0.0)
    //   accel.data += 0.125f;
    // if (joy_keys.btn_updown == -1.0 && prev_joy_keys.btn_updown == 0.0)
    //   accel.data -= 0.125f;

    // accel_pub->publish(accel);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto joy_to_cmd = std::make_shared<JoyToCmd>();

  rclcpp::spin(joy_to_cmd);
  rclcpp::shutdown();

  return 0;
}