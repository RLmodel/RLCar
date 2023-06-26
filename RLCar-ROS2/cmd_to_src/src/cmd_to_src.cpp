
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <src_control_message/msg/src_msg.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

using Twist = geometry_msgs::msg::Twist;
using SRCMsg = src_control_message::msg::SRCMsg;
using Float32 = std_msgs::msg::Float32;
using Imu = sensor_msgs::msg::Imu;

template <typename T>
inline void in_range(T &input, const T &low_val, const T &max_val) {
  if (input < low_val)
    input = low_val;
  if (input > max_val)
    input = max_val;
}

/// TODO : friend class CmdToSRC;
class YawRatePID : public rclcpp::Node {
public:
  enum TwiddleCase {
    CASE_P_1 = 0,
    CASE_P_2,
    CASE_I_1,
    CASE_I_2,
    CASE_D_1,
    CASE_D_2,
  };

private:
  float k_p, k_i, k_d;
  float d_p, d_i, d_d;
  float prev_err = 0.0f;
  float integrate_err = 0.0f;

  float best_err = 1000.0f;

  TwiddleCase twiddle_case_ = CASE_P_1;

public:
  YawRatePID(const float &k_p = 18.0, const float &k_i = -10.0,
             const float &k_d = -9.0)
      : Node("yaw_ctl_pid"), k_p(k_p), k_i(k_i), k_d(k_d) {
    d_p = 1.0;
    d_i = 1.0;
    d_d = 1.0;
  }

  void twiddle(const float &cur_err) {
    if ((d_p + d_i + d_d) > 0.2) {
      if (cur_err < best_err) {
        best_err = cur_err;
        switch (twiddle_case_) {
        case CASE_P_1:
          d_p *= 1.1;
          twiddle_case_ = CASE_I_1;
          break;
        case CASE_P_2:
          d_p *= 1.1;
          twiddle_case_ = CASE_I_1;
          break;
        case CASE_I_1:
          d_i *= 1.1;
          twiddle_case_ = CASE_D_1;
          break;
        case CASE_I_2:
          d_i *= 1.1;
          twiddle_case_ = CASE_D_1;
          break;
        case CASE_D_1:
          d_d *= 1.1;
          twiddle_case_ = CASE_P_1;
          break;
        case CASE_D_2:
          d_d *= 1.1;
          twiddle_case_ = CASE_P_1;
          break;
        default:
          break;
        }
      } else {
        switch (twiddle_case_) {
        case CASE_P_1:
          k_p -= 2 * d_p;
          twiddle_case_ = CASE_P_2;
          break;
        case CASE_P_2:
          k_p += d_p;
          d_p *= 0.9;
          twiddle_case_ = CASE_I_1;
          break;
        case CASE_I_1:
          k_i -= 2 * d_i;
          twiddle_case_ = CASE_I_2;
          break;
        case CASE_I_2:
          k_i += d_i;
          d_i *= 0.9;
          twiddle_case_ = CASE_D_1;
          break;
        case CASE_D_1:
          k_d -= 2 * d_d;
          twiddle_case_ = CASE_D_2;
          break;
        case CASE_D_2:
          k_d += d_d;
          d_d *= 0.9;
          twiddle_case_ = CASE_P_1;
          break;
        default:
          break;
        }
      }
    }
  }

  void setGain(const float &k_p_in, const float &k_i_in, const float &k_d_in) {
    k_p = k_p_in;
    k_i = k_i_in;
    k_d = k_d_in;
  }

  std::vector<float> getGain() const {
    return std::vector<float>{k_p, k_i, k_d};
  }

  int update(bool use_twiddle, bool sign_change, float object_val,
             float cur_val) {

    auto cur_err = object_val - cur_val;
    auto diff_err = cur_err - prev_err;

    integrate_err += cur_err;
    if (sign_change) {
      integrate_err = 0;
      diff_err = 0;
    }

    in_range(integrate_err, -10.0f, 10.0f);

    if (use_twiddle)
      twiddle(cur_err);

    prev_err = cur_err;

    RCLCPP_INFO(get_logger(), "cur_err : %f", cur_err);

    int output = -k_p * cur_err - k_i * integrate_err - k_d * diff_err;

    in_range(output, -30, 30);

    std::cout << "output : " << int(output) << std::endl;

    std::cout << "\nk_p : " << k_p << "\n k_i : " << k_i << "\n k_d : " << k_d
              << std::endl;

    return output;
  }

  ~YawRatePID() {}
};

class CmdToSRC : public rclcpp::Node {
private:
  rclcpp::Publisher<SRCMsg>::SharedPtr src_msg_pub_;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<Float32>::SharedPtr accel_vel_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  rclcpp::Time prev_time;
  double prev_heading_;
  double yaw_speed_;

  // TODO: src_msg params & yaml file (for accel, daccel, etc...)
  SRCMsg src_msg_;

  // double accel_;
  // double deaccel_;

  double linear_x_;
  double angular_z_;
  
  float prev_angular_vel_;

  bool use_twiddle_;
  unsigned int scale;
  unsigned int steering_offset_;
  
  std::shared_ptr<YawRatePID> pid_controller_;

public:
  CmdToSRC() : Node("cmd_vel_to_src_msg") {
    RCLCPP_INFO(get_logger(), "Cmd_Vel to SRC_Msg Node Created");

    pid_controller_ = std::make_shared<YawRatePID>();

    src_msg_pub_ = create_publisher<SRCMsg>("src_control", 10);

    imu_sub_ = create_subscription<Imu>(
        "imu/data", 10,
        std::bind(&CmdToSRC::imu_cb, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<Twist>(
        "cmd_vel", 10,
        std::bind(&CmdToSRC::cmd_vel_cb, this, std::placeholders::_1));

    accel_vel_sub_ = create_subscription<Float32>(
        "accel_vel", 10,
        std::bind(&CmdToSRC::accel_vel_cb, this, std::placeholders::_1));

    pub_timer_ = this->create_wall_timer(
        100ms, std::bind(&CmdToSRC::timer_callback, this));

    auto accel_ = declare_parameter("accel_scale", 5.0);
    RCLCPP_INFO(get_logger(), "accel : %f", accel_);

    auto deaccel_ = declare_parameter("deaccel_scale", 5.0);
    RCLCPP_INFO(get_logger(), "deaccel : %f", deaccel_);

    scale = declare_parameter("scale", 19);
    RCLCPP_INFO(get_logger(), "scale : %d", scale);
    
    steering_offset_ = declare_parameter("steering_offset", 20);
    RCLCPP_INFO(get_logger(), "steering_offset : %d", steering_offset_);

    auto p_gain_ = declare_parameter("p_gain", 18.0);
    RCLCPP_INFO(get_logger(), "p_gain : %f", p_gain_);

    auto i_gain_ = declare_parameter("i_gain", 0.0);
    RCLCPP_INFO(get_logger(), "i_gain : %f", i_gain_);

    auto d_gain_ = declare_parameter("d_gain", 18.0);
    RCLCPP_INFO(get_logger(), "d_gain : %f", d_gain_);

    use_twiddle_ = declare_parameter("use_twiddle", true);
    RCLCPP_INFO(get_logger(), "use_twiddle : %s", use_twiddle_ == true ? "true" : "false");


    src_msg_.speed = 0.0;
    src_msg_.steering = 0;
    src_msg_.light = false;
    src_msg_.direction = true;
    src_msg_.lcd_msg = "";
    src_msg_.accel = accel_;
    src_msg_.deaccel = deaccel_;
    src_msg_.scale = scale;

    prev_time = this->get_clock()->now();
    prev_heading_ = 0.0;

    pid_controller_->setGain(p_gain_, i_gain_, d_gain_);
  }

  // change to odom callback
  void imu_cb(const Imu::SharedPtr msg) {

    auto current_time = this->get_clock()->now();
    const double dt = (current_time - prev_time).seconds();
    // RCLCPP_INFO(get_logger(), "dt : %f", dt); // dt 0.02 ok

    prev_time = current_time;

    tf2::Quaternion q_;

    q_[0] = msg->orientation.x;
    q_[1] = msg->orientation.y;
    q_[2] = msg->orientation.z;
    q_[3] = msg->orientation.w;

    tf2::Matrix3x3 m(q_);
    double roll, pitch, heading;
    m.getRPY(roll, pitch, heading);

    yaw_speed_ = (heading - prev_heading_) / dt;
    prev_heading_ = heading;

    // RCLCPP_INFO(get_logger(), "yaw : %f", heading);
    // RCLCPP_INFO(get_logger(), "angular_z : %f", yaw_speed_);
  }

  void update_steering(){

    if (fabs(linear_x_) >= 0.02) {
      bool sign_change = false;
      int pid_result;

      if ((prev_angular_vel_ > 0 && angular_z_ < 0) ||
          (prev_angular_vel_ < 0 && angular_z_ > 0)) {
        RCLCPP_INFO(get_logger(), "sign changed");
        sign_change = true;
      }

      if (angular_z_ == 0.0)
        pid_result = steering_offset_;
      else {
        pid_result = steering_offset_ +
                     pid_controller_->update(use_twiddle_, sign_change,
                                             angular_z_, yaw_speed_);
      }

      in_range(pid_result, 0, 200);

      src_msg_.steering = pid_result;
      prev_angular_vel_ = angular_z_;
    }
    // src_msg_.steering = steering_offset_;
    else
      src_msg_.steering = steering_offset_;
  }

  void cmd_vel_cb(const Twist::SharedPtr msg) {
    linear_x_ = msg->linear.x;
    angular_z_ = msg->angular.z;

    src_msg_.speed = linear_x_;
    src_msg_.direction = src_msg_.speed >= 0 ? 1 : 0;

    if (linear_x_ < 0)
      angular_z_ *= -1;

    // RCLCPP_INFO(get_logger(), "steering : %d", src_msg_.steering); // dt 0.02
    // ok

    // src_msg_.steering =
    //     -((msg->angular.z) / 2 * steering_offset_) + steering_offset_;

    // TODO: Light On Off mode
  }

  void accel_vel_cb(const Float32::SharedPtr msg) {
    src_msg_.accel = msg->data;
    src_msg_.deaccel = msg->data;
  }

  void timer_callback() {
    update_steering(); 
    src_msg_pub_->publish(src_msg_); 
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto cmd_to_src = std::make_shared<CmdToSRC>();

  rclcpp::spin(cmd_to_src);
  rclcpp::shutdown();

  return 0;
}
