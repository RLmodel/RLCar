#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

typedef nav_msgs::msg::Odometry Odometry;

class NavVelUtil : public rclcpp::Node {

private:
  Odometry odom_msg_sub_;

  float x_diff;
  float prev_x;
  rclcpp::Time prev_time;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

public:
  NavVelUtil() : Node("nav_msgs_to_vel_util") {
    odom_sub_ = this->create_subscription<Odometry>(
        "odom", 10,
        std::bind(&NavVelUtil::odom_sub_cb, this, std::placeholders::_1));

    auto interval = (1000 / 50);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval),
                                     std::bind(&NavVelUtil::timer_cb, this));

    prev_time = this->get_clock()->now();
    x_diff = 0.0;
    prev_x = 0.0;
  }

  void timer_cb(){
    rclcpp::Time now = this->get_clock()->now();
    auto time_diff = (now - prev_time).seconds();
    prev_time = now;
    
    if(fabs(x_diff) > 1e-6){
        auto velocity = (x_diff / time_diff);
        std::cout << "\ntime_diff : " << time_diff 
                << "\nx_diff : " << x_diff
                << "\nvelocity : " << velocity << std::endl;
    }
  }

  void odom_sub_cb(const Odometry::SharedPtr odom_msg) {
    auto x_now = odom_msg->pose.pose.position.x;
    x_diff = x_now - prev_x;
    std::cout << "x_now : " << x_now << 
        "\nprev_x : " << prev_x << std::endl;
    
    prev_x = x_now;

  }

  ~NavVelUtil() {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<NavVelUtil>());

  rclcpp::shutdown();

  return 0;
}