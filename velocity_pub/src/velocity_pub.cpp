// Restamp Nav2's Twist as a TwistStamped for diff_drive_controller, which on
// Jazzy accepts nothing else.
//
// This used to publish from a 500 ms wall timer, holding the last command it
// had heard. That is left over from rclcpp's minimal_publisher example, and it
// meant a 20 Hz command stream reached the wheels at 2 Hz: nine commands in ten
// discarded, and each one that survived held for half a second. Every
// controller parameter downstream describes a 20 Hz loop, so they were all
// describing something that was not happening -- at 0.4 m/s the robot covered
// 20 cm between command updates, which is how it clipped a gate wall it had
// plenty of room for.
//
// Publishing on receipt keeps the rate and the timing the controller chose.

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class VelocityStamper : public rclcpp::Node
{
public:
  VelocityStamper()
  : Node("velocity_pub")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel_stamped", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped out;
        // get_clock() so the stamp follows simulated time when use_sim_time is
        // set; the wall timer this replaces did not.
        out.header.stamp = this->get_clock()->now();
        out.twist = *msg;
        publisher_->publish(out);
      });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityStamper>());
  rclcpp::shutdown();
  return 0;
}
