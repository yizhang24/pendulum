#include "double_pendulum/pendulum_node.h"
#include <chrono>
#include <rclcpp/executors.hpp>
#include <cmath>
using namespace std::chrono_literals;

PendulumNode::PendulumNode() : rclcpp::Node("pendulum_node") {
  this->dt_ms = 2;
  this->publisher_ =
      this->create_publisher<double_pendulum::msg::PendulumState>(
          "pendulum_state", 10);
  auto timer_callback = [this]() -> void {
    this->state_ = rk4_step(this->state_, this->dt_ms / 1000.0, 1.0, 1.0, 2.0, 2.0, 9.81);
    this->publish_state(this->state_);
  };
  this->timer_ = create_wall_timer(std::chrono::milliseconds(this->dt_ms), timer_callback);
  state_ = {M_PI / 4.0, 0.0, 0.0, 0.0};
}

double rad_to_degrees(double rad) {
  double degrees = std::fmod((rad * (180.0 / M_PI)), 360.0);
  // if (degrees < 0.0) degrees += 360.0;
  return degrees;
}

void PendulumNode::publish_state(const Pendulum &s) {
  auto msg = double_pendulum::msg::PendulumState();
  msg.set__omega1(rad_to_degrees(s.omega1));
  msg.set__omega2(rad_to_degrees(s.omega2));
  msg.set__theta1(rad_to_degrees(s.theta1));
  msg.set__theta2(rad_to_degrees(s.theta2));
  this->publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PendulumNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
