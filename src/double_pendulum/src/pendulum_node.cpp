#include "double_pendulum/pendulum_node.h"
#include <rclcpp/executors.hpp>
using namespace std::chrono_literals;

PendulumNode::PendulumNode() : rclcpp::Node("pendulum_node") {
  this->publisher_ =
      this->create_publisher<double_pendulum::msg::PendulumState>(
          "pendulum_state", 10);
  auto timer_callback = [this]() -> void {
    // this->state_.omega1 *= 0.99;
    // this->state_.omega2 *= 0.99;
    this->state_ = rk4_step(this->state_, 0.02, 1.0, 1.0, 2.0, 2.0, 9.81);
    this->publish_state(this->state_);
  };
  this->timer_ = create_wall_timer(2ms, timer_callback);
  state_ = {M_PI / 2, M_PI / 2, 0.0, 0.0};
}

void PendulumNode::publish_state(const Pendulum &s) {
  auto msg = double_pendulum::msg::PendulumState();
  msg.set__omega1(s.omega1);
  msg.set__omega2(s.omega2);
  msg.set__theta1(s.theta1);
  msg.set__theta2(s.theta2);
  this->publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PendulumNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
