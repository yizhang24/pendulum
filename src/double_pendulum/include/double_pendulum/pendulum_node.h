#ifndef PENDULUM_NODE
#define PENDULUM_NODE

#include <rclcpp/node.hpp>
#include "double_pendulum/msg/pendulum_state.hpp"
#include "double_pendulum/pendulum.h"
class PendulumNode : public rclcpp::Node {
  public:
  PendulumNode();
  private:
  Pendulum state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<double_pendulum::msg::PendulumState>::SharedPtr publisher_;
  // rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  void publish_state(const Pendulum& s);
};
#endif
