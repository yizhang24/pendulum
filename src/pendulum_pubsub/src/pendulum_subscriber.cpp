#include "messages/msg/pendulum_state.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
class PendulumSubscriber : public rclcpp::Node {
  public:
  PendulumSubscriber() : Node("pendulum_subscriber") {
    subscription_ = this->create_subscription<messages::msg::PendulumState>(
      "pendulum_angle",
      10,
      std::bind(&PendulumSubscriber::subscriber_callback, this, std::placeholders::_1)
    );
  }

  private:
  rclcpp::Subscription<messages::msg::PendulumState>::SharedPtr subscription_;
  void subscriber_callback(messages::msg::PendulumState::UniquePtr msg) {
    RCLCPP_INFO(this->get_logger(), "Theta 1 is %.3f, theta 2 is %.3f", msg->theta1, msg->theta2);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumSubscriber>());
  rclcpp::shutdown();
  return 0;
}
