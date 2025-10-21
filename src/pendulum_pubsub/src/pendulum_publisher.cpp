#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "messages/msg/pendulum_state.hpp"


class PendulumPublisher : public rclcpp::Node {
  public:
  PendulumPublisher(): Node("pendulum_publisher"), time_(0.0) {
    publisher_ = this->create_publisher<messages::msg::PendulumState>("pendulum_angle", 10);
    this->declare_parameter("frequency", 30.0);
    double freq = this->get_parameter("frequency").as_double();
    period_ = 1.0 / freq;
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PendulumPublisher::on_param_change, this, std::placeholders::_1));
    create_timer();
  }

  private:
  double time_;
  double period_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<messages::msg::PendulumState>::SharedPtr publisher_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  void create_timer() {
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_), std::bind(&PendulumPublisher::timer_callback, this));

  }

  void timer_callback() {
    time_ += 0.02;
    double angle1 = std::sin(time_);
    double angle2 = std::cos(time_);
    auto msg = messages::msg::PendulumState();
    msg.set__theta1(angle1);
    msg.set__theta2(angle2);
    publisher_->publish(msg);
    std::stringstream s;
    s << "Pendulum state: " << " theta1=" << msg.theta1 << ", theta2=" << msg.theta2 <<" ";
    RCLCPP_INFO(this->get_logger(), "%s", s.str().c_str());
  }

  rcl_interfaces::msg::SetParametersResult on_param_change(std::vector<rclcpp::Parameter> params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for(const auto &param : params) {
      if(param.get_name() == "frequency"){
        const double new_freq = param.as_double();
        if(new_freq <= 0.0) {
          result.set__successful(false);
          result.set__reason("Frequency must be positive");
          return result;
        }
        period_ = 1.0 / new_freq;
        create_timer();
        RCLCPP_INFO(this->get_logger(), "Updated frequency to %.1f Hz", 1.0 / period_);
      }
    }
    return result;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PendulumPublisher>());
  rclcpp::shutdown();
  return 0;
}
