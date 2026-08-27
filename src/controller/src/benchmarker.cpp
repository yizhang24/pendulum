#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

class Benchmarker : public rclcpp::Node {
public:
  Benchmarker()
  : Node("pendulum_benchmarker")
  {
    log_name_ = declare_parameter<std::string>("log_name", "benchmark");
    log_path_ = declare_parameter<std::string>("log_path", "benchmark.csv");
    disturbance_torque_ =
      declare_parameter<double>("disturbance_torque", 2000.0);
    disturbance_duration_ =
      declare_parameter<double>("disturbance_duration", 0.1);
    arming_duration_ = declare_parameter<double>("arming_duration", 0.2);
    duration_ = declare_parameter<double>("duration", 8.0);

    log_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_) {
      throw std::runtime_error("Could not open benchmark log: " + log_path_);
    }
    log_ << "time,log_name,cart_position,cart_velocity,pendulum_angle,"
      "pendulum_angular_velocity,force,disturbance_torque\n";
    log_ << std::setprecision(12);

    torque_pub_ =
      create_publisher<std_msgs::msg::Float64>("/disturbance_torque", 10);
    force_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/force", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        controller_force_ = msg->data;
        has_controller_force_ = true;
            });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state", 10,
            std::bind(&Benchmarker::joint_callback, this, std::placeholders::_1));
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 2 || msg->velocity.size() < 2) {
      return;
    }

    const double simulation_time =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const bool transports_ready =
      torque_pub_->get_subscription_count() > 0 && has_controller_force_;

    std_msgs::msg::Float64 command;
    command.data = 0.0;
    if (!transports_ready) {
      torque_pub_->publish(command);
      return;
    }

    if (start_time_ < 0.0) {
      if (arming_start_time_ < 0.0) {
        arming_start_time_ = simulation_time;
      }
      torque_pub_->publish(command);
      if (simulation_time - arming_start_time_ < arming_duration_) {
        return;
      }

      start_time_ = simulation_time;
      RCLCPP_INFO(
                get_logger(), "Benchmark armed; starting %s trial",
                log_name_.c_str());
    }

    const double elapsed = simulation_time - start_time_;
    const double torque =
      elapsed < disturbance_duration_ ? disturbance_torque_ : 0.0;
    command.data = torque;
    torque_pub_->publish(command);

    const double angle = constrain_angle(msg->position[1]);
    log_     << elapsed << ',' << log_name_ << ',' << msg->position[0] << ','
             << msg->velocity[0] << ',' << angle << ',' << msg->velocity[1]
             << ',' << controller_force_ << ',' << torque << '\n';

    if (elapsed >= duration_) {
      command.data = 0.0;
      torque_pub_->publish(command);
      log_.close();
      RCLCPP_INFO(get_logger(), "Benchmark completed after %.3f s", elapsed);
      rclcpp::shutdown();
    }
  }

  double constrain_angle(double angle) const
  {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    return angle - M_PI;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::string log_name_;
  std::string log_path_;
  std::ofstream log_;
  bool has_controller_force_ = false;
  double arming_start_time_ = -1.0;
  double start_time_ = -1.0;
  double controller_force_ = 0.0;
  double disturbance_torque_ = 2000.0;
  double disturbance_duration_ = 0.1;
  double arming_duration_ = 0.2;
  double duration_ = 8.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Benchmarker>());
  rclcpp::shutdown();
  return 0;
}
