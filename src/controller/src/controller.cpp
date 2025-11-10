#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <cstdlib>
using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
  public:
    Controller() : Node("double_pendulum_controller") {
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          "/joint_state", 10,
          std::bind(&Controller::joint_callback, this, std::placeholders::_1));

      force_pub_ = create_publisher<std_msgs::msg::Float64>(
          "/force", 10);

      t1_pub_ = create_publisher<std_msgs::msg::Float64>(
          "/t1", 10);

      setpoint_pub_ = create_publisher<std_msgs::msg::Float64>(
          "/setpoint", 10);


      control_timer_ = create_wall_timer(
          20ms, std::bind(&Controller::update, this));
    }

  private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
      // Extract angles, velocities, etc.
      // e.g., theta1 = msg->position[0], theta2 = msg->position[1]
      x_c = msg->position[0];
      v_c = msg->velocity[0];
      double t1 = msg->position[1] + k_starting_theta;
      double w1 = msg->velocity[1];
      theta_1 = constrainAngle(t1);
      omega_1 = std::abs(w1);
    }

    double constrainAngle(double x){
        x = fmod(x + M_PI,M_PI * 2.0);
        if (x < 0)
            x += M_PI * 2.0;
        return x - M_PI;
    }

    void update() {
      double setpoint = k_centering_factor * x_c;
      double p_err = setpoint - theta_1;
      double prop = (kp1_ * p_err);
      double deriv = std::copysign(std::min(std::abs(kd1_ * omega_1), std::abs(prop)), prop * -1);

      std_msgs::msg::Float64 cmd;
      cmd.data = prop + deriv;
      force_pub_->publish(cmd);

      std_msgs::msg::Float64 t1;
      t1.data = theta_1;
      t1_pub_ -> publish(t1);

      std_msgs::msg::Float64 sp;
      sp.data = setpoint;
      setpoint_pub_ -> publish(sp);
    }

    double x_c;
    double v_c;
    double theta_1;
    double omega_1;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double kp1_ = 200.0;
    double kd1_ = 30.0;

    double k_centering_factor = 1.0 * (M_PI / 180.0);

    double k_starting_theta = 25.0 * (M_PI / 180.0);
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};
