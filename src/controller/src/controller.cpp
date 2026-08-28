#include "controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

class Controller : public rclcpp::Node {
public:
    Controller()
        : Node("double_pendulum_controller")
        , theta_controller(kp_theta_, ki_theta_, kd_theta_)
        , cart_controller(kp_cart_, ki_cart_, kd_cart_)
    {
        controller_type_ = declare_parameter<std::string>("controller_type", "pid");

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state", 10,
            std::bind(&Controller::joint_callback, this, std::placeholders::_1));
        clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,
            std::bind(&Controller::clock_callback, this, std::placeholders::_1));
        force_pub_ = create_publisher<std_msgs::msg::Float64>("/force", 10);

        theta_controller.setIntegralZone(1.0);
        RCLCPP_INFO(get_logger(), "Using %s controller", controller_type_.c_str());
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 2 || msg->velocity.size() < 2) {
            return;
        }

        cart_position_ = msg->position[0];
        cart_velocity_ = msg->velocity[0];
        pendulum_angle_ = constrain_angle(msg->position[1]);
        pendulum_velocity_ = msg->velocity[1];
        has_state_ = true;
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        const double time = msg->clock.sec + msg->clock.nanosec * 1e-9;

        if (time < previous_time_) {
            theta_controller.reset(kp_theta_, ki_theta_, kd_theta_);
            theta_controller.setIntegralZone(1.0);
            cart_controller.reset(kp_cart_, ki_cart_, kd_cart_);
            applied_force_ = 0.0;
        } else if (time != previous_time_ && has_state_) {
            update_controller(time - previous_time_);
        }

        previous_time_ = time;
        std_msgs::msg::Float64 force;
        force.data = applied_force_;
        force_pub_->publish(force);
    }

    double constrain_angle(double angle) const
    {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
        return angle - M_PI;
    }

    void update_controller(double dt)
    {
        double force;
        if (controller_type_ == "lqr") {
            force = -(lqr_gain_[0] * cart_position_ + lqr_gain_[1] * cart_velocity_ + lqr_gain_[2] * pendulum_angle_ + lqr_gain_[3] * pendulum_velocity_);
        } else {
            force = theta_controller.compute(
                        0.0, pendulum_angle_, pendulum_velocity_, dt)
                + cart_controller.compute(
                    0.0, cart_position_, cart_velocity_, dt);
        }
        applied_force_ = std::clamp(force, -max_force_, max_force_);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;

    std::string controller_type_;
    bool has_state_ = false;
    double previous_time_ = 0.0;
    double cart_position_ = 0.0;
    double cart_velocity_ = 0.0;
    double pendulum_angle_ = 0.0;
    double pendulum_velocity_ = 0.0;
    double applied_force_ = 0.0;

    const double kp_theta_ = 100.0;
    const double ki_theta_ = 60.0;
    const double kd_theta_ = 12.0;
    const double kp_cart_ = -5.0;
    const double ki_cart_ = -1.0;
    const double kd_cart_ = -2.0;

    PIDController theta_controller;
    PIDController cart_controller;

    const std::array<double, 4> lqr_gain_ {
        -10.0000, -16.4138, 145.1047, 62.4718
    };
    const double max_force_ = 50.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
