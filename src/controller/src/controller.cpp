#include "controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <cstdlib>
using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
    Controller()
        : Node("double_pendulum_controller")
        , theta1_controller(kp1_, ki1_, kd1_)
        , cart_controller(kpCart_, kiCart_, kdCart_)
    {
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state", 10,
            std::bind(&Controller::joint_callback, this, std::placeholders::_1));

        clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,
            std::bind(&Controller::clock_callback, this, std::placeholders::_1));

        force_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/force", 10);

        time_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/time", 10);

        t1_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/t1", 10);

        setpoint_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/setpoint", 10);

        theta_error_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/theta_error", 10);

        integral_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/theta1_controller_integral", 10);

        theta1_controller.setIntegralZone(1.0);
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        x_cart = msg->position[0];
        v_cart = msg->velocity[0];
        double t1 = msg->position[1];
        double w1 = msg->velocity[1];
        theta_1 = constrainAngle(t1);
        omega_1 = w1;
    }

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        double sec = msg->clock.sec;
        double nsec = msg->clock.nanosec;
        time = sec + nsec * 1e-9;

        if (time < prev_time) {
            std::cout << "Time travel detected, resetting controllers" << std::endl;
            theta1_controller.reset(kp1_, ki1_, kd1_);
            theta1_controller.setIntegralZone(1.0);
            cart_controller.reset(kpCart_, kiCart_, kdCart_);
            applied_force = 0.0;
        } else if (time != prev_time) {
            update_controllers();
        }
        prev_time = time;
        publish();
    }

    double constrainAngle(double x)
    {
        x = fmod(x + M_PI, M_PI * 2.0);
        if (x < 0)
            x += M_PI * 2.0;
        return x - M_PI;
    }

    void update_controllers()
    {
        double dt = time - prev_time;
        double f_theta = theta1_controller.compute(0.0, theta_1, omega_1, dt);
        double f_cart = cart_controller.compute(0.0, x_cart, v_cart, dt);
        double f_combined = f_theta + f_cart;
        applied_force = fmax(fmin(f_combined, k_max_force), -k_max_force);
    }

    void publish()
    {
        std_msgs::msg::Float64 cmd;
        cmd.data = applied_force;
        force_pub_->publish(cmd);

        std_msgs::msg::Float64 t1;
        t1.data = theta_1;
        t1_pub_->publish(t1);

        std_msgs::msg::Float64 t;
        t.data = time;
        time_pub_->publish(t);

        std_msgs::msg::Float64 i;
        i.data = theta1_controller.getIntegral() * ki1_;
        integral_pub_->publish(i);

        std_msgs::msg::Float64 e;
        e.data = 0 - theta_1;
        theta_error_pub_->publish(e);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr theta_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr integral_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double time;
    double prev_time = 0.0;

    double x_cart;
    double v_cart;
    double theta_1;
    double omega_1;

    double kp1_ = 100.0;
    double ki1_ = 60.0;
    double kd1_ = 12.0;

    double kpCart_ = -5.0;
    double kiCart_ = -1.0;
    double kdCart_ = -2.0;

    PIDController theta1_controller;
    PIDController cart_controller;

    double applied_force = 0.0;

    double k_max_force = 50;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
