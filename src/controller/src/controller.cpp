#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <cstdlib>
using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
    Controller()
        : Node("double_pendulum_controller")
    {
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
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Extract angles, velocities, etc.
        // e.g., theta1 = msg->position[0], theta2 = msg->position[1]
        x_cart = msg->position[0];
        v_cart = msg->velocity[0];
        double t1 = msg->position[1];
        double w1 = msg->velocity[1];
        theta_1 = constrainAngle(t1);
        omega_1 = std::abs(w1);
    }

    double constrainAngle(double x)
    {
        x = fmod(x + M_PI, M_PI * 2.0);
        if (x < 0)
            x += M_PI * 2.0;
        return x - M_PI;
    }

    void update()
    {
        double setpoint = 0;
        double p_err = setpoint - theta_1;
        double prop = (kp1_ * p_err);
        double deriv = (kd1_ * -omega_1);

        double f_theta = prop + deriv;

        double f_cart = 0 * (-kpx_ * x_cart + -kdx_ * v_cart);

        double f
            = fmax(fmin(f_theta + f_cart, k_max_force), -k_max_force);

        std_msgs::msg::Float64 cmd;
        cmd.data = f;
        force_pub_->publish(cmd);

        std_msgs::msg::Float64 t1;
        t1.data = theta_1;
        t1_pub_->publish(t1);

        std_msgs::msg::Float64 sp;
        sp.data = setpoint;
        setpoint_pub_->publish(sp);
    }

    double x_cart;
    double v_cart;
    double theta_1;
    double omega_1;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr setpoint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double kp1_ = 80.0;
    double kd1_ = 12.0;

    double kpx_ = 1.0;
    double kdx_ = 5.0;

    double k_centering_factor = 1.0 * (M_PI / 180.0);
    double k_max_force = 15;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
