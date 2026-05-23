#include "controller/pid.hpp"
#include <cmath>
PIDController::PIDController(double kp, double ki, double kd)
    : Kp(kp)
    , Ki(ki)
    , Kd(kd)
    , prev_error(0.0)
    , integral(0.0)
    , integral_zone(INFINITY)
{
}

void PIDController::reset(double kp, double ki, double kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    prev_error = 0.0;
    integral = 0.0;
    integral_zone = INFINITY;
}

double PIDController::compute(double setpoint, double x, double dt)
{
    double error = setpoint - x;
    if (std::abs(error) < integral_zone)
        integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}
double PIDController::compute(double setpoint, double x, double xdot, double dt)
{
    double error = setpoint - x;
    if (std::abs(error) < integral_zone)
        integral += error * dt;
    prev_error = error;
    return (Kp * error) + (Ki * integral) - (Kd * xdot);
}
double PIDController::getIntegral()
{
    return integral;
}
void PIDController::setIntegralZone(double izone)
{
    integral_zone = izone;
}
