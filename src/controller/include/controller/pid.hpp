#pragma once
class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double setpoint, double measured, double dt);
    double compute(double setpoint, double measured_x, double measured_xdot, double dt);
    void reset(double kp, double ki, double kd);
    double getIntegral();
    void setIntegralZone(double izone);

private:
    double Kp,
        Ki,
        Kd;
    double prev_error;
    double integral;
    double integral_zone;
};
