#include "double_pendulum/pendulum.h"
#include <cmath>
#include <limits>

Pendulum operator+(const Pendulum& a, const Pendulum& b) {
  return {a.theta1 + b.theta1, a.theta2 + b.theta2,
        a.omega1 + b.omega1, a.omega2 + b.omega2};
}
Pendulum operator*(const Pendulum& a, const double scalar) {
  return {a.theta1 * scalar, a.theta2 * scalar,
        a.omega1 * scalar, a.omega2 * scalar};
}
Pendulum operator*(const double scalar, const Pendulum& a) {
  return a * scalar;
}

Pendulum dsdt(const Pendulum &s, double l1, double l2, double m1, double m2,
              double g) {
  Pendulum ret = Pendulum();
  const double th1 = s.theta1;
  const double th2 = s.theta2;
  const double w1 = s.omega1;
  const double w2 = s.omega2;

  const double delta = th1 - th2;

  const double denom_component = 2.0 * m1 + m2 - m2 * cos(2 * delta);
  const double denom_1 = l1 * denom_component;
  const double denom_2 = l2 * denom_component;

  const double eps = std::numeric_limits<double>::epsilon();
  const double safe_denom_1 = std::fabs(denom_1) < eps ? eps : denom_1;
  const double safe_denom_2 = std::fabs(denom_2) < eps ? eps : denom_2;

  ret.omega1 = (-g * (2.0 * m1 + m2) * sin(th1) - m2 * g * sin(delta - th2) -
                2.0 * sin(delta) * m2 * (w2 * w2 * l2 + w1 * w1 * l1 * cos(delta))) *
                (1.0 / safe_denom_1);
  ret.omega2 = (2.0 * sin(delta) *
                (w2 * w2 * l1 * (m1 + m2) + g * (m1 + m2) * cos(th1) +
                 w2 * w2 * l2 * m2 * cos(delta))) *
               (1.0 / safe_denom_2);
  ret.theta1 = w1;
  ret.theta2 = w2;
  return ret;
}

Pendulum rk4_step(const Pendulum& s, double dt, double l1, double l2, double m1, double m2,
              double g) {
  auto f = [&](const Pendulum& x) {
    return dsdt(x, l1, l2, m1, m2, g);
  };

  Pendulum a = f(s);
  Pendulum b = f(s + (dt / 2.0) * a);
  Pendulum c = f(s + (dt / 2.0) * b);
  Pendulum d = f(s + dt * c);
  return s + (dt / 6.0) * (a + 2 * b + 2 * c + d);
}
