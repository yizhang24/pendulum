#ifndef PENDULUM
#define PENDULUM

struct Pendulum {
  double theta1;
  double theta2;
  double omega1;
  double omega2;
  friend Pendulum operator+(const Pendulum& a, const Pendulum& b);
  friend Pendulum operator*(const Pendulum& a, const double scalar);
  friend Pendulum operator*(const double scalar, const Pendulum& a);
};

Pendulum dsdt(const Pendulum &s, double l1, double l2, double m1, double m2,
              double g);

Pendulum rk4_step(const Pendulum& s, double dt, double l1, double l2, double m1, double m2,
              double g);

#endif
