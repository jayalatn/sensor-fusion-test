#pragma once

#include <cmath>
#include <numbers>

namespace fusion {

constexpr double kPi = std::numbers::pi;

// to [-pi, pi)
inline double WrapAngle(double a) {
  if (a >= -kPi && a < kPi) return a;
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

struct Motion {
  double accel;     // [m/s^2]
  double yaw_rate;  // [rad/s]
  double speed;     // at the START of the step [m/s]
};

class Robot {
 public:
  Motion Step(double speed_target, double yaw_rate_target, double dt);

  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }
  double speed() const { return v_; }

 private:
  static constexpr double kTauSpeed = 0.5;  // [s]
  static constexpr double kTauYaw = 0.3;
  static constexpr double kMaxAccel = 5.0;      // [m/s^2]
  static constexpr double kMaxYawAccel = 10.0;  // [rad/s^2]
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double v_ = 0.0;
  double omega_ = 0.0;
};

}  // namespace fusion
