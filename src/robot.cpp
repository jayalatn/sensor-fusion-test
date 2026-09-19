#include "fusion/robot.hpp"

#include <algorithm>
#include <cmath>

namespace fusion {

Motion Robot::Step(double speed_target, double yaw_rate_target,
                   double dt) {
  const double accel =
      std::clamp((speed_target - v_) / kTauSpeed, -kMaxAccel, kMaxAccel);
  const double yaw_accel = std::clamp((yaw_rate_target - omega_) / kTauYaw,
                                      -kMaxYawAccel, kMaxYawAccel);

  const double speed_at_start = v_;
  omega_ += yaw_accel * dt;

  // midpoint heading, exact for a constant-rate arc
  const double theta_mid = theta_ + 0.5 * omega_ * dt;
  x_ += speed_at_start * std::cos(theta_mid) * dt;
  y_ += speed_at_start * std::sin(theta_mid) * dt;
  theta_ = WrapAngle(theta_ + omega_ * dt);

  v_ = speed_at_start + accel * dt;

  return Motion{accel, omega_, speed_at_start};
}

}  // namespace fusion
