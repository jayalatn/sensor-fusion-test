#include "fusion/sensors.hpp"

#include <cassert>
#include <cmath>

namespace fusion {

Imu::Imu(const ImuSpec& spec, std::uint64_t seed)
    : spec_(spec),
      rng_(seed),
      accel_bias_(spec.accel_bias_init * gauss_(rng_)),
      gyro_bias_(spec.gyro_bias_init * gauss_(rng_)) {}

ImuSample Imu::Sample(double true_accel, double true_yaw_rate,
                      double dt) {
  // sqrt(dt): bias variance grows with time, not sample count
  const double sqrt_dt = std::sqrt(dt);
  accel_bias_ += spec_.accel_bias_walk * sqrt_dt * gauss_(rng_);
  gyro_bias_ += spec_.gyro_bias_walk * sqrt_dt * gauss_(rng_);
  return {true_accel + accel_bias_ + spec_.accel_noise * gauss_(rng_),
          true_yaw_rate + gyro_bias_ + spec_.gyro_noise * gauss_(rng_)};
}

OdometrySample WheelOdometry::Sample() {
  assert(count_ > 0 && "Sample() needs at least one Accumulate()");
  const double v = speed_sum_ / count_;
  const double w = yaw_sum_ / count_;
  speed_sum_ = 0.0;
  yaw_sum_ = 0.0;
  count_ = 0;
  const double half_track = 0.5 * spec_.track;
  const double left = v - w * half_track + spec_.wheel_noise * gauss_(rng_);
  const double right = v + w * half_track + spec_.wheel_noise * gauss_(rng_);
  return {0.5 * (left + right), (right - left) / spec_.track};
}

}  // namespace fusion
