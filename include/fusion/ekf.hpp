#pragma once

#include <array>

#include "fusion/sensors.hpp"

namespace fusion {

class Ekf {
 public:
  static constexpr int kStateSize = 6;
  enum Index { kX = 0, kY, kTheta, kV, kBa, kBg };

  using Vector = std::array<double, kStateSize>;
  using Matrix = std::array<Vector, kStateSize>;

  Ekf(const ImuSpec& imu, const OdometrySpec& odom);

  void Predict(double accel, double gyro, double dt) noexcept;

  // interval-mean speed and yaw rate
  void UpdateOdometry(double speed_meas, double yaw_rate_meas) noexcept;

  double x() const { return x_[kX]; }
  double y() const { return x_[kY]; }
  double theta() const { return x_[kTheta]; }
  double speed() const { return x_[kV]; }
  double accel_bias() const { return x_[kBa]; }
  double gyro_bias() const { return x_[kBg]; }
  double covariance(int i, int j) const { return p_[i][j]; }

 private:
  Vector x_{};
  Matrix p_{};
  double q_theta_;
  double q_v_;
  double q_ba_;
  double q_bg_;
  double r_v_;
  double r_w_;
  int samples_since_update_ = 0;
  double gyro_sum_ = 0.0;
  double accel_weighted_sum_ = 0.0;
  double dt_sum_ = 0.0;
};

}  // namespace fusion
