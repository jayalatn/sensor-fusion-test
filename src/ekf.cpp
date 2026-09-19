#include "fusion/ekf.hpp"

#include <cmath>

#include "fusion/robot.hpp"  // WrapAngle

namespace fusion {

namespace {

constexpr int kN = Ekf::kStateSize;

// keeps S invertible with ideal sensors
constexpr double FloorVariance(double variance) {
  return variance > 1e-12 ? variance : 1e-12;
}

}  // namespace

Ekf::Ekf(const ImuSpec& imu, const OdometrySpec& odom)
    : q_theta_(imu.gyro_noise * imu.gyro_noise),
      q_v_(imu.accel_noise * imu.accel_noise),
      q_ba_(FloorVariance(imu.accel_bias_walk * imu.accel_bias_walk)),
      q_bg_(FloorVariance(imu.gyro_bias_walk * imu.gyro_bias_walk)),
      r_v_(FloorVariance(odom.SpeedVariance())),
      r_w_(FloorVariance(odom.YawRateVariance())) {
  p_[kBa][kBa] = FloorVariance(imu.accel_bias_init * imu.accel_bias_init);
  p_[kBg][kBg] = FloorVariance(imu.gyro_bias_init * imu.gyro_bias_init);
}

void Ekf::Predict(double accel, double gyro, double dt) noexcept {
  const double v = x_[kV];
  const double c = std::cos(x_[kTheta]);
  const double s = std::sin(x_[kTheta]);

  x_[kX] += v * c * dt;
  x_[kY] += v * s * dt;
  x_[kTheta] = WrapAngle(x_[kTheta] + (gyro - x_[kBg]) * dt);
  x_[kV] = v + (accel - x_[kBa]) * dt;

  ++samples_since_update_;
  gyro_sum_ += gyro;
  accel_weighted_sum_ += samples_since_update_ * accel * dt;
  dt_sum_ += dt;

  Matrix f{};
  for (int i = 0; i < kN; ++i) f[i][i] = 1.0;
  f[kX][kTheta] = -v * s * dt;
  f[kX][kV] = c * dt;
  f[kY][kTheta] = v * c * dt;
  f[kY][kV] = s * dt;
  f[kTheta][kBg] = -dt;
  f[kV][kBa] = -dt;

  Matrix fp;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double sum = 0.0;
      for (int k = 0; k < kN; ++k) sum += f[i][k] * p_[k][j];
      fp[i][j] = sum;
    }
  }
  // P = FP F^T, upper triangle mirrored so P stays symmetric
  for (int i = 0; i < kN; ++i) {
    for (int j = i; j < kN; ++j) {
      double sum = 0.0;
      for (int k = 0; k < kN; ++k) sum += fp[i][k] * f[j][k];
      p_[i][j] = p_[j][i] = sum;
    }
  }
  p_[kTheta][kTheta] += q_theta_ * dt * dt;
  p_[kV][kV] += q_v_ * dt * dt;
  p_[kBa][kBa] += q_ba_ * dt;
  p_[kBg][kBg] += q_bg_ * dt;
}

void Ekf::UpdateOdometry(double speed_meas, double yaw_rate_meas) noexcept {
  const int n = samples_since_update_;
  if (n == 0) return;
  const double c = dt_sum_ * (n + 1) / (2.0 * n);  // d v_mean / d b_a
  const double y0 =
      speed_meas - (x_[kV] - accel_weighted_sum_ / n + c * x_[kBa]);
  const double y1 = yaw_rate_meas - (gyro_sum_ / n - x_[kBg]);
  const double r_w = r_w_ + q_theta_ / n;
  samples_since_update_ = 0;
  gyro_sum_ = 0.0;
  accel_weighted_sum_ = 0.0;
  dt_sum_ = 0.0;

  // rows of HP (= columns of PH^T, P is symmetric)
  Vector hp0;
  Vector hp1;
  for (int j = 0; j < kN; ++j) {
    hp0[j] = p_[kV][j] + c * p_[kBa][j];
    hp1[j] = -p_[kBg][j];
  }
  const double s00 = hp0[kV] + c * hp0[kBa] + r_v_;
  const double s01 = -hp0[kBg];
  const double s11 = -hp1[kBg] + r_w;
  const double det = s00 * s11 - s01 * s01;
  const double i00 = s11 / det;
  const double i01 = -s01 / det;
  const double i11 = s00 / det;

  Vector k0;
  Vector k1;
  for (int i = 0; i < kN; ++i) {
    k0[i] = hp0[i] * i00 + hp1[i] * i01;
    k1[i] = hp0[i] * i01 + hp1[i] * i11;
    x_[i] += k0[i] * y0 + k1[i] * y1;
  }
  x_[kTheta] = WrapAngle(x_[kTheta]);
  for (int i = 0; i < kN; ++i) {
    for (int j = i; j < kN; ++j) {
      p_[i][j] = p_[j][i] = p_[i][j] - k0[i] * hp0[j] - k1[i] * hp1[j];
    }
  }
}

}  // namespace fusion
