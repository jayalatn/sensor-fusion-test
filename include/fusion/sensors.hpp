#pragma once

#include <cstdint>
#include <random>

namespace fusion {

//   b_k = b_{k-1} + bias_walk * sqrt(dt) * N(0,1)
//   z_k = truth_k + b_k + noise * N(0,1)
struct ImuSpec {
  double accel_noise = 0.0;      // [m/s^2]
  double gyro_noise = 0.0;       // [rad/s]
  double accel_bias_init = 0.0;  // [m/s^2]
  double gyro_bias_init = 0.0;   // [rad/s]
  double accel_bias_walk = 0.0;  // [m/s^2/sqrt(s)]
  double gyro_bias_walk = 0.0;   // [rad/s/sqrt(s)]
};

constexpr ImuSpec kHighQualityImu{.accel_noise = 0.02,
                                  .gyro_noise = 0.001,
                                  .accel_bias_init = 0.01,
                                  .gyro_bias_init = 0.001,
                                  .accel_bias_walk = 0.001,
                                  .gyro_bias_walk = 0.0001};
constexpr ImuSpec kCheapImu{.accel_noise = 0.2,
                            .gyro_noise = 0.02,
                            .accel_bias_init = 0.3,
                            .gyro_bias_init = 0.05,
                            .accel_bias_walk = 0.02,
                            .gyro_bias_walk = 0.005};

struct ImuSample {
  double accel;
  double gyro;
};

class Imu {
 public:
  Imu(const ImuSpec& spec, std::uint64_t seed);

  ImuSample Sample(double true_accel, double true_yaw_rate, double dt);

  double accel_bias() const { return accel_bias_; }
  double gyro_bias() const { return gyro_bias_; }

 private:
  ImuSpec spec_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
  double accel_bias_;
  double gyro_bias_;
};

struct OdometrySpec {
  double rate_hz;      // must divide the IMU rate
  double wheel_noise;  // per wheel std [m/s]
  double track = 0.4;  // [m]

  // diff drive: Var(v) = s^2/2, Var(w) = 2 s^2/track^2 from per-wheel noise s,
  // uncorrelated so R is diagonal
  constexpr double SpeedVariance() const {
    return 0.5 * wheel_noise * wheel_noise;
  }
  constexpr double YawRateVariance() const {
    return 2.0 * wheel_noise * wheel_noise / (track * track);
  }
};

struct OdometrySample {
  double speed;     // interval mean
  double yaw_rate;  // interval mean
};

class WheelOdometry {
 public:
  WheelOdometry(const OdometrySpec& spec, std::uint64_t seed)
      : spec_(spec), rng_(seed) {}

  // once per IMU step
  void Accumulate(double speed, double yaw_rate) {
    speed_sum_ += speed;
    yaw_sum_ += yaw_rate;
    ++count_;
  }

  OdometrySample Sample();

 private:
  OdometrySpec spec_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
  double speed_sum_ = 0.0;
  double yaw_sum_ = 0.0;
  int count_ = 0;
};

}  // namespace fusion
