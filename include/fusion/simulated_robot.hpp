#pragma once

#include <cstdint>
#include <optional>

#include "fusion/ekf.hpp"
#include "fusion/robot.hpp"
#include "fusion/sensors.hpp"

namespace fusion {

constexpr double kImuRateHz = 1000.0;

struct SensorTick {
  ImuSample imu;
  std::optional<OdometrySample> odometry;
};

class SimulatedRobot {
 public:
  // throws std::invalid_argument if the odom rate doesn't divide the IMU rate
  SimulatedRobot(const ImuSpec& imu_spec, const OdometrySpec& odom_spec,
                 std::uint64_t seed);

  SensorTick Command(double speed_target, double yaw_rate_target);

  static constexpr double imu_dt() { return 1.0 / kImuRateHz; }
  double odometry_dt() const { return samples_per_odom_ * imu_dt(); }

  const Ekf& estimator() const { return ekf_; }

  // scoring only
  const Robot& truth() const { return robot_; }
  const Imu& imu() const { return imu_; }

 private:
  Robot robot_;
  Imu imu_;
  WheelOdometry odometry_;
  Ekf ekf_;
  int samples_per_odom_;
  int tick_ = 0;
};

}  // namespace fusion
