#include "fusion/simulated_robot.hpp"

#include <cmath>
#include <stdexcept>

namespace fusion {

namespace {

int SamplesPerOdometry(double rate_hz) {
  const double ratio = kImuRateHz / rate_hz;
  // odom rate has to divide the IMU rate, otherwise the interval means in the
  // EKF don't line up
  if (rate_hz <= 0.0 || ratio < 1.0 ||
      std::fabs(ratio - std::round(ratio)) > 1e-9) {
    throw std::invalid_argument(
        "OdometrySpec::rate_hz must divide the IMU rate");
  }
  return static_cast<int>(std::lround(ratio));
}

}  // namespace

SimulatedRobot::SimulatedRobot(const ImuSpec& imu_spec,
                               const OdometrySpec& odom_spec,
                               std::uint64_t seed)
    : imu_(imu_spec, seed),
      odometry_(odom_spec, seed + 1),
      ekf_(imu_spec, odom_spec),
      samples_per_odom_(SamplesPerOdometry(odom_spec.rate_hz)) {}

SensorTick SimulatedRobot::Command(double speed_target,
                                   double yaw_rate_target) {
  const Motion motion = robot_.Step(speed_target, yaw_rate_target, imu_dt());
  SensorTick tick{imu_.Sample(motion.accel, motion.yaw_rate, imu_dt()), {}};
  ekf_.Predict(tick.imu.accel, tick.imu.gyro, imu_dt());

  odometry_.Accumulate(motion.speed, motion.yaw_rate);
  if (++tick_ % samples_per_odom_ == 0) {
    tick.odometry = odometry_.Sample();
    ekf_.UpdateOdometry(tick.odometry->speed, tick.odometry->yaw_rate);
  }
  return tick;
}

}  // namespace fusion
