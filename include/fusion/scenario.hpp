#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "fusion/sensors.hpp"

namespace fusion {

struct Segment {
  double duration;  // [s]
  double speed;     // [m/s]
  double yaw_rate;  // [rad/s]
};

inline constexpr std::array<Segment, 7> kDefaultScenario{{{3.0, 1.0, 0.0},
                                                          {3.0, 1.0, 0.5},
                                                          {4.0, 1.0, 0.0},
                                                          {4.0, 1.0, -0.8},
                                                          {8.0, 0.8, 0.8},
                                                          {4.0, 1.5, 0.0},
                                                          {4.0, 0.0, 0.0}}};

struct ErrorStats {
  double pos_rmse = 0.0;
  double pos_final = 0.0;
  double speed_rmse = 0.0;
  double heading_rmse_deg = 0.0;
};

struct ScenarioResult {
  ErrorStats fused;
  ErrorStats imu_only;
  ErrorStats wheel_only;
  double accel_bias_rmse = 0.0;  // second half of the run only
  double gyro_bias_rmse = 0.0;
};

ScenarioResult RunScenario(std::span<const Segment> segments,
                           const ImuSpec& imu_spec,
                           const OdometrySpec& odom_spec,
                           std::uint64_t seed = 1);

}  // namespace fusion
