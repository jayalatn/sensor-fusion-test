#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "fusion/ekf.hpp"
#include "fusion/robot.hpp"
#include "fusion/scenario.hpp"
#include "fusion/sensors.hpp"
#include "fusion/simulated_robot.hpp"

namespace fusion {
namespace {

constexpr ImuSpec kIdealImu{};
constexpr OdometrySpec kIdealOdom{.rate_hz = 50.0, .wheel_noise = 0.0};
constexpr OdometrySpec kNoisyOdom{.rate_hz = 50.0, .wheel_noise = 0.02};

constexpr std::array<Segment, 3> kStraight{
    {{2.0, 1.0, 0.0}, {2.0, 1.0, 0.0}, {2.0, 0.0, 0.0}}};
constexpr std::array<Segment, 3> kTurning{
    {{1.0, 1.0, 0.0}, {6.0, 1.0, 0.8}, {1.0, 1.0, 0.0}}};
constexpr std::array<Segment, 2> kFullLoops{
    {{1.0, 0.5, 0.0}, {20.0, 0.5, 1.5}}};

TEST(Imu, Noise) {
  Imu ideal(kIdealImu, 1);
  for (int i = 0; i < 100; ++i) {
    const ImuSample s = ideal.Sample(0.3, -0.2, 1e-3);
    EXPECT_DOUBLE_EQ(s.accel, 0.3);
    EXPECT_DOUBLE_EQ(s.gyro, -0.2);
  }
  EXPECT_DOUBLE_EQ(ideal.gyro_bias(), 0.0);

  constexpr ImuSpec kBiasOnly{.accel_bias_init = 0.3, .gyro_bias_init = 0.05};
  Imu biased(kBiasOnly, 7);
  const double ba = biased.accel_bias();
  const double bg = biased.gyro_bias();
  ASSERT_NE(bg, 0.0);
  for (int i = 0; i < 100; ++i) {
    const ImuSample s = biased.Sample(0.3, -0.2, 1e-3);
    EXPECT_NEAR(s.accel, 0.3 + ba, 1e-15);
    EXPECT_NEAR(s.gyro, -0.2 + bg, 1e-15);
  }
  EXPECT_DOUBLE_EQ(biased.gyro_bias(), bg);

  constexpr ImuSpec kWalkOnly{.gyro_bias_walk = 0.01};
  Imu walking(kWalkOnly, 7);
  for (int i = 0; i < 10000; ++i) walking.Sample(0.0, 0.0, 1e-3);
  EXPECT_GT(std::fabs(walking.gyro_bias()), 1e-6);
}

TEST(WheelOdometry, Mean) {
  WheelOdometry odo(kIdealOdom, 1);
  for (int i = 0; i < 4; ++i) odo.Accumulate(0.5 * i, 0.1 * i);
  const OdometrySample s = odo.Sample();
  EXPECT_NEAR(s.speed, 0.75, 1e-15);
  EXPECT_NEAR(s.yaw_rate, 0.15, 1e-15);
}

TEST(WheelOdometry, Covariance) {
  constexpr OdometrySpec kSpec{.rate_hz = 50.0, .wheel_noise = 0.02};
  constexpr int kCount = 20000;
  WheelOdometry odo(kSpec, 12345);
  double sum_v = 0.0;
  double sum_w = 0.0;
  double sum_vv = 0.0;
  double sum_ww = 0.0;
  double sum_vw = 0.0;
  for (int i = 0; i < kCount; ++i) {
    odo.Accumulate(1.0, 0.5);
    const OdometrySample s = odo.Sample();
    const double dv = s.speed - 1.0;
    const double dw = s.yaw_rate - 0.5;
    sum_v += dv;
    sum_w += dw;
    sum_vv += dv * dv;
    sum_ww += dw * dw;
    sum_vw += dv * dw;
  }
  const double mean_v = sum_v / kCount;
  const double mean_w = sum_w / kCount;
  const double var_v = sum_vv / kCount - mean_v * mean_v;
  const double var_w = sum_ww / kCount - mean_w * mean_w;
  const double cov = sum_vw / kCount - mean_v * mean_w;

  EXPECT_NEAR(var_v / kSpec.SpeedVariance(), 1.0, 0.1);
  EXPECT_NEAR(var_w / kSpec.YawRateVariance(), 1.0, 0.1);
  EXPECT_LT(std::fabs(cov) / std::sqrt(var_v * var_w), 0.05);
}

TEST(Fusion, IdealStraight) {
  const ScenarioResult r = RunScenario(kStraight, kIdealImu, kIdealOdom);
  EXPECT_LT(r.fused.pos_final, 1e-9);
  EXPECT_LT(r.fused.speed_rmse, 1e-9);
  EXPECT_LT(r.fused.heading_rmse_deg, 1e-12);
}

TEST(Fusion, IdealTurning) {
  const ScenarioResult r = RunScenario(kTurning, kIdealImu, kIdealOdom);
  EXPECT_LT(r.fused.heading_rmse_deg, 1e-9);
  EXPECT_LT(r.fused.speed_rmse, 1e-9);
  EXPECT_LT(r.fused.pos_final, 1e-2);  // first-order integration error
}

TEST(Fusion, BeatsBaselines) {
  double fused_speed = 0.0;
  double wheel_speed = 0.0;
  double imu_speed = 0.0;
  double fused_heading = 0.0;
  double wheel_heading = 0.0;
  for (std::uint64_t seed = 0; seed < 6; ++seed) {
    const ScenarioResult straight =
        RunScenario(kStraight, kHighQualityImu, kNoisyOdom, seed);
    fused_speed += straight.fused.speed_rmse;
    wheel_speed += straight.wheel_only.speed_rmse;
    imu_speed += straight.imu_only.speed_rmse;
    EXPECT_LT(straight.fused.pos_rmse, 0.05) << "seed " << seed;

    const ScenarioResult turning =
        RunScenario(kTurning, kHighQualityImu, kNoisyOdom, seed);
    fused_heading += turning.fused.heading_rmse_deg;
    wheel_heading += turning.wheel_only.heading_rmse_deg;
  }
  EXPECT_LT(fused_speed, wheel_speed);
  EXPECT_LT(fused_speed, imu_speed);
  EXPECT_LT(fused_heading, wheel_heading);
}

TEST(WrapAngle, Edges) {
  EXPECT_DOUBLE_EQ(WrapAngle(0.0), 0.0);
  EXPECT_NEAR(WrapAngle(kPi), -kPi, 1e-12);
  EXPECT_NEAR(WrapAngle(3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(WrapAngle(-2.0 * kPi - 0.5), -0.5, 1e-12);
}

TEST(WrapAngle, ManyTurns) {
  Ekf ekf(kIdealImu, kIdealOdom);
  for (const double gyro : {2.0, -2.0}) {
    for (int i = 0; i < 20000; ++i) {
      ekf.Predict(0.0, gyro, 1e-3);
      ASSERT_GE(ekf.theta(), -kPi);
      ASSERT_LT(ekf.theta(), kPi);
      if ((i + 1) % 20 == 0) ekf.UpdateOdometry(0.0, gyro);
    }
  }
  EXPECT_NEAR(ekf.theta(), 0.0, 1e-9);

  const ScenarioResult r = RunScenario(kFullLoops, kIdealImu, kIdealOdom);
  EXPECT_LT(r.fused.heading_rmse_deg, 1e-8);
  EXPECT_LT(r.imu_only.heading_rmse_deg, 1e-8);
}

TEST(Ekf, CovarianceDiag) {
  Ekf ekf(kHighQualityImu, kIdealOdom);
  for (int i = 0; i < 2000; ++i) {
    ekf.Predict(0.3, 0.2, 1e-3);
    if ((i + 1) % 20 == 0) ekf.UpdateOdometry(0.3 * (i + 1) * 1e-3, 0.2);
  }
  for (int i = 0; i < Ekf::kStateSize; ++i) {
    EXPECT_GE(ekf.covariance(i, i), 0.0) << "state " << i;
  }
}

TEST(Fusion, CheapImu) {
  constexpr int kSeeds = 20;
  double fused_pos = 0.0;
  double imu_pos = 0.0;
  double fused_speed = 0.0;
  double imu_speed = 0.0;
  double wheel_speed = 0.0;
  double heading = 0.0;
  double accel_bias = 0.0;
  double gyro_bias = 0.0;
  for (std::uint64_t seed = 0; seed < kSeeds; ++seed) {
    const ScenarioResult r =
        RunScenario(kDefaultScenario, kCheapImu, kNoisyOdom, seed);
    fused_pos += r.fused.pos_final / kSeeds;
    imu_pos += r.imu_only.pos_final / kSeeds;
    fused_speed += r.fused.speed_rmse / kSeeds;
    imu_speed += r.imu_only.speed_rmse / kSeeds;
    wheel_speed += r.wheel_only.speed_rmse / kSeeds;
    heading += r.fused.heading_rmse_deg / kSeeds;
    accel_bias += r.accel_bias_rmse / kSeeds;
    gyro_bias += r.gyro_bias_rmse / kSeeds;
  }
  ASSERT_GT(imu_pos, 10.0);
  EXPECT_LT(fused_pos, 0.4);
  EXPECT_LT(fused_pos, 0.05 * imu_pos);
  EXPECT_LT(fused_speed, 0.1 * imu_speed);
  EXPECT_LT(fused_speed, 0.5 * wheel_speed);
  EXPECT_LT(heading, 3.5);
  EXPECT_LT(accel_bias, 0.025);
  EXPECT_LT(gyro_bias, 0.01);
}

TEST(SimulatedRobot, OdomRate) {
  const auto make = [](double rate_hz) {
    return SimulatedRobot(kIdealImu,
                          OdometrySpec{.rate_hz = rate_hz, .wheel_noise = 0.0},
                          1);
  };
  EXPECT_NO_THROW(make(50.0));
  EXPECT_NO_THROW(make(1000.0));
  EXPECT_THROW(make(30.0), std::invalid_argument);
  EXPECT_THROW(make(3000.0), std::invalid_argument);
  EXPECT_THROW(make(0.0), std::invalid_argument);
  EXPECT_THROW(make(-50.0), std::invalid_argument);
}

}  // namespace
}  // namespace fusion
