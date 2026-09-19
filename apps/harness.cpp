#include <array>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

#include "fusion/ekf.hpp"
#include "fusion/scenario.hpp"

namespace {

constexpr fusion::OdometrySpec kOdom{.rate_hz = 50.0, .wheel_noise = 0.02};
constexpr std::uint64_t kSeed = 1;

struct ImuPreset {
  const char* name;
  fusion::ImuSpec spec;
};

constexpr std::array<ImuPreset, 2> kPresets{{
    {"high-quality", fusion::kHighQualityImu},
    {"cheap", fusion::kCheapImu},
}};

void PrintRow(const char* name, const fusion::ErrorStats& e) {
  std::printf("  %-12s%14.3f%15.3f%18.3f%20.2f\n", name, e.pos_rmse,
              e.pos_final, e.speed_rmse, e.heading_rmse_deg);
}

void Benchmark() {
  constexpr int kSamples = 1'000'000;
  constexpr double kDt = 1e-3;
  constexpr int kPerUpdate = 20;

  std::vector<double> accel(kSamples);
  std::vector<double> gyro(kSamples);
  std::mt19937_64 rng(0);
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (int i = 0; i < kSamples; ++i) {
    accel[i] = 0.5 * gauss(rng);
    gyro[i] = 0.3 * gauss(rng);
  }

  fusion::Ekf ekf(fusion::kHighQualityImu, kOdom);
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kSamples; ++i) {
    ekf.Predict(accel[i], gyro[i], kDt);
    if ((i + 1) % kPerUpdate == 0) ekf.UpdateOdometry(1.0, 0.1);
  }
  const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - start;
  const double ns = 1e9 * elapsed.count() / kSamples;

  std::printf("Estimator: %.1f ns per IMU sample including the 50 Hz update, ",
              ns);
  std::printf("%.4f%% of the 1 ms budget\n", 100.0 * ns * 1e-9 / kDt);
  // so the loop isn't optimised away
  std::printf("  (checksum %.6g)\n", ekf.x() + ekf.y());
}

}  // namespace

int main() {
  for (const ImuPreset& preset : kPresets) {
    const fusion::ScenarioResult r = fusion::RunScenario(
        fusion::kDefaultScenario, preset.spec, kOdom, kSeed);
    std::printf("IMU: %s   odometry %g Hz, wheel noise %g m/s\n", preset.name,
                kOdom.rate_hz, kOdom.wheel_noise);
    std::printf("  %-12s%14s%15s%18s%20s\n", "estimator", "pos RMSE [m]",
                "pos final [m]", "speed RMSE [m/s]", "heading RMSE [deg]");
    PrintRow("fused", r.fused);
    PrintRow("imu only", r.imu_only);
    PrintRow("wheel only", r.wheel_only);
    std::printf("\n");
  }
  Benchmark();
  return 0;
}
