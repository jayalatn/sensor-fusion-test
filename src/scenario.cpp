#include "fusion/scenario.hpp"

#include <cmath>

#include "fusion/robot.hpp"
#include "fusion/simulated_robot.hpp"

namespace fusion {

namespace {

struct DeadReckoning {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;

  // imu only
  void Predict(const ImuSample& s, double dt) {
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta = WrapAngle(theta + s.gyro * dt);
    v += s.accel * dt;
  }

  // wheels only
  void Update(const OdometrySample& o, double dt) {
    const double theta_mid = theta + 0.5 * o.yaw_rate * dt;
    x += o.speed * std::cos(theta_mid) * dt;
    y += o.speed * std::sin(theta_mid) * dt;
    theta = WrapAngle(theta + o.yaw_rate * dt);
    v = o.speed;
  }
};

class ErrorAccumulator {
 public:
  void Add(const Robot& truth, double x, double y, double theta,
           double v) {
    const double dx = x - truth.x();
    const double dy = y - truth.y();
    const double dv = v - truth.speed();
    const double dth = WrapAngle(theta - truth.theta());
    last_pos_sq_ = dx * dx + dy * dy;
    pos_sq_ += last_pos_sq_;
    speed_sq_ += dv * dv;
    heading_sq_ += dth * dth;
    ++count_;
  }

  ErrorStats Stats() const {
    return {std::sqrt(pos_sq_ / count_), std::sqrt(last_pos_sq_),
            std::sqrt(speed_sq_ / count_),
            std::sqrt(heading_sq_ / count_) * 180.0 / kPi};
  }

 private:
  double pos_sq_ = 0.0;
  double last_pos_sq_ = 0.0;
  double speed_sq_ = 0.0;
  double heading_sq_ = 0.0;
  int count_ = 0;
};

int StepsFor(double duration) {
  return static_cast<int>(std::lround(duration / SimulatedRobot::imu_dt()));
}

}  // namespace

ScenarioResult RunScenario(std::span<const Segment> segments,
                           const ImuSpec& imu_spec,
                           const OdometrySpec& odom_spec, std::uint64_t seed) {
  SimulatedRobot bot(imu_spec, odom_spec, seed);
  const Ekf& ekf = bot.estimator();
  DeadReckoning imu_only;
  DeadReckoning wheel_only;
  ErrorAccumulator fused_err;
  ErrorAccumulator imu_err;
  ErrorAccumulator wheel_err;

  int total = 0;
  for (const Segment& seg : segments) total += StepsFor(seg.duration);
  double ba_sq = 0.0;
  double bg_sq = 0.0;
  int bias_count = 0;

  int i = 0;
  for (const Segment& seg : segments) {
    const int steps = StepsFor(seg.duration);
    for (int k = 0; k < steps; ++k, ++i) {
      const SensorTick tick = bot.Command(seg.speed, seg.yaw_rate);
      imu_only.Predict(tick.imu, SimulatedRobot::imu_dt());
      if (tick.odometry) wheel_only.Update(*tick.odometry, bot.odometry_dt());

      const Robot& truth = bot.truth();
      fused_err.Add(truth, ekf.x(), ekf.y(), ekf.theta(), ekf.speed());
      imu_err.Add(truth, imu_only.x, imu_only.y, imu_only.theta, imu_only.v);
      wheel_err.Add(truth, wheel_only.x, wheel_only.y, wheel_only.theta,
                    wheel_only.v);
      if (i >= total / 2) {  // biases settled by now
        const double ea = ekf.accel_bias() - bot.imu().accel_bias();
        const double eg = ekf.gyro_bias() - bot.imu().gyro_bias();
        ba_sq += ea * ea;
        bg_sq += eg * eg;
        ++bias_count;
      }
    }
  }

  return {fused_err.Stats(), imu_err.Stats(), wheel_err.Stats(),
          std::sqrt(ba_sq / bias_count), std::sqrt(bg_sq / bias_count)};
}

}  // namespace fusion
