#include "pendulum/estimator.hpp"

#include <cmath>

namespace pendulum {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;

double filterAlpha(double cutoff_hz, double dt_s) {
  return std::exp(-kTwoPi * cutoff_hz * dt_s);
}
}  // namespace

StateEstimator::StateEstimator(EstimatorConfig config) : config_(config) {}

void StateEstimator::reset(const EncoderMeasurement& measurement) {
  last_x_m_ = measurement.cart_position_m;
  last_theta_rad_ = Controller::wrapAngle(measurement.pendulum_angle_rad);
  filtered_x_dot_mps_ = 0.0;
  filtered_theta_dot_radps_ = 0.0;
  initialized_ = measurement.valid && std::isfinite(last_x_m_) &&
                 std::isfinite(last_theta_rad_);
}

Estimate StateEstimator::update(const EncoderMeasurement& measurement,
                                double dt_s) {
  Estimate estimate;
  const double theta = Controller::wrapAngle(measurement.pendulum_angle_rad);
  const bool basic_valid = measurement.valid &&
                           std::isfinite(measurement.cart_position_m) &&
                           std::isfinite(theta) && std::isfinite(dt_s) &&
                           dt_s >= config_.min_dt_s &&
                           dt_s <= config_.max_dt_s;
  if (!initialized_) {
    reset(measurement);
    estimate.state = State{last_x_m_, 0.0, last_theta_rad_, 0.0};
    estimate.valid = false;  // Require one complete difference interval.
    return estimate;
  }
  if (!basic_valid) {
    estimate.state = State{last_x_m_, filtered_x_dot_mps_, last_theta_rad_,
                           filtered_theta_dot_radps_};
    estimate.valid = false;
    return estimate;
  }

  const double delta_x = measurement.cart_position_m - last_x_m_;
  const double delta_theta = Controller::wrapAngle(theta - last_theta_rad_);
  if (std::abs(delta_x) > config_.max_position_step_m ||
      std::abs(delta_theta) > config_.max_angle_step_rad) {
    estimate.state = State{last_x_m_, filtered_x_dot_mps_, last_theta_rad_,
                           filtered_theta_dot_radps_};
    estimate.valid = false;
    return estimate;
  }

  const double raw_x_dot = delta_x / dt_s;
  const double raw_theta_dot = delta_theta / dt_s;
  const double x_alpha = filterAlpha(config_.cart_velocity_cutoff_hz, dt_s);
  const double theta_alpha =
      filterAlpha(config_.angle_velocity_cutoff_hz, dt_s);
  filtered_x_dot_mps_ =
      x_alpha * filtered_x_dot_mps_ + (1.0 - x_alpha) * raw_x_dot;
  filtered_theta_dot_radps_ = theta_alpha * filtered_theta_dot_radps_ +
                              (1.0 - theta_alpha) * raw_theta_dot;
  last_x_m_ = measurement.cart_position_m;
  last_theta_rad_ = theta;
  estimate.state = State{last_x_m_, filtered_x_dot_mps_, last_theta_rad_,
                         filtered_theta_dot_radps_};
  estimate.valid = true;
  return estimate;
}

}  // namespace pendulum
