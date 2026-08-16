#pragma once

#include "pendulum/controller.hpp"

namespace pendulum {

struct EstimatorConfig {
  double cart_velocity_cutoff_hz = 35.0;
  double angle_velocity_cutoff_hz = 50.0;
  double max_position_step_m = 0.050;
  double max_angle_step_rad = 0.50;
  double min_dt_s = 0.0005;
  double max_dt_s = 0.0015;
};

struct EncoderMeasurement {
  double cart_position_m = 0.0;
  double pendulum_angle_rad = 0.0;
  bool valid = true;
};

struct Estimate {
  State state;
  bool valid = false;
};

class StateEstimator {
 public:
  explicit StateEstimator(EstimatorConfig config = EstimatorConfig{});

  void reset(const EncoderMeasurement& measurement);
  Estimate update(const EncoderMeasurement& measurement, double dt_s);
  bool initialized() const { return initialized_; }

 private:
  EstimatorConfig config_;
  bool initialized_ = false;
  double last_x_m_ = 0.0;
  double last_theta_rad_ = 0.0;
  double filtered_x_dot_mps_ = 0.0;
  double filtered_theta_dot_radps_ = 0.0;
};

}  // namespace pendulum
