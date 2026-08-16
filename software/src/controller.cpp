#include "pendulum/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pendulum {
namespace {
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

bool finiteState(const State& state) {
  return std::isfinite(state.x_m) && std::isfinite(state.x_dot_mps) &&
         std::isfinite(state.theta_rad) &&
         std::isfinite(state.theta_dot_radps);
}
}  // namespace

Controller::Controller(Config config) : config_(config) {}

double Controller::wrapAngle(double angle_rad) {
  double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (wrapped < 0.0) wrapped += 2.0 * kPi;
  return wrapped - kPi;
}

const char* Controller::modeName(Mode mode) {
  switch (mode) {
    case Mode::kIdle: return "IDLE";
    case Mode::kSwingUp: return "SWING_UP";
    case Mode::kCapture: return "CAPTURE";
    case Mode::kBalance: return "BALANCE";
    case Mode::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

bool Controller::safeToArm(const State& state,
                           const SafetyInputs& safety) const {
  const double trip = config_.trip_fraction * config_.usable_half_travel_m;
  return finiteState(state) && safety.encoder_valid &&
         !safety.left_limit_active && !safety.right_limit_active &&
         std::abs(state.x_m) < trip &&
         std::abs(safety.motor_current_a) < config_.current_limit_a;
}

bool Controller::arm(const State& state, const SafetyInputs& safety) {
  if (mode_ == Mode::kFault || !safeToArm(state, safety)) return false;
  armed_ = true;
  mode_ = Mode::kIdle;
  last_force_n_ = 0.0;
  return true;
}

void Controller::startSwingUp() {
  if (!armed_ || mode_ == Mode::kFault) return;
  mode_ = Mode::kSwingUp;
  capture_gate_time_s_ = 0.0;
  swing_time_s_ = 0.0;
}

void Controller::stop() {
  armed_ = false;
  if (mode_ != Mode::kFault) mode_ = Mode::kIdle;
  last_force_n_ = 0.0;
}

void Controller::resetFault(const State& state,
                            const SafetyInputs& safety) {
  if (!safeToArm(state, safety)) return;
  faults_ = kNoFault;
  mode_ = Mode::kIdle;
  armed_ = false;
  overcurrent_time_s_ = 0.0;
  consecutive_overruns_ = 0;
  last_force_n_ = 0.0;
}

void Controller::latchFault(std::uint32_t fault) {
  faults_ |= fault;
  mode_ = Mode::kFault;
  armed_ = false;
  last_force_n_ = 0.0;
}

std::uint32_t Controller::checkSafety(const State& state,
                                      const SafetyInputs& safety,
                                      double dt_s) {
  std::uint32_t detected = kNoFault;
  if (!finiteState(state)) detected |= kInvalidState;
  if (!safety.encoder_valid) detected |= kEncoderInvalid;
  if (safety.left_limit_active || safety.right_limit_active) {
    detected |= kHardLimit;
  }
  if (std::isfinite(state.x_m) &&
      std::abs(state.x_m) >=
          config_.trip_fraction * config_.usable_half_travel_m) {
    detected |= kRailEnvelope;
  }

  if (std::abs(safety.motor_current_a) > config_.current_limit_a) {
    overcurrent_time_s_ += std::max(0.0, dt_s);
  } else {
    overcurrent_time_s_ = 0.0;
  }
  if (overcurrent_time_s_ >= config_.current_trip_delay_s) {
    detected |= kOverCurrent;
  }

  if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > config_.max_dt_s) {
    ++consecutive_overruns_;
  } else {
    consecutive_overruns_ = 0;
  }
  if (consecutive_overruns_ >= config_.max_consecutive_overruns) {
    detected |= kTimingOverrun;
  }
  return detected;
}

double Controller::pendulumEnergy(const State& state) const {
  const double m = config_.pendulum_mass_kg;
  const double l = config_.pivot_to_com_m;
  const double inertia_about_pivot =
      config_.pendulum_inertia_kg_m2 + m * l * l;
  return 0.5 * inertia_about_pivot * state.theta_dot_radps *
             state.theta_dot_radps +
         m * config_.gravity_mps2 * l * (std::cos(state.theta_rad) - 1.0);
}

double Controller::swingUpForce(const State& state, double energy_j,
                                double dt_s) {
  swing_time_s_ += dt_s;
  // Energy shaping. The phase term changes sign at the points where cart
  // acceleration must reverse to add (or remove) pendulum energy.
  const double phase = state.theta_dot_radps * std::cos(state.theta_rad);
  double energy_force = config_.energy_gain * energy_j * phase;

  // The exact downward equilibrium cannot self-start under pure energy shaping.
  if (std::abs(state.theta_dot_radps) <
          config_.startup_speed_deadband_radps &&
      energy_j < -0.25 * config_.pendulum_mass_kg *
                     config_.gravity_mps2 * config_.pivot_to_com_m) {
    const auto half_cycles = static_cast<long>(
        swing_time_s_ / config_.startup_kick_half_period_s);
    const double direction = (half_cycles % 2 == 0) ? 1.0 : -1.0;
    energy_force += direction * config_.startup_kick_n;
  }

  const double center_force = -config_.swing_center_kp * state.x_m -
                              config_.swing_center_kd * state.x_dot_mps;
  return clamp(energy_force + center_force, -config_.max_swing_force_n,
               config_.max_swing_force_n);
}

double Controller::lqrForce(const State& state) const {
  const auto& k = config_.lqr_k;
  return -(k[0] * state.x_m + k[1] * state.x_dot_mps +
           k[2] * state.theta_rad + k[3] * state.theta_dot_radps);
}

double Controller::applyRailEnvelope(double force_n,
                                     const State& state) const {
  const double x_max = config_.usable_half_travel_m;
  const double distance = std::abs(state.x_m);
  const double outward_sign = state.x_m >= 0.0 ? 1.0 : -1.0;
  const double amber = config_.amber_fraction * x_max;
  const double red = config_.red_fraction * x_max;

  if (distance <= amber) return force_n;
  if (distance >= red) {
    // In the red zone, override outward requests with active bounded braking.
    const double braking = -config_.red_zone_center_kp * state.x_m -
                           config_.red_zone_center_kd * state.x_dot_mps;
    return outward_sign * force_n > 0.0 ? braking : force_n;
  }

  if (outward_sign * force_n <= 0.0) return force_n;
  const double scale = 1.0 - (distance - amber) / (red - amber);
  return force_n * clamp(scale, 0.0, 1.0);
}

double Controller::limitOutput(double force_n, double dt_s) {
  force_n = clamp(force_n, -config_.max_force_n, config_.max_force_n);
  const double max_step = config_.max_force_slew_nps * std::max(0.0, dt_s);
  force_n = clamp(force_n, last_force_n_ - max_step,
                  last_force_n_ + max_step);
  last_force_n_ = force_n;
  return force_n;
}

Output Controller::update(const State& input_state,
                          const SafetyInputs& safety, double dt_s) {
  State state = input_state;
  state.theta_rad = wrapAngle(state.theta_rad);
  const std::uint32_t detected = checkSafety(state, safety, dt_s);
  if (detected != kNoFault) latchFault(detected);

  Output output;
  output.mode = mode_;
  output.faults = faults_;
  output.energy_j = finiteState(state) ? pendulumEnergy(state) : 0.0;

  if (!armed_ || mode_ == Mode::kIdle || mode_ == Mode::kFault) {
    output.requested_force_n = 0.0;
    output.motor_enable = false;
    return output;
  }

  double requested_force = 0.0;
  if (mode_ == Mode::kSwingUp) {
    requested_force = swingUpForce(state, output.energy_j, dt_s);
    const bool in_capture_gate =
        std::abs(state.theta_rad) < config_.capture_enter_angle_rad &&
        std::abs(state.theta_dot_radps) < config_.capture_enter_speed_radps &&
        std::abs(state.x_m) < config_.capture_enter_x_fraction *
                                  config_.usable_half_travel_m;
    capture_gate_time_s_ =
        in_capture_gate ? capture_gate_time_s_ + dt_s : 0.0;
    if (capture_gate_time_s_ >= config_.capture_dwell_s) {
      mode_ = Mode::kCapture;
      capture_time_s_ = 0.0;
      balance_gate_time_s_ = 0.0;
      capture_start_force_n_ = requested_force;
    }
  } else if (mode_ == Mode::kCapture) {
    capture_time_s_ += dt_s;
    const double blend = clamp(capture_time_s_ / config_.capture_blend_s,
                               0.0, 1.0);
    const double balance_force = lqrForce(state);
    requested_force = (1.0 - blend) * capture_start_force_n_ +
                      blend * balance_force;

    const bool stable_gate =
        std::abs(state.theta_rad) < config_.balance_enter_angle_rad;
    balance_gate_time_s_ =
        stable_gate ? balance_gate_time_s_ + dt_s : 0.0;
    if (balance_gate_time_s_ >= config_.balance_dwell_s) {
      mode_ = Mode::kBalance;
    } else if (std::abs(state.theta_rad) > config_.balance_exit_angle_rad ||
               std::abs(state.theta_dot_radps) >
                   config_.balance_exit_speed_radps) {
      mode_ = Mode::kSwingUp;
      capture_gate_time_s_ = 0.0;
      swing_time_s_ = 0.0;
    }
  } else if (mode_ == Mode::kBalance) {
    requested_force = lqrForce(state);
    if (std::abs(state.theta_rad) > config_.balance_exit_angle_rad ||
        std::abs(state.theta_dot_radps) >
            config_.balance_exit_speed_radps) {
      mode_ = Mode::kSwingUp;
      capture_gate_time_s_ = 0.0;
      swing_time_s_ = 0.0;
    }
  }

  requested_force = applyRailEnvelope(requested_force, state);
  output.requested_force_n = limitOutput(requested_force, dt_s);
  output.mode = mode_;
  output.faults = faults_;
  output.motor_enable = true;
  return output;
}

}  // namespace pendulum
