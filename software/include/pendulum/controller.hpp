#pragma once

#include <array>
#include <cstdint>

namespace pendulum {

enum class Mode : std::uint8_t {
  kIdle,
  kSwingUp,
  kCapture,
  kBalance,
  kFault,
};

enum Fault : std::uint32_t {
  kNoFault = 0,
  kEncoderInvalid = 1u << 0,
  kHardLimit = 1u << 1,
  kRailEnvelope = 1u << 2,
  kOverCurrent = 1u << 3,
  kTimingOverrun = 1u << 4,
  kInvalidState = 1u << 5,
};

struct State {
  double x_m = 0.0;
  double x_dot_mps = 0.0;
  double theta_rad = 0.0;       // Wrapped upright error in [-pi, pi).
  double theta_dot_radps = 0.0;
};

struct SafetyInputs {
  bool encoder_valid = true;
  bool left_limit_active = false;
  bool right_limit_active = false;
  double motor_current_a = 0.0;
};

struct Config {
  // Identified plant values. Replace these nominal values with measurements.
  double cart_mass_kg = 0.50;
  double pendulum_mass_kg = 0.20;
  double pivot_to_com_m = 0.30;
  double pendulum_inertia_kg_m2 = 0.006;
  double gravity_mps2 = 9.81;

  double nominal_dt_s = 0.001;
  double max_dt_s = 0.0015;
  unsigned max_consecutive_overruns = 3;

  double usable_half_travel_m = 0.60;
  double amber_fraction = 0.26;
  double red_fraction = 0.38;
  double trip_fraction = 0.95;

  double max_force_n = 12.0;
  double max_force_slew_nps = 300.0;
  double current_limit_a = 8.0;
  double current_trip_delay_s = 0.040;

  // Swing-up. The sign is plant/motor-convention dependent and must be checked.
  double energy_gain = 12.0;
  double swing_center_kp = 4.0;
  double swing_center_kd = 2.0;
  double max_swing_force_n = 8.0;
  double startup_kick_n = 1.5;
  double startup_kick_half_period_s = 0.30;
  double startup_speed_deadband_radps = 0.08;
  double red_zone_center_kp = 50.0;
  double red_zone_center_kd = 10.0;

  // Continuous-time nominal LQR gain for state [x, x_dot, theta, theta_dot].
  // Generated for the nominal plant; redesign after system identification.
  std::array<double, 4> lqr_k{-7.07106781, -9.53549520,
                              -59.94577030, -10.91250741};

  double capture_enter_angle_rad = 12.0 * 3.14159265358979323846 / 180.0;
  double capture_enter_speed_radps = 1.5;
  double capture_enter_x_fraction = 0.60;
  double capture_dwell_s = 0.030;
  double capture_blend_s = 0.100;
  double balance_enter_angle_rad = 8.0 * 3.14159265358979323846 / 180.0;
  double balance_dwell_s = 0.100;
  double balance_exit_angle_rad = 20.0 * 3.14159265358979323846 / 180.0;
  double balance_exit_speed_radps = 2.5;
};

struct Output {
  Mode mode = Mode::kIdle;
  std::uint32_t faults = kNoFault;
  double requested_force_n = 0.0;
  double energy_j = 0.0;
  bool motor_enable = false;
};

class Controller {
 public:
  explicit Controller(Config config = Config{});

  bool arm(const State& state, const SafetyInputs& safety);
  void startSwingUp();
  void stop();
  void resetFault(const State& state, const SafetyInputs& safety);
  Output update(const State& state, const SafetyInputs& safety, double dt_s);

  Mode mode() const { return mode_; }
  std::uint32_t faults() const { return faults_; }
  const Config& config() const { return config_; }

  static double wrapAngle(double angle_rad);
  static const char* modeName(Mode mode);

 private:
  bool safeToArm(const State& state, const SafetyInputs& safety) const;
  std::uint32_t checkSafety(const State& state, const SafetyInputs& safety,
                            double dt_s);
  double pendulumEnergy(const State& state) const;
  double swingUpForce(const State& state, double energy_j, double dt_s);
  double lqrForce(const State& state) const;
  double applyRailEnvelope(double force_n, const State& state) const;
  double limitOutput(double force_n, double dt_s);
  void latchFault(std::uint32_t fault);

  Config config_;
  Mode mode_ = Mode::kIdle;
  std::uint32_t faults_ = kNoFault;
  bool armed_ = false;
  double overcurrent_time_s_ = 0.0;
  unsigned consecutive_overruns_ = 0;
  double capture_gate_time_s_ = 0.0;
  double capture_time_s_ = 0.0;
  double balance_gate_time_s_ = 0.0;
  double swing_time_s_ = 0.0;
  double capture_start_force_n_ = 0.0;
  double last_force_n_ = 0.0;
};

}  // namespace pendulum
