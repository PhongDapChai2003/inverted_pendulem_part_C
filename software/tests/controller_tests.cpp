#include "pendulum/controller.hpp"
#include "pendulum/estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
int failures = 0;

void expect(bool condition, const char* message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}
}  // namespace

int main() {
  using namespace pendulum;
  constexpr double pi = 3.14159265358979323846;

  expect(std::abs(Controller::wrapAngle(2.0 * pi)) < 1e-12,
         "2*pi should wrap to zero");
  expect(std::abs(Controller::wrapAngle(3.0 * pi) + pi) < 1e-12,
         "3*pi should wrap to -pi");

  Config cfg;
  SafetyInputs safe;
  State down{0.0, 0.0, pi, 0.0};
  Controller controller(cfg);
  expect(controller.arm(down, safe), "safe state should arm");
  controller.startSwingUp();
  auto output = controller.update(down, safe, cfg.nominal_dt_s);
  expect(output.motor_enable, "swing-up should enable motor");
  expect(std::abs(output.requested_force_n) <= cfg.max_force_n,
         "output must obey force saturation");
  expect(std::abs(output.requested_force_n) <=
             cfg.max_force_slew_nps * cfg.nominal_dt_s + 1e-12,
         "first output must obey slew limit");

  SafetyInputs invalid = safe;
  invalid.encoder_valid = false;
  output = controller.update(down, invalid, cfg.nominal_dt_s);
  expect(output.mode == Mode::kFault, "invalid encoder should fault");
  expect((output.faults & kEncoderInvalid) != 0,
         "encoder fault bit should latch");
  expect(!output.motor_enable && output.requested_force_n == 0.0,
         "fault must disable motor and force zero");

  Controller rail_controller(cfg);
  State outside = down;
  outside.x_m = cfg.trip_fraction * cfg.usable_half_travel_m;
  expect(!rail_controller.arm(outside, safe),
         "controller must refuse to arm outside rail envelope");

  Controller capture_controller(cfg);
  State upright{0.0, 0.0, 0.01, 0.0};
  expect(capture_controller.arm(upright, safe), "upright should arm");
  capture_controller.startSwingUp();
  const int capture_steps =
      static_cast<int>(std::ceil(cfg.capture_dwell_s / cfg.nominal_dt_s)) + 2;
  for (int i = 0; i < capture_steps; ++i) {
    output = capture_controller.update(upright, safe, cfg.nominal_dt_s);
  }
  expect(output.mode == Mode::kCapture || output.mode == Mode::kBalance,
         "capture requires dwell and then must enter capture");
  const int balance_steps =
      static_cast<int>(std::ceil((cfg.capture_blend_s + cfg.balance_dwell_s) /
                                 cfg.nominal_dt_s)) + 5;
  for (int i = 0; i < balance_steps; ++i) {
    output = capture_controller.update(upright, safe, cfg.nominal_dt_s);
  }
  expect(output.mode == Mode::kBalance,
         "stable upright state should reach balance mode");

  Controller timing_controller(cfg);
  expect(timing_controller.arm(down, safe), "timing test should arm");
  timing_controller.startSwingUp();
  for (unsigned i = 0; i < cfg.max_consecutive_overruns; ++i) {
    output = timing_controller.update(down, safe, cfg.max_dt_s * 2.0);
  }
  expect(output.mode == Mode::kFault, "repeated timing overrun should fault");
  expect((output.faults & kTimingOverrun) != 0,
         "timing fault bit should latch");

  StateEstimator estimator;
  EncoderMeasurement measurement{0.0, pi - 0.001, true};
  estimator.reset(measurement);
  Estimate estimate;
  for (int i = 1; i <= 100; ++i) {
    measurement.cart_position_m = i * 0.001;  // 1 m/s at 1 kHz.
    measurement.pendulum_angle_rad =
        Controller::wrapAngle(pi - 0.001 + i * 0.002);
    estimate = estimator.update(measurement, 0.001);
    expect(estimate.valid, "valid encoder motion should estimate");
  }
  expect(std::abs(estimate.state.x_dot_mps - 1.0) < 0.02,
         "cart velocity filter should converge to 1 m/s");
  expect(std::abs(estimate.state.theta_dot_radps - 2.0) < 0.03,
         "angle velocity must remain continuous across wraparound");

  measurement.cart_position_m += 0.20;
  estimate = estimator.update(measurement, 0.001);
  expect(!estimate.valid, "implausible encoder jump should be rejected");

  if (failures == 0) {
    std::cout << "All controller tests passed\n";
    return EXIT_SUCCESS;
  }
  std::cerr << failures << " controller test(s) failed\n";
  return EXIT_FAILURE;
}
