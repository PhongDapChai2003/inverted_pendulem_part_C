// Hardware-adapter example. Replace the placeholder functions with the chosen
// STM32 HAL/Arduino implementation; keep the controller core unchanged.
#include "pendulum/controller.hpp"
#include "pendulum/estimator.hpp"

using pendulum::Controller;
using pendulum::EncoderMeasurement;
using pendulum::SafetyInputs;
using pendulum::State;
using pendulum::StateEstimator;

namespace board {
double cartPositionMeters();
double pendulumAngleRadians();
double motorCurrentAmps();
bool encodersValid();
bool leftLimitActive();
bool rightLimitActive();
void setBridgeEnabled(bool enabled);
void setRequestedForceNewtons(double force_n);
void log(const State& state, const pendulum::Output& output);
}  // namespace board

Controller controller;
StateEstimator estimator;

// Call once after calibration and only while the rail/current/limit state is safe.
bool armAndStart() {
  const EncoderMeasurement measurement{board::cartPositionMeters(),
                                       board::pendulumAngleRadians(),
                                       board::encodersValid()};
  estimator.reset(measurement);
  const State state{measurement.cart_position_m, 0.0,
                    Controller::wrapAngle(measurement.pendulum_angle_rad), 0.0};
  const SafetyInputs safety{board::encodersValid(), board::leftLimitActive(),
                            board::rightLimitActive(),
                            board::motorCurrentAmps()};
  if (!controller.arm(state, safety)) return false;
  controller.startSwingUp();
  return true;
}

// Call from a measured 1 kHz control interrupt. Do not use a guessed dt.
void controlTick(double measured_dt_s) {
  const EncoderMeasurement measurement{board::cartPositionMeters(),
                                       board::pendulumAngleRadians(),
                                       board::encodersValid()};
  const auto estimate = estimator.update(measurement, measured_dt_s);
  const State& state = estimate.state;
  const SafetyInputs safety{estimate.valid, board::leftLimitActive(),
                            board::rightLimitActive(),
                            board::motorCurrentAmps()};
  const auto output = controller.update(state, safety, measured_dt_s);
  board::setRequestedForceNewtons(output.requested_force_n);
  board::setBridgeEnabled(output.motor_enable);
  board::log(state, output);
}
