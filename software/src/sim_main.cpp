#include "pendulum/controller.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
using pendulum::Config;
using pendulum::Controller;
using pendulum::Mode;
using pendulum::SafetyInputs;
using pendulum::State;

struct Plant {
  State state;
  double cart_friction_ns_m = 0.10;
  double pivot_damping_nms = 0.005;
};

State derivative(const Plant& plant, const Config& cfg, double force_n) {
  const State& s = plant.state;
  const double M = cfg.cart_mass_kg;
  const double m = cfg.pendulum_mass_kg;
  const double l = cfg.pivot_to_com_m;
  const double J = cfg.pendulum_inertia_kg_m2 + m * l * l;
  const double sin_t = std::sin(s.theta_rad);
  const double cos_t = std::cos(s.theta_rad);
  const double a11 = M + m;
  const double a12 = m * l * cos_t;
  const double a22 = J;
  const double rhs1 = force_n - plant.cart_friction_ns_m * s.x_dot_mps +
                      m * l * s.theta_dot_radps * s.theta_dot_radps * sin_t;
  const double rhs2 = m * cfg.gravity_mps2 * l * sin_t -
                      plant.pivot_damping_nms * s.theta_dot_radps;
  const double determinant = a11 * a22 - a12 * a12;
  const double x_ddot = (rhs1 * a22 - a12 * rhs2) / determinant;
  const double theta_ddot = (a11 * rhs2 - a12 * rhs1) / determinant;
  return State{s.x_dot_mps, x_ddot, s.theta_dot_radps, theta_ddot};
}

State addScaled(const State& state, const State& derivative, double scale) {
  return State{state.x_m + scale * derivative.x_m,
               state.x_dot_mps + scale * derivative.x_dot_mps,
               state.theta_rad + scale * derivative.theta_rad,
               state.theta_dot_radps + scale * derivative.theta_dot_radps};
}

void stepRk4(Plant& plant, const Config& cfg, double force_n, double dt_s) {
  const State initial = plant.state;
  const State k1 = derivative(plant, cfg, force_n);
  plant.state = addScaled(initial, k1, 0.5 * dt_s);
  const State k2 = derivative(plant, cfg, force_n);
  plant.state = addScaled(initial, k2, 0.5 * dt_s);
  const State k3 = derivative(plant, cfg, force_n);
  plant.state = addScaled(initial, k3, dt_s);
  const State k4 = derivative(plant, cfg, force_n);
  plant.state = State{
      initial.x_m + dt_s * (k1.x_m + 2*k2.x_m + 2*k3.x_m + k4.x_m) / 6.0,
      initial.x_dot_mps + dt_s * (k1.x_dot_mps + 2*k2.x_dot_mps + 2*k3.x_dot_mps + k4.x_dot_mps) / 6.0,
      Controller::wrapAngle(initial.theta_rad + dt_s *
          (k1.theta_rad + 2*k2.theta_rad + 2*k3.theta_rad + k4.theta_rad) / 6.0),
      initial.theta_dot_radps + dt_s *
          (k1.theta_dot_radps + 2*k2.theta_dot_radps +
           2*k3.theta_dot_radps + k4.theta_dot_radps) / 6.0};
}
}  // namespace

int main(int argc, char** argv) {
  const std::string output_path = argc > 1 ? argv[1] : "simulation.csv";
  Config cfg;
  // Optional tuning arguments make repeatable parameter sweeps possible:
  // energy_gain center_kp center_kd amber red red_kp red_kd max_swing_force.
  if (argc > 2) cfg.energy_gain = std::stod(argv[2]);
  if (argc > 3) cfg.swing_center_kp = std::stod(argv[3]);
  if (argc > 4) cfg.swing_center_kd = std::stod(argv[4]);
  if (argc > 5) cfg.amber_fraction = std::stod(argv[5]);
  if (argc > 6) cfg.red_fraction = std::stod(argv[6]);
  if (argc > 7) cfg.red_zone_center_kp = std::stod(argv[7]);
  if (argc > 8) cfg.red_zone_center_kd = std::stod(argv[8]);
  if (argc > 9) cfg.max_swing_force_n = std::stod(argv[9]);
  Controller controller(cfg);
  Plant plant;
  plant.state.theta_rad = 3.14159265358979323846 - 0.002;
  SafetyInputs safety;

  if (!controller.arm(plant.state, safety)) {
    std::cerr << "Controller refused to arm\n";
    return 2;
  }
  controller.startSwingUp();

  std::ofstream csv(output_path);
  csv << "time_s,mode,x_m,x_dot_mps,theta_rad,theta_dot_radps,force_n,energy_j,faults\n";
  csv << std::setprecision(10);

  constexpr double kDuration = 20.0;
  const double dt = cfg.nominal_dt_s;
  Mode last_mode = controller.mode();
  double balanced_time = 0.0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(kDuration / dt); ++i) {
    const double time = i * dt;
    const auto output = controller.update(plant.state, safety, dt);
    stepRk4(plant, cfg, output.requested_force_n, dt);

    if (i % 5 == 0) {
      csv << time << ',' << Controller::modeName(output.mode) << ','
          << plant.state.x_m << ',' << plant.state.x_dot_mps << ','
          << plant.state.theta_rad << ',' << plant.state.theta_dot_radps << ','
          << output.requested_force_n << ',' << output.energy_j << ','
          << output.faults << '\n';
    }
    if (output.mode != last_mode) {
      std::cout << std::fixed << std::setprecision(3) << time << " s: "
                << Controller::modeName(last_mode) << " -> "
                << Controller::modeName(output.mode) << '\n';
      last_mode = output.mode;
    }
    if (output.mode == Mode::kBalance &&
        std::abs(plant.state.theta_rad) < 5.0 * 3.14159265358979323846 / 180.0) {
      balanced_time += dt;
    } else {
      balanced_time = 0.0;
    }
    if (output.mode == Mode::kFault) {
      std::cerr << "Simulation faulted with mask " << output.faults << '\n';
      return 3;
    }
    if (balanced_time >= 1.0) {
      std::cout << "PASS: balanced inside +/-5 deg for 1 s; log: "
                << output_path << '\n';
      return 0;
    }
  }
  std::cerr << "FAIL: did not achieve a 1 s stable balance; log: "
            << output_path << '\n';
  return 4;
}
