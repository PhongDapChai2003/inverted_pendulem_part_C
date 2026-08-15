# Track C Inverted Pendulum

This repository contains the design plan, controller, simulation, and tests for a Track C linear cart-pole inverted pendulum.

The project starts with the pendulum hanging downward, uses energy-based control to swing it upward, and changes to LQR control to catch and balance it.

## Main features

- nonlinear energy-based swing-up;
- hysteretic `SWING_UP -> CAPTURE -> BALANCE` state machine;
- full-state LQR near upright;
- rail-envelope, encoder, hard-limit, current, and timing fault handling;
- force saturation and slew limiting;
- a nonlinear RK4 desktop simulation and unit tests.

## Run locally

```sh
make test
make simulate
```

The nominal simulation writes `output/simulation/nominal_run.csv` and succeeds only after remaining inside +/-5 degrees in `BALANCE` for one continuous second.

## Files

- `include/pendulum/controller.hpp`: configuration, state, safety, and public controller interface.
- `src/controller.cpp`: controller and safety implementation.
- `src/estimator.cpp`: wrap-safe encoder differentiation and low-pass velocity estimation.
- `src/sim_main.cpp`: full nonlinear cart-pole simulation.
- `tests/controller_tests.cpp`: deterministic safety and transition tests.
- `firmware/main_loop_example.cpp`: adapter boundary for the eventual STM32/Arduino board.
- `docs/track_c_implementation_plan.md`: complete commissioning plan.
- `docs/bill_of_materials.md`: parts list with prices and purchase links.
- `docs/Proposed_Plan_Track_C_Advanced_Swing_Up.pdf`: original project proposal.

## Values that must be replaced before hardware testing

Do not energize a real cart with the nominal configuration. Measure and update:

- cart and pendulum masses;
- pivot-to-center-of-mass distance and inertia;
- usable half-travel and mechanical clearances;
- force/PWM/current mapping, current envelope, and force slew limit;
- encoder counts, signs, velocity filters, and validity checks;
- control period and measured overrun threshold;
- LQR gain after plant identification;
- capture thresholds after simulation and low-power testing.

The software limits supplement independent normally closed limit switches, a bridge-enable cutoff, an emergency stop, a fuse, and mechanical end stops. They do not replace them.

## Angle and force conventions

- `theta = 0` upright and `theta = pi` hanging downward.
- `x = 0` at rail center.
- The controller state is `[x, x_dot, theta, theta_dot]` in SI units.
- Positive motor force must match the modeled positive cart direction.

Verify every sign at low power. If the motor polarity is reversed, energy shaping and LQR both inject energy in the wrong direction.
