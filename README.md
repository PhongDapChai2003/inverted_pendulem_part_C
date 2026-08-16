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
make -C software test
make -C software simulate
```

The nominal simulation writes `software/output/simulation/nominal_run.csv` and succeeds only after remaining inside +/-5 degrees in `BALANCE` for one continuous second.

## Project layout

- `software/`: controller code, Pico firmware, simulation, and tests.
- `docs/`: project proposal, implementation plan, bill of materials, and document tool.
- `platformio.ini`: Raspberry Pi Pico 2 build configuration.

The folders inside `software/` follow the usual C++ layout: headers are in `include/`, implementations are in `src/`, hardware entry points are in `firmware/`, and checks are in `tests/`.

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
