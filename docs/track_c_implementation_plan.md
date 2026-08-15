# Track C Advanced Swing-Up: Corrected Implementation Plan

## Project objective

Build a hands-free linear cart-pole that starts with the pendulum hanging downward, pumps energy into the pendulum, catches it near the upright equilibrium, and balances it without touching a rail limit. Optimize only after the system is safe and repeatable.

The proposal's energy-based swing-up plus LQR architecture is appropriate. The original two-page plan is not yet an implementation specification: it omits the angle convention, nonlinear model, actuator limits, state estimation, switch hysteresis, rail-end protection, timing requirements, and numerical pass/fail criteria.

## What the three references actually establish

1. **Scott Rumschlag, "Inverted Pendulum(s) #3 - Swing Up"** demonstrates a linear cart-pole and emphasizes motor performance/feedback and continuous speed/acceleration monitoring. It is a useful warning that a controller cannot compensate for an actuator whose acceleration, braking, or feedback is unknown.
2. **Cillk 11, "Rotary Inverted Pendulum (PID) - Design, Build, Model, Swing Up and Stabilisation"** is a rotary inverted pendulum, not a linear cart-pole. It uses an Arduino Mega, geared motor, two optical encoders, and an L298N driver. The presentation reports successful PID stabilization/swing-up; LQR stabilization and an energy-forcing swing-up are listed as future work. Its strongest reusable contribution is the disciplined development and test matrix, not direct controller gains or mechanical geometry.
3. **Brianno Coller, "Classic Inverted Pendulum - Equations of Motion"** derives the full nonlinear equations for the linear cart-pole with Newtonian free-body diagrams. This is the relevant modeling foundation. Linearization is valid only near upright; swing-up must use the nonlinear model or an energy argument.

Reference URLs:

- https://www.youtube.com/watch?v=hQK_3C6S4Ak
- https://www.youtube.com/watch?v=bY4t6yfBA24
- https://www.youtube.com/watch?v=5qJY-ZaKSic

## Non-negotiable conventions

- Cart position `x = 0` at rail center; positive toward the motor-defined positive direction.
- Pendulum angle `theta = 0` upright and `theta = pi` hanging downward.
- Wrap angle error to `[-pi, pi)` before control and scoring.
- State vector: `z = [x, x_dot, theta, theta_dot]^T`.
- Controller output is a requested motor force or cart acceleration. A separately calibrated inner motor loop converts that request to PWM/current.
- Record units in SI: meters, seconds, radians, newtons, amperes, and volts.

Changing any sign convention requires rerunning the polarity tests; copying gains from another build is unsafe.

## Dynamic model

Use a cart mass `M`, pendulum mass `m`, pivot-to-center-of-mass distance `l`, pendulum inertia about its center of mass `I`, viscous cart friction `b`, pivot damping `c`, gravity `g`, and horizontal actuator force `F`.

Define `J = I + m*l^2`. One consistent nonlinear model for `theta = 0` upright is:

```text
(M + m) x_ddot + b x_dot + m l (theta_ddot cos(theta) - theta_dot^2 sin(theta)) = F
J theta_ddot + c theta_dot - m g l sin(theta) + m l x_ddot cos(theta) = 0
```

Derive the linear upright model from the same equations and measured parameters, then verify its signs against small manual perturbations. Do not tune LQR from nominal catalog values alone.

## Hardware baseline

### Required

- Rigid rail with low play and enough usable travel for at least one full energy-pumping cycle.
- Belt-driven DC motor or servo-class actuator with bidirectional current capability, braking, and acceleration headroom.
- High-resolution quadrature encoder for pendulum angle.
- Cart position from a motor/cart encoder with no belt-slip ambiguity; a direct linear encoder is better if available.
- STM32-class controller preferred. An Arduino Uno may prove the concept but has little timing, memory, and diagnostics margin for two encoders, estimation, logging, safety, and a 500-1000 Hz loop.
- Modern low-loss H-bridge sized from measured stall/current transients. Do not select an L298N merely because it appears in the rotary reference; its voltage loss and thermal limits make it a poor default for a fast cart.
- Normally closed hard limit switches at both ends, independent motor-enable cutoff, emergency stop, fused supply, and mechanical end stops with energy absorption.
- Motor current sensing and supply-voltage measurement.

### Optional, not primary feedback

- An ultrasonic sensor can serve as a coarse sanity check, but not as the high-rate cart state sensor. Its latency, beam geometry, and outliers are incompatible with the balance loop.

## Real-time architecture

- Control interrupt: start at 1 kHz; accept 500 Hz only after measured timing and stability tests.
- Timestamp encoder edges or sample hardware counters atomically.
- Estimate velocities with finite differences followed by a low-lag IIR filter. Log both raw and filtered values.
- Run safety checks before computing control and again before applying PWM.
- Use output saturation, slew-rate limiting, current limiting, and a watchdog.
- Stream or buffer telemetry outside the control interrupt.
- Log `time, mode, x, x_dot, theta, theta_dot, command, PWM, current, voltage, fault_flags`.

## Controller state machine

```text
BOOT -> CALIBRATE -> IDLE -> SWING_UP -> CAPTURE -> BALANCE
                         \-> FAULT <- any active mode
```

- **BOOT/CALIBRATE:** verify encoder direction, find rail center at low speed, validate limit switches, check current sensor, and require operator arm.
- **IDLE:** motor disabled or holding zero command; all states valid.
- **SWING_UP:** nonlinear energy shaping with rail-centering and actuator limits.
- **CAPTURE:** blend the swing-up command toward LQR over 50-150 ms to avoid a command step.
- **BALANCE:** full-state LQR plus optional integral correction on cart position.
- **FAULT:** disable the bridge, preserve logs, and require a deliberate reset.

Never switch based on angle alone.

## Energy-based swing-up

With the chosen convention, use pendulum energy relative to upright:

```text
E = 0.5 * J * theta_dot^2 + m*g*l*(cos(theta) - 1)
E_target = 0
```

A starting energy-shaping command is proportional to `E * theta_dot * cos(theta)`, with the final polarity determined by the motor-direction calibration. Add a rail-centering term and limit the requested acceleration:

```text
a_energy = k_E * E * theta_dot * cos(theta)
a_center = -k_x_su*x - k_v_su*x_dot
a_cmd = saturate(a_energy + a_center, -a_max, +a_max)
```

Practical additions:

- Suppress chattering near `theta_dot = 0` with a smooth sign/tanh function or a deadband.
- Reduce energy injection as the cart enters the soft-limit zone.
- If there is insufficient travel for a catch, dump energy and recenter rather than forcing another pump.
- Learn `a_max` from safe actuator tests, not from the power-supply rating.

## LQR stabilization and capture

Linearize the identified model about `theta = 0`, discretize it at the measured loop period, and compute `u = -Kz`. Choose `Q` and `R` from normalized physical limits so each state penalty is interpretable:

```text
Q = diag(1/x_allow^2, 1/v_allow^2, 1/theta_allow^2, 1/omega_allow^2)
R = 1/u_allow^2
```

Initial capture thresholds, to be validated in simulation and low-energy tests:

- Enter CAPTURE only if `|theta| < 12 deg`, `|theta_dot| < 1.5 rad/s`, and `|x| < 60%` of usable half-travel for at least 30 ms.
- Enter BALANCE after CAPTURE if `|theta| < 8 deg` for 100 ms.
- Return to SWING_UP if `|theta| > 20 deg` or `|theta_dot| > 2.5 rad/s`, provided rail position is safe.
- Enter FAULT rather than SWING_UP if a hard/soft limit, overcurrent, invalid encoder, timing overrun, or watchdog condition is active.

These are starting gates, not claimed final values. Hysteresis and dwell time are mandatory.

## Rail and electrical safety envelope

Define usable half-travel `x_max` after excluding mechanical stop clearance.

- Green: `|x| <= 0.70*x_max`; normal control.
- Amber: `0.70*x_max < |x| <= 0.85*x_max`; reduce energy injection and bias toward center.
- Red: `0.85*x_max < |x| <= 0.95*x_max`; cancel swing-up, command bounded braking/centering only.
- Trip: `|x| > 0.95*x_max`, a limit switch opens, encoder validity fails, current exceeds its time-current envelope, or control timing overruns repeatedly; disable motor and latch FAULT.

The software envelope supplements rather than replaces hard switches and mechanical stops.

## Development sequence with exit gates

1. **Mechanical and electrical checkout**
   - Measure travel, masses, center of mass, inertia estimate, backlash, supply limits, and current.
   - Exit: cart can traverse at low speed, both limits stop motion, emergency stop works, encoder signs are documented.
2. **Motor characterization**
   - Map PWM/current to acceleration in both directions; measure deadband, braking, friction, saturation, and delay.
   - Exit: repeatable acceleration model and safe `a_max`/current envelope.
3. **State estimation**
   - Calibrate counts per meter/radian and velocity filters.
   - Exit: no missed counts in worst-case motion; velocity noise and lag are quantified.
4. **Parameter identification and simulation**
   - Estimate `M, m, l, I, b, c`; compare free-swing and cart-step data with the nonlinear model.
   - Exit: simulated direction, frequency, and damping agree with experiments within documented tolerance.
5. **Upright stabilization first**
   - Manually place the pendulum near upright with swing-up disabled; tune discrete LQR under command/current limits.
   - Exit: at least 20 catches from small perturbations with no safety trips and acceptable rail centering.
6. **Swing-up at reduced power**
   - Tune energy gain and centering without enabling capture.
   - Exit: approach the capture region repeatably while staying inside the amber zone.
7. **Capture integration**
   - Enable dwell, hysteresis, and command blending.
   - Exit: at least 18 successful hands-free catches in 20 consecutive trials.
8. **Performance optimization**
   - Tune only after repeatability; vary one factor at a time and keep configuration/version identifiers in every log.

## Test matrix

- Encoder direction, wraparound, disconnect, and implausible-jump tests.
- Left/right actuator symmetry, braking, deadband, saturation, and supply-sag tests.
- Both hard limits, software zones, emergency stop, watchdog, overcurrent, and loop-overrun tests.
- Upright catch from a grid of initial angles and angular velocities.
- Swing-up repeatability from rest and from small initial disturbances.
- Disturbance rejection after balance.
- Reduced-rail-range test.
- Simulation-versus-hardware overlay for angle, cart position, and command.
- Thermal soak and repeated-trial test.

## Scoring and acceptance

The original plan's generic 10%-to-90% rise-time definition is ambiguous for a wrapped 180-degree swing-up. Before optimizing, obtain the official Track C rule text and encode its exact start event, target, settling band, trial timeout, and failure handling.

Until then, report these engineering metrics:

- Swing-up time: arm/start event to first entry into the capture region.
- Capture time: first capture entry to continuous residence inside `|theta| <= 5 deg` for 1 s.
- Total success time: start to 1 s stable balance.
- Peak upright angular error after capture.
- Peak cart displacement and minimum rail clearance.
- Settling time, RMS angle error, RMS cart-position error, peak current, and energy used.
- Success rate over 20 consecutive trials, with every fault counted as a failure.

Provisional project acceptance:

- At least 18/20 successful autonomous trials.
- No contact with a hard stop and no bypassed safety interlock.
- Stable balance for at least 30 s per successful trial.
- No sustained overcurrent or missed control deadlines.
- All raw logs and controller parameters retained for each scored trial.

## Immediate decisions before purchasing hardware

1. Confirm the official Track C rules and physical size/power constraints.
2. Measure or choose usable rail travel, cart mass, pendulum mass/length, and target swing-up time.
3. Select a motor, transmission, driver, and supply from required peak cart acceleration and current—not from a reference video's parts list.
4. Choose encoder resolutions that keep quantization noise below the velocity-estimation target at 1 kHz.
5. Freeze the sign convention and safety wiring diagram before writing controller code.

## Definition of done

The project is complete only when a cold-start, hands-free run performs calibration, swing-up, capture, and 30 seconds of upright balance; respects the rail/current/timing envelope; produces a complete log; and repeats successfully in at least 18 of 20 consecutive trials.
