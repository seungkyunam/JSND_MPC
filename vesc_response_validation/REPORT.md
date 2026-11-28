# VESC response-consistent MPC validation

Baseline commit: `42e6a99`  
Scenario: `test0`, obstacles, `improved1s`, exactly 3 laps (lap 1 warm-up)  
Structure retained: 1 s prediction, N=10, 50 Hz, existing physical limits and emergency-stop path

## Root cause

The MPC predicts its first longitudinal state with the requested acceleration, but the
simulated VESC plant applies

`a_plant = clip((v_command - v_measured) / 0.12 s, friction limits)`.

The old command adapter advanced the speed setpoint by only `a_mpc * 0.02 s`.  At the
collision (`lap=3`, `s=29.1955 m`) the values were:

- MPC/applied request: `-4.0304 m/s²`
- measured speed: `2.4020 m/s`
- VESC command: `2.3042 m/s`
- resulting plant deceleration: `(2.3042 - 2.4020) / 0.12 = -0.8145 m/s²`
- clearance: `-0.2462 m` (collision)

Thus the main error was not the MPC cost or a solver fallback. It was the inconsistent
acceleration-to-speed-command model: the 20 ms integration increment was passed to a
120 ms first-order speed plant.

## Change

- Add the measured/simulated VESC speed-response constant `speed_response_tau_s=0.12`.
- Invert the same first-order model at the command boundary:
  `v_command = v_measured + tau * a_mpc`.
- Use the same response time in the curvature braking envelope. The effective lead is
  `max(existing_brake_lead, tau)`, so the profile's 0.08 s lead becomes 0.12 s without
  double-counting an already configured delay.
- Apply the response-aware target to bounded soft STOP. Hard emergency stop and all
  command, speed, friction, steering, tracking, and physical limits remain unchanged.
- No objective weight or global speed limit was changed.

## Results

| Version | Mean timed lap | Flying laps | Collision | Avoidance | Terminal stop | Solver failure |
|---|---:|---|---:|---:|---:|---:|
| stable `f515c0e` | 13.510 s | existing run | 0 | 100% | 0 | 3.257% |
| input `42e6a99` | 13.030 s | existing run | 1 | 75% | 0 | 3.147% |
| response-consistent | **12.970 s** | 13.300, 12.640 s | **0** | **100%** | **0** | 1.002% |

The accepted result is 0.540 s (4.00%) faster than the stable version and 0.060 s
(0.46%) faster than `42e6a99`, while removing its collision. Maximum speed remained
4.987 m/s under the unchanged 5 m/s controller/plant cap. Minimum counted obstacle
clearance was 0.1082 m; avoidance geometry was not reduced.

In `s=22..30 m` (laps 2-3), braking samples with requested acceleration below
`-0.25 m/s²` changed as follows:

| Trace | Samples | Mean MPC braking | Mean plant braking | Mean absolute mismatch |
|---|---:|---:|---:|---:|
| `42e6a99` | 121 | -3.9094 m/s² | -2.6265 m/s² | 1.5326 m/s² |
| response-consistent | 108 | -3.3015 m/s² | -3.3015 m/s² | **0.0000 m/s²** |

The new run was performed once; no post-result parameter search or repeat tuning was
performed. This is simulation-only validation and was not deployed to a real vehicle.
