# JSND steering-only LTV MPC

## Scope and source of truth

Implementation target: **`/home/sk/JSND` tracked source**, baseline commit
`2091417c8772d87f76eafcb73f402474481c16a7`.
`.jsnd-docker/workspace/src/racing_stack` is an independent copy and is **not**
updated by this work. Existing `.jsnd-docker`, `.humble-test`, `.vscode` contents
are preserved. `install/` is not refreshed. No ROS launch, DDS control publication,
map regeneration, real vehicle connection or full build is part of this work.

The default remains `lateral_controller: PP`. Both selection and all `mpc_*`
parameters are **startup-only**. Runtime attempts are rejected before mutating
any other parameter in the same callback batch. Restarting a running controller
is an operational action, not something this implementation performs.

## Data flow and preserved behavior

```text
LiDAR → existing detection/tracking → existing Static Avoidance / Recovery
     → State Machine /behavior_strategy.local_wpnts
     → controller_manager.behavior_cb (existing nine-column array)
     → Controller.main_loop:
           existing future-position / lateral-error processing
           existing calc_speed_command + speed_adjust_heading
           PP: original L1 + heading/acceleration/speed/error steering corrections
           MPC: separate LateralMPC.solve; none of the PP steering corrections
           existing finite-value guard and AEB
           MPC failure: explicit speed=0 safety override AFTER AEB
     → existing manager watchdog → existing Ackermann publisher → existing mux
```

`FTGONLY` continues to use `ftg_cycle` instead of either path tracker. Mapping
also retains its original FTG path. No publisher was added by the MPC module.
The manager feeds its final issued steering command back to MPC after watchdog
or FTG selection. This is the command issued by this node, **not measured servo
angle or proof of what a downstream mux/estop actually applied**.

All original Controller methods except `__init__` and `main_loop` are unchanged.
In particular `calc_speed_command`, trailing control, speed reduction functions,
PP steering functions, AEB and waypoint helpers retain their original bodies.
Global waypoint speeds, sector tuner, local replanning, perception, static
avoidance, recovery spliner, state machine and VESC files are unchanged.

Same speed algorithm does not mean identical closed-loop speeds: a different
steering history changes the existing predicted lateral error. STOP on MPC
failure is an explicit safety override, not a new longitudinal optimizer.

## Model and optimization

`controller/mpc/bicycle.py` uses a rear-axle kinematic bicycle, road-wheel angle
in radians, forward Euler with `dt=1/50 s`:

\[
x^+=x+\Delta t\,v\cos\psi,\quad
y^+=y+\Delta t\,v\sin\psi,\quad
\psi^+=\psi+\Delta t\,v\tan\delta/L.
\]

Speed is an external scalar or an N-element fixed forecast. The ROS adapter uses
**constant measured current speed**, without assuming immediate acceleration to
waypoint speed. The solver never outputs or overwrites a target speed.

At each reference sample, `linearize` constructs

\[
z_{k+1}=A_k z_k+B_k\delta_k+c_k,
\quad A_{0,2}=-\Delta t\,v_k\sin\bar\psi_k,
\quad A_{1,2}=\Delta t\,v_k\cos\bar\psi_k,
\quad B_{2}=\Delta t\,v_k/(L\cos^2\bar\delta_k).
\]

Other entries of A are the identity, other entries of B are zero, and
`c=f(zbar,deltabar)-A*zbar-B*deltabar` retains the affine defect.
Reference steering is `atan(L*kappa)` from the **selected local path geometry**.

States are condensed into affine functions of N steering inputs. OSQP solves:

\[
\min_{\delta_{0:N-1}}
\sum_{k=0}^{N-1}\bigl[
q_p\Vert p_{k+1}-\bar p_{k+1}\Vert^2+
q_\psi(\psi_{k+1}-\bar\psi_{k+1})^2+
r_\delta(\delta_k-\bar\delta_k)^2+
r_{\Delta}(\delta_k-\delta_{k-1})^2\bigr].
\]

The final state's tracking cost is multiplied by `terminal_scale`.
The first slew term uses the last issued steering command. Constraints are
`abs(delta_k) <= max_steer` and
`abs(delta_k-delta_(k-1)) <= max_steer_rate * dt`.
The optimization vector contains only steering, with no speed/acceleration inputs.

For predictable repeated work, the solver reuses its fixed constraint/Hessian
sparsity structure, updates Hessian values and vectors, and shifts the previous
steering solution as a warm start. The applied first steering is clipped only
to remove numerical solver tolerance. PP corrections are never applied to it.

### Reference construction

Only local x/y columns are used by MPC. Adjacent duplicate points are removed;
projection onto the ordered local polyline chooses the start arc length.
Samples advance by `v[k]*dt`; tangent heading is unwrapped, interpolated and
aligned to the current yaw branch. Curvature is recomputed from local geometry,
so a stale global heading/curvature cannot pull the controller off an avoidance
path. Coordinates are translated near the vehicle for numerical conditioning.

No map filename, JSON file, cyclic endpoint wrap or extrapolation is used.
Insufficient path length produces a STOP request. The ROS adapter additionally
requires enough finite nine-column waypoints for the unchanged legacy speed
curvature window: at least 21 points and at least one point in the slice starting
10 points after the nearest index. Near the end of a stale local path, it stops
instead of changing the speed algorithm or propagating an empty curvature mean.

The pose-to-rear-axle transform is a signed longitudinal displacement:
`p_rear = p_pose + offset * [cos(yaw), sin(yaw)]`.
A negative offset places the rear axle behind the reported pose origin. Lateral
offset, pose/velocity frame consistency and slip are not estimated here.

## Configuration and dependency gates

`stack_master/config/controller.yaml` contains the selector and MPC tuning.
MPC startup requires:

- An **explicit** `wheelbase` ROS parameter. The old manager fallback 0.321 m is
  allowed for PP but is not sufficient to arm MPC.
- `mpc_geometry_confirmed: true` after verifying the actual pose reference,
  steering angle units/sign and geometry.
- Positive measured/approved `mpc_max_steer_rad` and
  `mpc_max_steer_rate_radps`. Their defaults are deliberately zero (invalid for MPC).
- `mpc_pose_to_rear_axle_m` matching the reported pose origin; default zero is
  meaningful only if the pose really is located at the rear axle.

Do not copy test geometry into vehicle configuration. The synthetic plant uses
L=0.32 m, angle limit=0.53 rad, slew=2 rad/s solely as a repeatable test fixture.

Algorithm tuning defaults: N=25 (0.5 s), position/heading weights 12/4, steering
and per-step difference weights 0.2/1, terminal multiplier 3, max 400 iterations.
Tracking/heading envelopes are 1 m / 0.7 rad; these are tuning envelopes, **not
obstacle clearance guarantees or vehicle-certified limits**.

OSQP is imported only when MPC is constructed. PP needs no new solver install.
`setup.py` declares optional extra `mpc`: NumPy, SciPy, OSQP `>=1.0,<2`.
An ament/colcon build does not automatically install this optional pip extra;
the actual vehicle Python interpreter must be checked explicitly.

The existing quali launch does not automatically pass the CAR vehicle config
to the controller. This work does not change that launch or assume CAR/SIM
values. An explicit confirmed configuration must be passed to the node before
MPC is enabled. The default PP configuration remains usable as before.

## Failure policy and timing

`MPCResult` exposes steering, `SafetyRequest`, status, elapsed wall time, OSQP
runtime, iteration count, reference/predicted trajectory and steering sequence.
Only OSQP `solved` is accepted; `solved inaccurate`, timeout, infeasible,
iteration limit, exceptions, nonfinite output and violated constraints request
STOP. Large initial errors, excess reference curvature, insufficient horizon,
or nonlinear rollout outside the tracking envelope also request STOP.

On failure, the previous finite steering is held within angle bounds and speed
is overridden to zero **after** legacy AEB. Holding steering is not a claim that
the vehicle has stopped. If the previous command itself exceeds the MPC angle
bound (e.g. transition from a different controller), both slew and angle limits
cannot always be satisfied at once: STOP plus the angle bound take priority.
The next successful solve may resume; there is no separate latched supervisory
re-arm policy in this implementation.

There is deliberately no automatic PP fallback: switching trackers cannot make
an absent or unsafe path safe. FTG remains a state-machine decision.

OSQP's limit defaults to 8 ms; the **whole MPC call** must finish within 15 ms,
below the 20 ms timer period. A late result is discarded and requests STOP.
This is an overrun detector, not Python hard-real-time preemption: a blocked
Python/OS call cannot emit a stop until execution resumes. Existing external
watchdogs/emergency-stop arrangements remain essential and unchanged.

With existing `measure=true`, `/controller/mpc_elapsed_ms` reports MPC elapsed
milliseconds on non-FTG cycles. Existing `/controller/latency` semantics are
unchanged (historically reciprocal elapsed time, despite the topic name).
Warning logs report failure status changes; result objects retain detailed
diagnostics for offline analysis.

The nonlinear rollout checks a tracking envelope only. There is **no obstacle
geometry input or collision constraint inside this MPC**. Static avoidance is
provided by the unchanged planner; body footprint clearance, moving obstacles,
path discontinuities and localization quality require higher-level validation.

## Offline verification

`controller/test/offline_support.py` generates straight, radius-1.2 m sharp curve,
S curve and an obstacle-detour/return path in memory. These are not real maps.

`legacy_pp_golden.json` contains 160 outputs produced from the unmodified baseline
PP, covering four paths and GB_TRACK/OVERTAKE/RECOVERY/START/TRAILING. Regression
checks compare all returned fields, including speed and steering, at 1e-12
tolerance. Visualization messages are lightweight value-object shims, without
ROS initialization. Manager safety tests execute actual method bodies with
mock publishers, and therefore do not certify ROS middleware behavior.

Model tests cover numerical Jacobians, closed-loop tracking, angle/slew limits,
avoidance/return of a point-model vehicle, invalid/short paths, yaw wrap,
duplicates, standstill, external speed forecasts, solver failures, timeouts and
recovery. Time is mocked only for deterministic model unit tests; the separate
benchmark uses real wall time and retains deadline failures.

Safe offline commands (use the approved test interpreter with OSQP available):

```bash
cd /home/sk/JSND
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=controller OPENBLAS_NUM_THREADS=1 \
  /path/to/test/python -m pytest controller/test -q -p no:cacheprovider
PYTHONPATH=controller OPENBLAS_NUM_THREADS=1 \
  /path/to/test/python controller/test/benchmark_mpc.py --output /tmp/jsnd-mpc-results.json
```

The benchmark output path must not already exist. It reports cold/warm MPC,
OSQP runtime, full `Controller.main_loop` timing with a mock Frenet converter,
cross-track errors, and all failure statuses. It excludes ROS callbacks, DDS,
real converter cost, visualization, mux and actuator latency. Laptop timings
are not a guarantee of Jetson or other vehicle CPU performance.

## Before tomorrow's vehicle deployment

1. Confirm the actual source checkout, ROS overlay, interpreter and OSQP version;
   the independent Docker copy and old install tree do not contain this change.
2. Verify wheelbase, pose/velocity frame and rear-axle offset, steering sign,
   road-wheel units, mechanical limits, slew and actuator delay. Do not simply
   use old PP software clamps as measured actuator limits.
3. Check local path ordering, freshness, point density and available horizon in
   recorded ROS messages. No map regeneration is required.
4. Confirm which emergency stop and mux/watchdog paths are active. The existing
   launch `use_estop` defaults were false; preserving source does not activate it.
   Existing AEB can return 2 m/s and is not a general emergency stop.
5. Validate offline replay/shadow computation and full callback timing on the
   vehicle CPU without publishing control. Measure under representative load.
6. Review fixed-speed feasibility and planner footprint clearance. The MPC
   tracking envelope alone does not certify collision avoidance.
7. Only after configuration and supervision are confirmed, perform separately
   authorized low-speed isolated tests. No real vehicle test is performed here.

## Future extension

The ROS adapter, rear-axle model, reference generation and solver are separated.
An integrated speed/steering MPC can later introduce a different model/result
interface and explicit longitudinal arbitration without changing this module's
steering-only contract implicitly. Horizon, objective and constraints are not
hard-coded into `calc_speed_command`.

API references: [OSQP Python](https://osqp.org/docs/interfaces/python.html),
[OSQP settings](https://osqp.org/docs/interfaces/solver_settings.html).
