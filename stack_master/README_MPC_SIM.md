# Experimental MPC selection for simulation

`race.launch.xml` defaults to `controller:=pp`; the existing Pure Pursuit
configuration and topic interfaces remain in use. `controller:=mpc` selects
the existing lateral MPC with `config/mpc_sim.yaml`, and requires the exact
argument `sim:=true`. Other controller names and real-mode MPC fail at launch
before the base system is started. This is not a real-vehicle enablement.

The sim-only YAML copies the MPC geometry and tuning from
`.humble-test/mpc-fast/mpc.yaml`, except that the controller uses the normal
`/vesc/high_level/ackermann_cmd` topic. The 19.949 s test also used a separate
forwarder with a 3 m/s cap, error-dependent slowdown and acceleration limit,
plus a 0.65 map speed-profile scale. Those are **not** applied by this launch.
Do not assume its speed, lap time or safety match the earlier test. Keep the
simulator stopped until the planned speed profile and supervision are checked.

The map argument is forwarded to `global_trajectory_publisher`, which loads
`maps/<map>/global_waypoints.json` from the selected `stack_master` package.
The simulator also requires the corresponding map and sector files. The
isolated install contains `test0` and `test1`, but not `11_16_3`; that map
has not been tested here. Install a complete map into the JSND simulation
workspace and rebuild the isolated overlay before selecting it.

The tested overlay is built under `.humble-test/selector/`, not the real-car
workspace. In each terminal, source `.humble-test/env.bash`, then
`.humble-test/selector/install/local_setup.bash`, and set a free, simulation-only
`ROS_DOMAIN_ID=213` and `ROS_LOCALHOST_ONLY=1`. Do not run this simulation in a
domain shared with a physical car. Use these terminal commands:

1. `ros2 launch stack_master race.launch.xml map:=test0 sim:=true controller:=mpc virtual:=false gp_predictor:=false`
2. `ros2 run joy joy_node`
3. `pitwall`, or `ros2 launch stack_master pitwall.launch.xml` if the alias is unavailable.

The sim launch may already open an RViz window. The headless checks observed
the controller-to-mux and `/joy` subscription connections but did not test a
physical joystick, pitwall GUI interaction, or a lap with this new launch.
Results and logs are in `.humble-test/selector/`; GUI and DDS socket errors
occurred inside the restricted headless environment. The earlier lap result is
not a validation of this launch.

Before any separate real-vehicle implementation or test, independently verify
the vehicle model, reference frame and rear-axle offset; measured steering sign,
limits and response; speed profile, braking and independent E-stop; joystick,
mux and watchdog behavior; solver failures and 20 ms deadlines under target
hardware and telemetry load. Review with an operator, begin with wheels off the
ground, and use a controlled course with a physical safety observer. MPC real
mode remains intentionally disabled; do not use the simulation geometry for a
car without measurement and a separate safety review.
