> 최신 통합 MPC 실행/검증은 [INTEGRATED_MPC_STATUS.md](../INTEGRATED_MPC_STATUS.md)를 참조한다. 아래 문서는 차량 안전 게이트와 기존 PP 복구 경로를 함께 설명한다.

# MPC SSH staging (simulation only)

This stages source for an isolated ROS 2 Humble or Jazzy simulation. Jazzy is
not yet validated on a Jazzy system. It does **not**
deploy a real-vehicle controller, change `/home/sk/rober`, authorize motion,
or remove the `sim:=true` launch guard. Do not bypass the guard by launching
the controller directly. The target machine, architecture, dependencies,
joystick and actuator behavior have not been verified by staging.

## Source comparison and transfer

On the laptop, from the JSND root, first inspect the target:

```bash
./deploy_mpc.sh VEHICLE_IP USERNAME /absolute/vehicle/workspace
```

This reads source files below the target workspace's `src`, compares hashes,
and lists differing and one-sided files. It writes only local inspection data.
Review the full report, including missing and duplicate package errors. The
source layout can contain symlinks to the workspace's source tree, but symlinks
inside the selected package contents are rejected and require review.

After review, the following repeats inspection and asks for `STAGE` before
copying to a new `$HOME/mpc_ws` directory. It refuses an existing directory or
symlink and never replaces
files in the existing workspace. SSH host authentication remains enabled.

```bash
./deploy_mpc.sh VEHICLE_IP USERNAME /absolute/vehicle/workspace --apply --ros-distro auto
```

The source snapshot contains `controller`, `stack_master`, these scripts and
the source maps; it does **not** contain a complete simulator or the map
`11_16_3`. It cannot make a missing target map available. Only use an
approved, complete map from an independently verified installation.

## Target build

SSH into the vehicle, then enter the exact new staging path printed by the
deploy command. Stop before a build if the existing underlay path or ROS
version is uncertain. No build output belongs in the existing workspace.

```bash
cd /path/printed/by/deploy
./build_mpc.sh /absolute/vehicle/workspace --ros-distro auto
```

`auto` detects the underlay distribution; specify `humble` or `jazzy` to
require that version. The scripts reject distribution mismatches. Use a clean
shell and a fresh overlay for each distribution; do not mix binary packages.
The script sources `/opt/ros/<distribution>` and the selected workspace underlay,
creates a new `mpc-build.*` directory and Python virtual environment under the
stage, and builds only `controller` and `stack_master` there. It checks for
NumPy, SciPy and OSQP version constraints plus core ROS Python imports before
building. This is not an exhaustive check of launch dependencies. If packages
are missing and the
operator authorizes network installation into the new virtual environment,
use `./build_mpc.sh /absolute/vehicle/workspace --install-deps`. This does not
install system packages. The target must supply `python3-venv`, pip, Python
`packaging`, colcon,
ament build tools, the matching ROS distribution and other underlay packages;
`mpc_deployment/requirements.txt` lists the optional Python MPC dependencies.
Inspect build errors and generated console-script shebangs on the target; a
successful build on a laptop does not establish target compatibility.

The overlay has the same ROS package names as the underlay. Sourcing it masks
the underlay's `controller`, `stack_master`, configuration and maps even for
PP. Never source it in a production or normal PP driving shell. Start a clean
shell and source only the original workspace to return to the original PP.

## Simulation and interfaces

The run script checks its build marker, requires an explicit isolation
acknowledgment and nonzero ROS domain, refuses known custom DDS discovery
configuration, checks for existing ROS nodes, and always passes `sim:=true`.
These checks do not prove isolation. Physically disable vehicle actuators,
disconnect or isolate real ROS networks and bridges, and verify the domain is
free before testing. Neither Humble's `ROS_LOCALHOST_ONLY=1` nor Jazzy's
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` isolates another process on the same
host or guarantees correct DDS configuration.

```bash
SIM_ONLY_ISOLATED=YES ROS_DOMAIN_ID=213 ./run_mpc.sh /path/to/mpc-build.XXXXXX test0 mpc --ros-distro auto
```

The controller argument can also be `pp`, for simulation regression only. The race
launch defaults to PP and selects MPC only with `controller:=mpc sim:=true`.
It uses the existing map's global trajectory and sector speed scaling through
the state machine's local path, not a new MPC-specific speed profile. The
controller publishes to `/vesc/high_level/ackermann_cmd`; `simple_mux` consumes
it and `/joy`, and publishes `/vesc/ackermann_cmd`. Do not assume the
simulation overlay reproduces the earlier 19.949-second forwarder experiment.

On an already-running isolated simulation, use a separate shell with the same
underlay, virtual environment, overlay and `ROS_DOMAIN_ID`. Use
`ROS_LOCALHOST_ONLY=1` for Humble, or
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for Jazzy (without setting
`ROS_LOCALHOST_ONLY`):

```bash
ros2 param get /controller_manager sim
ros2 param get /controller_manager lateral_controller
ros2 topic info --no-daemon -v /vesc/high_level/ackermann_cmd
ros2 topic info --no-daemon -v /vesc/ackermann_cmd
ros2 topic info --no-daemon -v /joy
```

Nodes or topics visible there are not evidence that the actual joystick,
actuator, safety stop or a full lap works. Domain, middleware and network
must be checked independently on both machines. A joystick node must run on
the computer to which the physical joystick is connected; a remote joystick
requires a separately verified communication-loss safety policy.
The bundled `sim_probe.py` validates the distribution-specific observer settings
above, rejects custom DDS configuration, and launches its own MPC simulation.
Run it on an empty, physically isolated domain, not alongside an existing launch:

```bash
SIM_ONLY_ISOLATED=YES ROS_DOMAIN_ID=213 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 mpc_deployment/sim_probe.py /path/to/jazzy-build test0 /path/to/new-results
```

Source the matching underlay, build venv and overlay first; unset
`ROS_LOCALHOST_ONLY` for Jazzy. For Humble replace the discovery assignment with
`ROS_LOCALHOST_ONLY=1`. The child launcher sets its own discovery environment;
the observer retains the validated setting. This adaptation has unit coverage,
but actual Jazzy DDS discovery and simulation remain unverified.

## Conditions before a real-mode design

Real MPC remains blocked. A separate, reviewed change would need a vehicle-only
configuration based on measured wheelbase, pose origin and signed rear-axle
offset, steering sign/limits/rate, speed conversion and braking distance; a
verified vehicle map and speed profile; and an approved deployment method
that preserves the original PP workspace. It must also provide and validate
an independent E-stop and fail-closed actuator-side supervision for joystick,
sensor, command, solver, process and network failures, including deliberate
re-arming. Verify all alternate controller states including FTG, low-speed
limits and worst-case timing on target hardware. The existing joystick mux
does not disarm autonomous mode on joystick loss; the software E-stop is
disabled by default and can pass commands on missing input. The current MPC
failure path requests zero but is not an independently verified latched stop.
None of these requirements is satisfied by the staging scripts.
