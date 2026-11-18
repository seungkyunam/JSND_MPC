# Vehicle commissioning validation scope

No vehicle, Jazzy target, VESC, joystick or physical E-stop was connected for these checks.
Existing MPC algorithm / full Humble driving simulations were not rerun.

- New safety/profile/approval/graph adapter unit suite: 31 passed (one opt-in DDS test skipped in the default suite).
- New localhost shadow DDS test: exercised without simulator or hardware nodes; verified no publisher on `/vesc/ackermann_cmd`, joystick timeout despite healthy MPC, E-stop latching, duplicate control publisher rejection, scan loss and explicit neutral re-arming.
- New ament_python package build/install: verified under locally available Humble, not a Jazzy compatibility certification. Bytecode-disabled warnings are expected.
- Python syntax, Bash syntax and new-file whitespace checks: passed during local handoff checks.

Example offline test command after loading the matching ROS and MPC overlay:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q mpc_vehicle/test -p no:cacheprovider
```

The DDS test is opt-in with `MPC_SHADOW_DDS_TEST=YES`. Run only on an isolated localhost ROS domain without hardware; do not set this on a driving graph. It creates synthetic sensor and joystick publishers with the stack's real topic names. Tests never approve a vehicle or generate a production wheels-up approval record.

Outstanding deployment requirements are listed in `../VEHICLE_RUN_GUIDE.md`. In particular, actual independent E-stop hardware/bridge and firmware command-timeout behavior must be provided and verified before any nonzero hardware output. A profile checkbox or ROS service is not evidence of physical safety.
