# test0 offline validation

This directory validates the real `stack_master/maps/test0` assets with the
production PP/MPC controller classes and production static Frenet spliner.
Results from the earlier `final_validation/` campaign are untouched.

The simulation injects synthetic static objects at the tracked `Obstacle`
message boundary. It executes `StaticObstacleSpliner._plan` and the state
machine's production velocity-profile algorithm, but it does not execute lidar
detection, clustering/tracking or the full ROS state-machine graph. The report
therefore calls this a planner/controller closed-loop test rather than a full
perception-to-actuator obstacle-system test.

Recreate all nine trials (destructive only if the output path already exists):

```bash
test0_validation/sim_env.bash test0_validation/run_matrix.py \
  --output test0_validation/results_20261127 --laps 11 --jobs 3
```

Regenerate tables and figures from the saved trials without rerunning them:

```bash
test0_validation/sim_env.bash test0_validation/analyze.py \
  --results test0_validation/results_20261127 \
  --output test0_validation/analysis_20261127
```

The tracked result set contains every attempted lap. The improved MPC's ideal
obstacle trial stopped during lap 3; its incomplete run is retained and counted
as a failure rather than removed from the comparison.
