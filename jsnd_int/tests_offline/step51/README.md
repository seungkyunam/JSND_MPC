# Step 51 offline artifacts

Base: `de416d9` (Step 50), map: repository `test0`, existing Python `/home/sk/JSND_MPC/.venv/bin/python` (OSQP 1.1.3). No packages installed. No ROS graph or hardware started.

- `compare.py`: reuses production Controller/MPC and existing `lap_sim`. `baseline` is Step 49 control settings with Step 51 acceleration-cost feature OFF; `time_cost` enables only that feature. `gain2` multiplies the final successful command **only inside this offline script**. `pp` is legacy PP; `pp_matched` caps its input speed plan to 5 m/s and geometric ay 4.5. It does not add MPC constraints to PP.
- `ablations.jsonl`: one-factor changes, zero pose delay, 60 ms servo delay, VESC tau .12. Noise .02 m, seed 0, 2 laps, 50 Hz. Older rows omit `pose_delay_ticks` (=0) and `real_deadlines` (=false). Never interpret this as a calibrated car model.
- `delay.jsonl`: pose delay 120 ms; `final_cases.jsonl`: pose 140 ms and response tau .45 sensitivity, plus PP with capped plan. These are assumptions, not identified parameters. No obstacles/TTC sensor simulation in lap_sim.
- `production_budgets.jsonl`: production 16 ms control deadline / 8 ms OSQP budget on this laptop; other runs use 20/20 ms. Neither demonstrates Jetson timing.
- `.npz`: tick traces (speed, command, cross-track, steering, stop flags, solver time). Early files without `_poseN` have zero pose delay. `samples` contains successful solves only: measured speed, optimal acceleration, first predicted speed, peak predicted speed, pre-jerk integrated command, actual clipped command. It is not aligned with all trace ticks. Use production diagnostics for the exact post-jerk/pre-clip layer.
- `probe.py`, `same_state.json`: same input state/path/previous steering and command for PP/baseline/candidate; includes full MPC diagnostics. Common input speed plan has max 5 and ay cap 4.5. MPC intentionally ignores speed columns.
- `straight_*.csv`, `straight_summary.json`: 20 m straight horizon with feedback, 50 Hz. Acceleration median includes actual speeds .5–3 m/s, not near-ceiling cruising. The gain experiment stops at first solver failure. No use of sparse real logs to invent a measured optimizer output.
- `summarize.py`, `phase_metrics.json`, `profiles.png`: saved traces only. Corner threshold |kappa|>.12/m; entry d|kappa|/ds>.05/m², exit <-.05, middle otherwise. These are pooled geometric phases, not named real-track corners. Positive `d*sign(kappa)` means inside error, not proof of boundary contact. No footprint/collision certification.
- Physical audits: both predicted nonlinear MPC friction ellipse and the simulated plant. The plant has independent longitudinal saturation and kinematic steering, **not tire friction saturation**. It can reveal infeasible demanded combined motion; passing the solver check alone is insufficient. See phase metrics for actual-plant combined-limit violations.
- `log_audit.txt`: Step 50's existing script applied to the original Step 49 log. STOP requests are status-change events, not control tick counts.

Run from repository root:

```bash
OPENBLAS_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python jsnd_int/tests_offline/step51/compare.py --variant baseline --pose-delay 6 --real-deadlines
OPENBLAS_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python jsnd_int/tests_offline/step51/compare.py --variant time_cost --pose-delay 6 --real-deadlines
OPENBLAS_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python jsnd_int/tests_offline/step51/probe.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python -m pytest -q jsnd_int/tests_offline/step51/test_step51.py jsnd_int/tests_offline/test_step45_pose_prediction.py jsnd_int/tests_offline/test_integrated_v2.py jsnd_int/tests_offline/test_step39_speed_ceiling.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD` avoids an existing global anyio/pytest version mismatch. Old tests were updated to Step 35/49 documented behavior (obstacle-cap braking recovery, bounded 10-tick continuation, soft STOP and explicit latency), without changing those production behaviors.
