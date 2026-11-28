# MPC control-frequency comparison

Baseline: `1775362`  
Scenario: `test0`, obstacles, `improved1s`, 3 laps per frequency  
50 Hz source: reused unchanged result from
`vesc_response_validation/test0_obstacles_3lap` (no duplicate run)

## Controlled configuration

The prediction duration, N=10, objective weights, vehicle limits, avoidance geometry,
and VESC response constant (`0.12 s`) were held fixed. No weight tuning was performed.
Each new frequency was run exactly once.

| Rate | control dt / deadline | remaining 9-stage dt | steer step at 3.2 rad/s | accel jerk step at 20 m/s³ | plan/fallback reuse |
|---:|---:|---:|---:|---:|---:|
| 25 Hz | 40.000 ms | 106.667 ms | 0.1280 rad | 0.8000 m/s² | 5 ticks = 0.20 s |
| 50 Hz | 20.000 ms | 108.889 ms | 0.0640 rad | 0.4000 m/s² | 10 ticks = 0.20 s |
| 75 Hz | 13.333 ms | 109.630 ms | 0.0427 rad | 0.2667 m/s² | 15 ticks = 0.20 s |
| 100 Hz | 10.000 ms | 110.000 ms | 0.0320 rad | 0.2000 m/s² | 20 ticks = 0.20 s |

For every row, the first MPC stage is one control period and the other nine stages
sum to the remainder, so the complete prediction is exactly 1.000 s. Steering and
positive-acceleration changes are rate limits multiplied by control dt. Solver deadline
and time limit equal one control period, and both verified-plan continuation and PP
fallback retain the original 0.20 s physical duration. Obstacle planning is tick-scaled
near 10 Hz; 25/50/100 Hz are exactly 100 ms and 75 Hz is quantized to 106.7 ms.

## Results

Lap 1 is the common warm-up; the mean uses the two timed flying laps.

| Rate | Mean lap | Max speed | Mean speed | Tracking RMSE | Avoidance | Collisions | Solver P99 | Cycle overrun |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 25 Hz | **12.340 s** | 4.996 m/s | **3.357 m/s** | **0.300 m** | 100% | 0 | 4.550 ms | **0.000%** |
| 50 Hz (reused) | 12.970 s | 4.987 m/s | 3.190 m/s | 0.315 m | 100% | 0 | **3.078 ms** | **0.000%** |
| 75 Hz | 13.073 s | 4.991 m/s | 3.164 m/s | 0.314 m | 100% | 0 | 3.621 ms | 0.051% |
| 100 Hz | 12.920 s | 4.986 m/s | 3.199 m/s | 0.308 m | 100% | 0 | 4.593 ms | 0.464% |

All four conditions completed three laps with no terminal stop, no physical-limit
violation, zero collisions, and 100% avoidance. Frequency did not materially change
maximum speed; differences came from the closed-loop timing around braking, avoidance,
and reacceleration.

25 Hz produced the shortest lap and lowest tracking RMSE in this deterministic run,
but its minimum straight-obstacle clearance was only **0.004 m**, versus 0.108 m at
50 Hz, 0.105 m at 75 Hz, and 0.105 m at 100 Hz. It is therefore a marginal result, not
a robust frequency recommendation. 100 Hz improved the reused 50 Hz mean by 0.050 s
(0.39%) and tracking RMSE by 0.007 m, but its 10 ms budget caused the highest overrun
rate (0.464%). 75 Hz had no performance advantage here. The existing 50 Hz setting
remains the balanced result with meaningful clearance and zero observed overruns.

This comparison is simulation-only; no real-vehicle deployment was performed.
