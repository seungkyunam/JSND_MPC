# PP vs Integrated LTV-MPC 비교 결과 (자동 생성)

생성: 2026-11-26 21:12. 측정값만 표기, 없는 값은 "미측정". 정의는 `cmp_analyze.py` 머리말.

## 비교 조건별 요약 (같은 cap_label·layout·map끼리만 비교)

### cap=unknown · layout=unknown · map=? — **비교 불가: ?만 있음**

| 항목 |  `3p0_20261124_203250` |  `lowspeed_20261124_204417` |
|---|---|---|
| 랩 수 | 3 | 6 |
| 최고 랩 [s] | 15.135 | 14.692 |
| 평균 랩 [s] | 15.245 | 15.243 |
| 랩 표준편차 [s] | 0.096 | 0.341 |
| 횡오차 RMSE [m] | 0.281 | 0.2765 |
| 횡오차 RMSE GB_TRACK [m] | 미측정 | 0.1595 |
| 평균 속도 [m/s] | 0.919 | 0.934 |
| 최고 속도 [m/s] | 0.994 | 0.992 |
| 조향 RMS [rad] | 0.1301 | 0.1316 |
| 조향 변화율 RMS [rad/s] | 0.705 | 0.937 |
| 가속도 RMS [m/s²] | 0.239 | 0.227 |
| 최대 가속 [m/s²] | 1.588 | 1.531 |
| 최대 감속 [m/s²] | 2.399 | 2.67 |
| Jerk RMS [m/s³] | 2.17 | 1.73 |
| 계산시간 p50 [ms] | 4.434 | 5.683 |
| 계산시간 p99 [ms] | 16.371 | 15.831 |
| 실제 제어 주파수 [Hz] | 49.02 | 49.15 |
| MPC solved [%] | 98.94 | 99.24 |
| 회피 성공률 [%] | 미측정 | 미측정 |
| 회피 시도 | 미측정 | 미측정 |
| 정지 횟수 | 0 | 0 |
| 정지/랩 | 0.0 | 0.0 |
| 수동 개입 | 1 | 1 |
| 최소 LiDAR 거리 [m] | 미측정 | 미측정 |
| 조향 포화(자기 한계) [%] | 미측정 | 미측정 |
| 조향 포화(≥0.395) [%] | 0.0 | 0.0 |
| 조건 출처 | none, profile=?,  | ~/.ros/log/2026-11-24-20-44-05-828617-jsnd-9055 (cmd 기록 없음), profile=?,  |

- `3p0_20261124_203250` 미기록 토픽: /controller/mpc_loop_period_ms /controller/latency /scan /state_machine /global_waypoints /lap_data
- `lowspeed_20261124_204417` 미기록 토픽: /controller/mpc_loop_period_ms /controller/latency /scan /global_waypoints /lap_data

### cap=3p0 · layout=unknown · map=11_18_6 — **비교 불가: mpc만 있음**

| 항목 | mpc `3p0bash_20261124_211343` |
|---|---|
| 랩 수 | 11 |
| 최고 랩 [s] | 6.863 |
| 평균 랩 [s] | 7.46 |
| 랩 표준편차 [s] | 0.418 |
| 횡오차 RMSE [m] | 0.3039 |
| 횡오차 RMSE GB_TRACK [m] | 0.3108 |
| 평균 속도 [m/s] | 1.96 |
| 최고 속도 [m/s] | 2.903 |
| 조향 RMS [rad] | 0.1883 |
| 조향 변화율 RMS [rad/s] | 2.669 |
| 가속도 RMS [m/s²] | 1.072 |
| 최대 가속 [m/s²] | 1.813 |
| 최대 감속 [m/s²] | 4.641 |
| Jerk RMS [m/s³] | 5.55 |
| 계산시간 p50 [ms] | 6.966 |
| 계산시간 p99 [ms] | 22.169 |
| 실제 제어 주파수 [Hz] | 48.14 |
| MPC solved [%] | 80.79 |
| 회피 성공률 [%] | 미측정 |
| 회피 시도 | 미측정 |
| 정지 횟수 | 1 |
| 정지/랩 | 0.091 |
| 수동 개입 | 2 |
| 최소 LiDAR 거리 [m] | 0.184 |
| 조향 포화(자기 한계) [%] | 3.27 |
| 조향 포화(≥0.395) [%] | 3.27 |
| 조건 출처 | ~/.ros/log/2026-11-24-21-13-35-573325-jsnd-15953 (cmd 기록 없음) + bench_logs launch log, profile=3p0,  |

- `3p0bash_20261124_211343` 미기록 토픽: /controller/mpc_loop_period_ms /controller/latency /global_waypoints /lap_data

### cap=4p0 · layout=unknown · map=11_18_6 — **비교 불가: mpc만 있음**

| 항목 | mpc `4p0_20261124_182441` |
|---|---|
| 랩 수 | 0 |
| 최고 랩 [s] | 미측정 |
| 평균 랩 [s] | 미측정 |
| 랩 표준편차 [s] | 미측정 |
| 횡오차 RMSE [m] | 미측정 |
| 횡오차 RMSE GB_TRACK [m] | 미측정 |
| 평균 속도 [m/s] | 미측정 |
| 최고 속도 [m/s] | 미측정 |
| 조향 RMS [rad] | 미측정 |
| 조향 변화율 RMS [rad/s] | 미측정 |
| 가속도 RMS [m/s²] | 미측정 |
| 최대 가속 [m/s²] | 미측정 |
| 최대 감속 [m/s²] | 미측정 |
| Jerk RMS [m/s³] | 미측정 |
| 계산시간 p50 [ms] | 미측정 |
| 계산시간 p99 [ms] | 미측정 |
| 실제 제어 주파수 [Hz] | 미측정 |
| MPC solved [%] | 미측정 |
| 회피 성공률 [%] | 미측정 |
| 회피 시도 | 미측정 |
| 정지 횟수 | 0 |
| 정지/랩 | 미측정 |
| 수동 개입 | 0 |
| 최소 LiDAR 거리 [m] | 미측정 |
| 조향 포화(자기 한계) [%] | 미측정 |
| 조향 포화(≥0.395) [%] | 미측정 |
| 조건 출처 | ~/.ros/log/2026-11-24-18-20-55-361152-jsnd-8327 (cmd 기록 없음) + bench_logs launch log, profile=4p0,  |

- `4p0_20261124_182441` 미기록 토픽: /controller/mpc_loop_period_ms /controller/latency /scan /state_machine /global_waypoints /lap_data

### cap=5p0 · layout=unknown · map=11_18_6 — **비교 불가: mpc만 있음**

| 항목 | mpc `5p0_20261124_205612` | mpc `step45_mpc_20261126_182005` |
|---|---|---|
| 랩 수 | 3 | 13 |
| 최고 랩 [s] | 6.299 | 6.612 |
| 평균 랩 [s] | 8.16 | 10.551 |
| 랩 표준편차 [s] | 2.485 | 5.128 |
| 횡오차 RMSE [m] | 0.3405 | 0.4683 |
| 횡오차 RMSE GB_TRACK [m] | 0.3597 | 0.3818 |
| 평균 속도 [m/s] | 2.054 | 1.82 |
| 최고 속도 [m/s] | 3.145 | 3.207 |
| 조향 RMS [rad] | 0.0375 | 0.1739 |
| 조향 변화율 RMS [rad/s] | 0.402 | 5.292 |
| 가속도 RMS [m/s²] | 0.225 | 1.132 |
| 최대 가속 [m/s²] | 1.947 | 1.975 |
| 최대 감속 [m/s²] | 4.832 | 4.356 |
| Jerk RMS [m/s³] | 1.04 | 5.86 |
| 계산시간 p50 [ms] | 0.853 | 5.383 |
| 계산시간 p99 [ms] | 11.052 | 22.277 |
| 실제 제어 주파수 [Hz] | 49.08 | 47.34 |
| MPC solved [%] | 3.03 | 64.22 |
| 회피 성공률 [%] | 미측정 | 미측정 |
| 회피 시도 | 미측정 | 미측정 |
| 정지 횟수 | 1 | 8 |
| 정지/랩 | 0.333 | 0.615 |
| 수동 개입 | 3 | 1 |
| 최소 LiDAR 거리 [m] | 미측정 | 0.134 |
| 조향 포화(자기 한계) [%] | 0.1 | 7.72 |
| 조향 포화(≥0.395) [%] | 0.1 | 7.72 |
| 조건 출처 | ~/.ros/log/2026-11-24-20-56-11-342197-jsnd-10890 (cmd 기록 없음) + bench_logs launch log, profile=5p0,  | ~/.ros/log/2026-11-26-18-20-00-082650-jsnd-9408 (mpc_car.yaml 로드) + bench_logs launch log, profile=5p0, 7ec7263 |

- `5p0_20261124_205612` 미기록 토픽: /controller/mpc_loop_period_ms /controller/latency /scan /global_waypoints /lap_data
- `step45_mpc_20261126_182005` 미기록 토픽: /controller/latency /lap_data

### cap=lowspeed · layout=unknown · map=11_18_6 — **비교 불가: mpc만 있음**

| 항목 | mpc `step45_improved_20261125_142036` | mpc `step45_mpc_20261126_192651` | mpc `step45_mpc_20261126_193127` |
|---|---|---|---|
| 랩 수 | 6 | 0 | 3 |
| 최고 랩 [s] | 6.941 | 미측정 | 14.675 |
| 평균 랩 [s] | 10.941 | 미측정 | 18.828 |
| 랩 표준편차 [s] | 3.944 | 미측정 | 3.683 |
| 횡오차 RMSE [m] | 0.2831 | 0.3874 | 0.5256 |
| 횡오차 RMSE GB_TRACK [m] | 0.2129 | 0.4317 | 0.2886 |
| 평균 속도 [m/s] | 1.193 | 0.807 | 0.834 |
| 최고 속도 [m/s] | 2.877 | 0.978 | 0.985 |
| 조향 RMS [rad] | 0.1783 | 0.1868 | 0.2083 |
| 조향 변화율 RMS [rad/s] | 1.661 | 1.533 | 2.195 |
| 가속도 RMS [m/s²] | 0.68 | 0.515 | 0.568 |
| 최대 가속 [m/s²] | 1.855 | 1.768 | 2.138 |
| 최대 감속 [m/s²] | 4.797 | 3.46 | 2.748 |
| Jerk RMS [m/s³] | 3.85 | 5.2 | 3.52 |
| 계산시간 p50 [ms] | 5.027 | 5.635 | 5.906 |
| 계산시간 p99 [ms] | 18.378 | 14.478 | 17.081 |
| 실제 제어 주파수 [Hz] | 46.8 | 48.44 | 48.63 |
| MPC solved [%] | 78.63 | 89.66 | 86.62 |
| 회피 성공률 [%] | 미측정 | 미측정 | 미측정 |
| 회피 시도 | 미측정 | 미측정 | 미측정 |
| 정지 횟수 | 3 | 1 | 8 |
| 정지/랩 | 0.5 | 미측정 | 2.667 |
| 수동 개입 | 5 | 1 | 1 |
| 최소 LiDAR 거리 [m] | 0.15 | 0.117 | 0.099 |
| 조향 포화(자기 한계) [%] | 7.26 | 3.91 | 11.49 |
| 조향 포화(≥0.395) [%] | 7.26 | 3.91 | 11.49 |
| 조건 출처 | ~/.ros/log/2026-11-25-14-19-13-179712-jsnd-11657 (mpc_car.yaml 로드) + bench_logs launch log, profile=lowspeed, 4959165 | ~/.ros/log/2026-11-26-19-25-59-676951-jsnd-8863 (mpc_car.yaml 로드) + bench_logs launch log, profile=lowspeed, 3a2ba8f | ~/.ros/log/2026-11-26-19-25-59-676951-jsnd-8863 (mpc_car.yaml 로드) + bench_logs launch log, profile=lowspeed, 3a2ba8f |

- `step45_improved_20261125_142036` 미기록 토픽: /controller/latency /global_waypoints /lap_data
- `step45_mpc_20261126_192651` 미기록 토픽: /controller/latency /lap_data
- `step45_mpc_20261126_193127` 미기록 토픽: /controller/latency /lap_data

### cap=7p0 · layout=unknown · map=11_18_6 — **비교 불가: mpc만 있음**

| 항목 | mpc `step45_mpc_20261126_184738` |
|---|---|
| 랩 수 | 14 |
| 최고 랩 [s] | 6.197 |
| 평균 랩 [s] | 6.508 |
| 랩 표준편차 [s] | 0.265 |
| 횡오차 RMSE [m] | 0.418 |
| 횡오차 RMSE GB_TRACK [m] | 0.3501 |
| 평균 속도 [m/s] | 2.141 |
| 최고 속도 [m/s] | 3.082 |
| 조향 RMS [rad] | 0.1781 |
| 조향 변화율 RMS [rad/s] | 2.614 |
| 가속도 RMS [m/s²] | 1.131 |
| 최대 가속 [m/s²] | 3.594 |
| 최대 감속 [m/s²] | 3.207 |
| Jerk RMS [m/s³] | 6.41 |
| 계산시간 p50 [ms] | 7.622 |
| 계산시간 p99 [ms] | 23.497 |
| 실제 제어 주파수 [Hz] | 46.58 |
| MPC solved [%] | 84.48 |
| 회피 성공률 [%] | 미측정 |
| 회피 시도 | 미측정 |
| 정지 횟수 | 0 |
| 정지/랩 | 0.0 |
| 수동 개입 | 1 |
| 최소 LiDAR 거리 [m] | 0.347 |
| 조향 포화(자기 한계) [%] | 1.48 |
| 조향 포화(≥0.395) [%] | 1.48 |
| 조건 출처 | ~/.ros/log/2026-11-26-18-44-11-199062-jsnd-16135 (mpc_car.yaml 로드) + bench_logs launch log, profile=7p0, a5e7044 |

- `step45_mpc_20261126_184738` 미기록 토픽: /controller/latency /lap_data

## lap_analyser 텍스트 기록 (bag 없음 — 랩타임만, 수동 개입 랩 구분 불가)

주의: lap_analyser는 자율/수동 구분 없이 결승선 통과 간격을 기록한다. 정지·수동 개입 랩이 섞여 있어 평균·표준편차는 왜곡된다(최고·중앙값 위주로 볼 것). bag 기반 표와 같은 조건 비교가 아니다.

### 11_18_6 맵 컨트롤러·프로파일별 합계

| 컨트롤러 | 프로파일 | 세션 수 | 랩 수 | 최고 [s] | 중앙값 [s] |
|---|---|---|---|---|---|
| mpc | 1p5 | 1 | 5 | 11.705 | 12.784 |
| mpc | 2p25 | 1 | 7 | 8.380 | 13.888 |
| mpc | 3p0 | 9 | 111 | 3.098 | 7.714 |
| mpc | 50 | 1 | 6 | 14.251 | 14.999 |
| mpc | 5p0 | 13 | 97 | 4.260 | 9.132 |
| mpc | 7p0 | 6 | 258 | 3.359 | 8.138 |
| mpc | lowspeed | 7 | 51 | 6.439 | 20.089 |
| pp | pp_default | 1 | 18 | 5.447 | 8.084 |

### 세션별

| 세션 | 컨트롤러 | 프로파일 | 맵 | 분류 근거 | 랩 수 | 최고 [s] | 중앙값 [s] | 평균 [s] | 표준편차 [s] |
|---|---|---|---|---|---|---|---|---|---|
| 01-01 09:01:03 `lap_analyzer_0101_0901.txt` | pp | pp_default | ? | MPC 통합(11-22) 이전 날짜 → 원본 JSND(PP) | 14 | 3.009 | 3.039 | 3.078 | 0.076 |
| 11-12 13:25:14 `lap_analyzer_1211_1325.txt` | pp | pp_default | test0 | ~/.ros/log/2026-11-12-13-25-06-656531-jsnd-24531 (controller cmd에 mpc_car.yaml 없음) | 1 | 19.525 | 19.525 | 19.525 | nan |
| 11-16 19:25:59 `lap_analyzer_1611_1925.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-16-19-25-51-888197-jsnd-28380 (원본 JSND controller_manager) | 11 | 9.489 | 13.640 | 40.831 | 83.255 |
| 11-16 19:35:10 `lap_analyzer_1611_1935.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-16-19-35-01-378950-jsnd-31059 (원본 JSND controller_manager) | 6 | 5.330 | 5.490 | 5.722 | 0.657 |
| 11-16 19:43:20 `lap_analyzer_1611_1943.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-16-19-43-12-311895-jsnd-33404 (원본 JSND controller_manager) | 2 | 6.106 | 6.538 | 6.538 | 0.611 |
| 11-16 19:53:04 `lap_analyzer_1611_1953.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-16-19-52-56-785857-jsnd-38837 (원본 JSND controller_manager) | 5 | 5.882 | 6.380 | 6.872 | 0.931 |
| 11-16 20:00:33 `lap_analyzer_1611_2000.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-16-20-00-25-551901-jsnd-41743 (원본 JSND controller_manager) | 2 | 5.665 | 7.896 | 7.896 | 3.156 |
| 11-16 20:02:01 `lap_analyzer_1611_2002.txt` | pp | pp_default | ? | MPC 통합(11-22) 이전 날짜 → 원본 JSND(PP) | 31 | 3.009 | 3.217 | 31.463 | 118.859 |
| 11-16 20:16:24 `lap_analyzer_1611_2016.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-16-20-16-06-142642-jsnd-44685 (원본 JSND controller_manager) | 27 | 3.010 | 3.154 | 8.120 | 24.861 |
| 11-18 19:20:03 `lap_analyzer_1811_1920.txt` | pp | pp_default | 11_18_4 | ~/.ros/log/2026-11-18-19-19-54-625027-jsnd-44309 (원본 JSND controller_manager) | 7 | 5.464 | 21.163 | 23.046 | 17.853 |
| 11-18 19:27:23 `lap_analyzer_1811_1927.txt` | pp | pp_default | 11_18_5 | ~/.ros/log/2026-11-18-19-27-12-698262-jsnd-47295 (원본 JSND controller_manager) | 5 | 5.955 | 6.292 | 9.322 | 6.967 |
| 11-18 19:30:26 `lap_analyzer_1811_1930.txt` | pp | pp_default | 11_18_5 | ~/.ros/log/2026-11-18-19-30-17-889991-jsnd-48323 (원본 JSND controller_manager) | 16 | 10.360 | 11.328 | 16.127 | 14.760 |
| 11-18 19:36:28 `lap_analyzer_1811_1936.txt` | pp | pp_default | 11_18_5 | ~/.ros/log/2026-11-18-19-36-18-648384-jsnd-52092 (원본 JSND controller_manager) | 10 | 3.242 | 15.795 | 21.630 | 17.178 |
| 11-18 19:41:55 `lap_analyzer_1811_1941.txt` | pp | pp_default | 11_18_5 | ~/.ros/log/2026-11-18-19-41-48-497093-jsnd-55558 (원본 JSND controller_manager) | 19 | 5.385 | 19.865 | 39.187 | 57.073 |
| 11-18 20:01:30 `lap_analyzer_1811_2001.txt` | pp | pp_default | 11_18_6 | ~/.ros/log/2026-11-18-20-01-20-798193-jsnd-62063 (원본 JSND controller_manager) | 18 | 5.447 | 8.084 | 28.234 | 56.260 |
| 11-19 17:14:55 `lap_analyzer_1911_1714.txt` | pp | pp_default | ? | MPC 통합(11-22) 이전 날짜 → 원본 JSND(PP) | 3 | 5.761 | 7.942 | 17.370 | 18.251 |
| 11-20 17:30:46 `lap_analyzer_2011_1730.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-20-17-30-36-235276-jsnd-13072 (원본 JSND controller_manager) | 1 | 19.366 | 19.366 | 19.366 | nan |
| 11-22 12:17:53 `lap_analyzer_2211_1217.txt` | mpc | lowspeed | 11_18_6 | bench_logs/race_mpc_20261122_121742.log (MPC 로그 확인) | 7 | 8.367 | 16.050 | 22.510 | 14.845 |
| 11-22 12:37:18 `lap_analyzer_2211_1237.txt` | mpc | lowspeed | 11_18_6 | bench_logs/race_mpc_20261122_123708.log (MPC 로그 확인) | 1 | 331.355 | 331.355 | 331.355 | nan |
| 11-22 14:40:16 `lap_analyzer_2211_1440.txt` | mpc | 50 | 11_18_6 | bench_logs/race_mpc50_20261122_144007.log (MPC 로그 확인) | 6 | 14.251 | 14.999 | 22.941 | 13.188 |
| 11-22 14:56:16 `lap_analyzer_2211_1456.txt` | mpc | 1p5 | 11_18_6 | bench_logs/race_mpc1p5_20261122_145608.log (MPC 로그 확인) | 5 | 11.705 | 12.784 | 16.120 | 5.312 |
| 11-22 15:20:09 `lap_analyzer_2211_1520.txt` | mpc | lowspeed | 11_18_6 | bench_logs/race_mpc_20261122_151958.log (MPC 로그 확인) | 1 | 28.760 | 28.760 | 28.760 | nan |
| 11-22 15:26:30 `lap_analyzer_2211_1526.txt` | mpc | lowspeed | 11_18_6 | bench_logs/race_mpc_20261122_152621.log (MPC 로그 확인) | 17 | 11.216 | 17.356 | 78.668 | 146.796 |
| 11-23 12:37:29 `lap_analyzer_2311_1237.txt` | mpc | 2p25 | 11_18_6 | bench_logs/race_mpc2p25_20261123_123719.log (MPC 로그 확인) | 7 | 8.380 | 13.888 | 89.609 | 134.749 |
| 11-23 13:21:20 `lap_analyzer_2311_1321.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_20261123_132111.log (MPC 로그 확인) | 1 | 121.826 | 121.826 | 121.826 | nan |
| 11-23 13:42:26 `lap_analyzer_2311_1342.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_20261123_134216.log (MPC 로그 확인) | 5 | 10.608 | 17.337 | 57.184 | 73.792 |
| 11-23 14:37:31 `lap_analyzer_2311_1437.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_20261123_143721.log | 2 | 5.808 | 14.760 | 14.760 | 12.660 |
| 11-23 14:54:07 `lap_analyzer_2311_1454.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_20261123_145358.log | 18 | 7.190 | 7.942 | 13.025 | 14.512 |
| 11-23 15:00:15 `lap_analyzer_2311_1500.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_20261123_150006.log | 10 | 7.740 | 7.923 | 23.763 | 50.131 |
| 11-24 19:03:33 `lap_analyzer_2411_1903.txt` | ? |  | ? | 매칭 launch 로그 없음(시계 미동기 포함) | 2 | 15.014 | 15.048 | 15.048 | 0.049 |
| 11-24 19:06:53 `lap_analyzer_2411_1906.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_integrated_20261124_190644.log (MPC 로그 확인) | 3 | 6.429 | 6.621 | 6.780 | 0.451 |
| 11-24 19:08:42 `lap_analyzer_2411_1908.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_integrated_20261124_190832.log (MPC 로그 확인) | 11 | 4.260 | 16.065 | 118.167 | 240.350 |
| 11-24 19:44:10 `lap_analyzer_2411_1944.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_integrated_20261124_194401.log (MPC 로그 확인) | 62 | 3.098 | 7.108 | 12.908 | 19.195 |
| 11-24 20:00:59 `lap_analyzer_2411_2000.txt` | ? |  | ? | 매칭 launch 로그 없음(시계 미동기 포함) | 1 | 15.160 | 15.160 | 15.160 | nan |
| 11-24 20:03:30 `lap_analyzer_2411_2003.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_integrated_20261124_200320.log (MPC 로그 확인) | 4 | 7.394 | 7.695 | 7.660 | 0.205 |
| 11-24 20:06:06 `lap_analyzer_2411_2006.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_integrated_20261124_200556.log (MPC 로그 확인) | 2 | 7.672 | 8.209 | 8.209 | 0.760 |
| 11-24 20:29:43 `lap_analyzer_2411_2029.txt` | ? |  | ? | 매칭 launch 로그 없음(시계 미동기 포함) | 8 | 15.063 | 15.287 | 75.575 | 108.336 |
| 11-24 20:44:13 `lap_analyzer_2411_2044.txt` | ? |  | ? | 매칭 launch 로그 없음(시계 미동기 포함) | 6 | 14.674 | 15.282 | 15.223 | 0.343 |
| 11-24 20:56:18 `lap_analyzer_2411_2056.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_integrated_20261124_205609.log (MPC 로그 확인) | 3 | 6.297 | 7.177 | 8.133 | 2.459 |
| 11-24 21:13:43 `lap_analyzer_2411_2113.txt` | mpc | 3p0 | 11_18_6 | bench_logs/race_mpc3p0_integrated_20261124_211334.log (MPC 로그 확인) | 12 | 6.822 | 7.630 | 8.999 | 5.422 |
| 11-24 21:17:55 `lap_analyzer_2411_2117.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_integrated_20261124_211745.log (MPC 로그 확인) | 1 | 12.964 | 12.964 | 12.964 | nan |
| 11-24 21:20:37 `lap_analyzer_2411_2120.txt` | mpc | 5p0 | 11_18_6 | bench_logs/race_mpc5p0_integrated_20261124_212027.log (MPC 로그 확인) | 9 | 6.950 | 7.823 | 14.907 | 21.275 |
| 11-25 14:26:53 `lap_analyzer_2511_1426.txt` | mpc | lowspeed | 11_18_6 | bench_logs/step45_mpc_lowspeed_20261125_142646.log (MPC 로그 확인) | 4 | 14.362 | 14.557 | 22.037 | 15.090 |
| 11-25 14:30:37 `lap_analyzer_2511_1430.txt` | mpc | 3p0 | 11_18_6 | bench_logs/step45_mpc_3p0_20261125_143029.log (MPC 로그 확인) | 4 | 6.938 | 7.546 | 8.045 | 1.472 |
| 11-25 14:42:36 `lap_analyzer_2511_1442.txt` | mpc | 3p0 | 11_18_6 | bench_logs/step45_mpc_3p0_20261125_144229.log (MPC 로그 확인) | 5 | 7.002 | 7.240 | 12.472 | 11.915 |
| 11-25 14:45:29 `lap_analyzer_2511_1445.txt` | mpc | 5p0 | 11_18_6 | bench_logs/step45_mpc_5p0_20261125_144520.log (MPC 로그 확인) | 14 | 7.193 | 9.070 | 10.090 | 4.070 |
| 11-25 14:50:15 `lap_analyzer_2511_1450.txt` | pp | pp_default | 11_16_3 | ~/.ros/log/2026-11-25-14-50-07-586460-jsnd-20374 (원본 JSND controller_manager) | 9 | 5.747 | 8.491 | 19.752 | 21.311 |
| 11-25 14:59:54 `lap_analyzer_2511_1459.txt` | mpc | 5p0 | 11_18_6 | ~/.ros/log/2026-11-25-14-59-46-187670-jsnd-21346 (mpc_car.yaml 로드) | 6 | 6.992 | 9.169 | 32.916 | 53.413 |
| 11-25 16:15:05 `lap_analyzer_2511_1615.txt` | mpc | 5p0 | 11_18_6 | bench_logs/step45_mpc_5p0_20261125_161456.log (MPC 로그 확인) | 3 | 7.330 | 15.537 | 12.936 | 4.860 |
| 11-25 16:30:59 `lap_analyzer_2511_1630.txt` | mpc | 5p0 | 11_18_6 | bench_logs/step45_mpc_5p0_20261125_163051.log (MPC 로그 확인) | 12 | 6.712 | 11.498 | 11.782 | 4.351 |
| 11-26 18:13:14 `lap_analyzer_2611_1813.txt` | mpc | 5p0 | 11_18_6 | bench_logs/step45_mpc_5p0_20261126_181303.log (MPC 로그 확인) | 7 | 8.077 | 11.644 | 14.470 | 7.026 |
| 11-26 18:20:06 `lap_analyzer_2611_1820.txt` | mpc | 5p0 | 11_18_6 | bench_logs/step45_mpc_5p0_20261126_181959.log (MPC 로그 확인) | 14 | 6.607 | 8.006 | 12.367 | 8.454 |
| 11-26 18:28:22 `lap_analyzer_2611_1828.txt` | mpc | 7p0 | 11_18_6 | bench_logs/step45_mpc_7p0_20261126_182812.log (MPC 로그 확인) | 46 | 3.359 | 8.933 | 10.256 | 4.966 |
| 11-26 18:38:36 `lap_analyzer_2611_1838.txt` | mpc | 7p0 | 11_18_6 | bench_logs/step45_mpc_7p0_20261126_183827.log (MPC 로그 확인) | 15 | 6.828 | 7.508 | 8.000 | 1.158 |
| 11-26 18:44:19 `lap_analyzer_2611_1844.txt` | mpc | 7p0 | 11_18_6 | bench_logs/step45_mpc_7p0_20261126_184410.log (MPC 로그 확인) | 54 | 5.993 | 6.508 | 12.757 | 24.779 |
| 11-26 19:03:02 `lap_analyzer_2611_1903.txt` | mpc | 7p0 | 11_18_6 | bench_logs/step45_mpc_7p0_20261126_190253.log (MPC 로그 확인) | 21 | 6.053 | 6.373 | 6.967 | 1.929 |
| 11-26 19:26:08 `lap_analyzer_2611_1926.txt` | mpc | lowspeed | 11_18_6 | bench_logs/step45_mpc_lowspeed_20261126_192558.log (MPC 로그 확인) | 10 | 6.439 | 19.872 | 29.748 | 28.331 |
| 11-26 19:56:01 `lap_analyzer_2611_1956.txt` | mpc | lowspeed | 11_18_6 | bench_logs/step45_mpc_lowspeed_20261126_195553.log (MPC 로그 확인) | 11 | 14.331 | 21.694 | 28.185 | 14.302 |
| 11-26 20:03:10 `lap_analyzer_2611_2003.txt` | mpc | 7p0 | 11_18_6 | bench_logs/step45_mpc_7p0_20261126_200302.log (MPC 로그 확인) | 10 | 6.839 | 12.318 | 15.640 | 8.987 |
| 11-26 20:13:32 `lap_analyzer_2611_2013.txt` | mpc | 7p0 | 11_18_6 | bench_logs/step45_mpc_7p0_20261126_201321.log (MPC 로그 확인) | 112 | 3.370 | 12.311 | 19.910 | 20.971 |
