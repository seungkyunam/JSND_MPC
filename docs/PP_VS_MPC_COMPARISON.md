# PP vs Integrated LTV-MPC 비교 (2026-11-26, 기존 기록만 사용)

이 문서는 실차에서 이미 기록된 데이터만 사용한다. 새 주행이나 추정값은 없다.
- 표기: **미측정** = 해당 데이터 자체가 없음, **미확인** = 데이터는 있지만 조건이나 분류를 확정할 수 없음.
- 자동 생성 원본: `docs/results/pp_vs_mpc_existing_20261126/` (`runs.csv`, `laps.csv`, `laps_lap_analyser_txt.csv`, `summary.md`, PNG)
- 도구: `jsnd_int/compare_pp_mpc/` (Jetson 경로 `~/JSND_INT/compare_pp_mpc/`). 기존 코드와 파라미터는 바꾸지 않았다.

## 1. 데이터 현황
| 구분 | PP | MPC |
|---|---|---|
| rosbag | **없음 (0개)** | 10개 (`~/bench_logs/bags`, 11-24~11-26) |
| lap_analyser 랩 기록 (11_18_6) | 1세션 18랩 (11-18 20:01, 원본 JSND) | 다수 세션 |
| 같은 날·같은 장애물 배치 기록 | 없음 | 장애물 실제 배치 기록 없음 |

→ **같은 조건(속도 상한·장애물 배치)에서 비교한 기록은 없다.** 아래 1:1로 놓을 수 있는 항목은 11_18_6 맵의 랩타임뿐이며, 이것도 조건 일치 여부는 미확인이다.

## 2. 비교 가능한 수치 (11_18_6)
| 항목 | PP (11-18 20:01, lap_analyser) | MPC 7p0 (bag 11-26 18:47, Step 52 코드 a5e7044) |
|---|---|---|
| 랩 수 | 18 (수동·정지 랩 포함, 구분 불가) | 14 (전 구간 autodrive인 랩만) |
| 최고 랩 | 5.447 s | 6.197 s |
| 중앙값 | 8.084 s (정지·수동 랩 포함) | 6.437 s |
| 평균 / 표준편차 | 미확인 (28.2 / 56.3 s, 118 s 랩 포함으로 왜곡) | 6.508 s / 0.265 s |
| 속도 상한 | PP 기본(레이싱라인 × 섹터 0.6/1.0) | 프로파일 7.0 (VESC 5.04 m/s에서 포화), 측정 최고 3.08 m/s |
| 장애물 배치 | 미확인 | 미확인 (정적 장애물 검출 있음) |
| 랩타임 산출 | lap_analyser 결승선 간격 | Frenet s 랩 경계 (autodrive 99% 이상인 랩) |

해석상 주의: 최고 랩 기준으로는 PP가 0.75 s 빠르다. 다만 속도 상한, 장애물, 랩 판정 방식이 다르므로 이 차이를 성능 차이로 단정할 수 없다(미확인).

## 3. MPC 단독 측정값 (bag, PP 대응값 없음 → PP 열은 미측정)
| 항목 | MPC lowspeed 1.0 (19:31, 회피, 3a2ba8f) | MPC 5p0 (18:20, 7ec7263) | MPC 7p0 (18:47, a5e7044) | PP |
|---|---|---|---|---|
| 최고 / 평균 랩 [s] | 14.675 / 18.828 | 6.612 / 10.551 | 6.197 / 6.508 | 미측정 |
| 랩 표준편차 [s] | 3.683 | 5.128 | 0.265 | 미측정 |
| 횡오차 RMSE 전체 / GB_TRACK [m] | 0.526 / 0.289 | 0.468 / 0.382 | 0.418 / 0.350 | 미측정 |
| 평균 / 최고 속도 [m/s] | 0.834 / 0.985 | 1.82 / 3.207 | 2.141 / 3.082 | 미측정 |
| 조향 RMS [rad] / 변화율 RMS [rad/s] | 0.208 / 2.195 | 0.174 / 5.292 | 0.178 / 2.614 | 미측정 |
| 가속도 RMS [m/s²] / Jerk RMS [m/s³] | 0.568 / 3.52 | 1.132 / 5.86 | 1.131 / 6.41 | 미측정 |
| 계산시간 p50 / p99 [ms] | 5.91 / 17.08 | 5.38 / 22.28 | 7.62 / 23.50 | 미측정 (PP 모드 `measure` 꺼짐) |
| 실제 제어 주파수 [Hz] (목표 50) | 48.63 | 47.34 | 46.58 | 미측정 |
| MPC solved [%] | 86.6 | 64.2 | 84.5 | 해당 없음 |
| 정지 횟수 (≥0.5 s) | 8 | 8 | 0 | 미측정 |
| 최소 LiDAR 거리 [m] (센서 원점 기준) | 0.099 | 0.134 | 0.347 | 미측정 |
| 조향 포화 (한계 0.4) [%] | 11.49 | 7.72 | 1.48 | 미측정 |
| 장애물 회피 성공률 | 미확인 (배치 기록 없음) | 미확인 | 미확인 | 미측정 |

- 정의는 `jsnd_int/compare_pp_mpc/cmp_analyze.py` 머리말에 있다. 횡오차는 `/car_state/odom_frenet` d(레이싱라인 기준)이며, 회피 중에는 의도적으로 라인을 벗어나므로 GB_TRACK 값을 같이 표기했다.
- 기존 문서와 대조: 7p0 14랩 6.20~7.08 s, 평균 6.51, σ 0.265는 STEP53.md와 같다. 19:31 랩 21.7/14.7/20.1 s는 STEP53B_RUN.md와 같다.
- 컨트롤러 미확인 bag: `3p0_20261124_203250`, `lowspeed_20261124_204417`. 근거가 되는 launch 로그가 없어 분류하지 않았다(runs.csv에서 controller 칸이 비어 있음).
- 최종 Step 56 코드로 주행한 런(20:13, 7p0)은 bag이 없어 표에 넣지 않았다.

## 4. 공정 비교 실험 방법 (아직 실행하지 않음)
조건: 같은 차량, 맵 `11_18_6`, 같은 배터리 상태, `layouts.yaml`에 실측 배치를 적은 같은 장애물 layout, 같은 속도 조건 라벨(cap).
```bash
# 1) 스택 실행 (기존 스크립트, foreground)
bash ~/JSND_MPC_step51/jsnd_int/run/step45_run.sh pp  11_18_6 lowspeed   # PP
bash ~/JSND_MPC_step51/jsnd_int/run/step45_run.sh mpc 11_18_6 lowspeed   # MPC (1.0 m/s 상한)
# 2) PP 속도를 MPC 상한에 맞춤 (실행 중 param만 변경, 코드·파일 무변경. PP 런에서만)
bash ~/JSND_INT/compare_pp_mpc/cmp_pp_speed.sh 1.0
# 3) 녹화 (별도 창, 조건 meta.json 자동 기록)
bash ~/JSND_INT/compare_pp_mpc/cmp_rec.sh pp  cap1p0 L1
bash ~/JSND_INT/compare_pp_mpc/cmp_rec.sh mpc cap1p0 L1
#    정지: tmux send-keys -t rec C-c; sleep 2; tmux kill-session -t rec
# 4) 분석 (CSV + summary.md + PNG 자동 생성)
bash ~/JSND_INT/compare_pp_mpc/cmp_analyze.sh --bags ~/bench_logs/bags/cmp_pp_cap1p0_L1_* ~/bench_logs/bags/cmp_mpc_cap1p0_L1_* \
     --out ~/bench_logs/compare_pp_mpc/cap1p0_L1
```
남은 한계:
- PP 속도 조절은 속도 프로파일 전체를 비율로 줄이는 방식이고, MPC는 상한에서 자르는 방식이라 같은 상한이어도 의미가 다르다. 실측 최고속도를 표에 함께 적는다.
- PP 계산 시간은 현재 launch에서 `measure`가 꺼져 있어 측정되지 않는다. 켜려면 launch 수정이 필요하다(승인 필요, 이번에는 바꾸지 않음).
- 최소 거리는 LiDAR 원점 기준이다. 차체 외곽 오프셋은 측정된 적이 없다.
