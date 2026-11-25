# Step 50 — Step 49 실차 주행 분석 (2026-11-25, 코드 무변경)

- 분석 대상: Step 49 `step45_mpc_5p0_20261125_163051.log` (checkout `1cd9736`, JSND_INT `89e1011`, 5p0, 자율 141 s, 12랩)
- 비교 대상: Step 48 `step45_mpc_5p0_20261125_161456.log` (`fbeeefd`, 자율 31 s, 3랩). 15:49 런은 이벤트 없음(9.6 s), 15:47 런은 OSQP NONCVX 기동 실패라 제외.
- 두 런 모두 **bag 없음**. 로그 사본: `docs/logs/step49_run_*.log.txt`, `docs/logs/step48_run_*.log.txt`, 랩 기록 `docs/logs/lap_analyzer_step48_1615_step49_1630.txt`.
- 분석 스크립트: `jsnd_int/tests_offline/step50/tl.py <log>`(RB 기준 이벤트 타임라인), `met.py <log>...`(분당 지표).
- 목표 속도 = 레이싱라인 `global_traj_wpnts_iqp.vx_mps`(11_18_6). 설정점/실제 = VESC `Current braking engaged/released: target/actual` 로그와 state_machine `[STATIC_OT] speed=`. **가속도 명령은 로그에 없음.** s 위치는 시간으로 추정.

## 1. 랩 시간이 둘로 갈린다
| 구분 | 랩(lap_analyser) | 랩 시간 | 코너2 크리프→정지 |
|---|---|---|---|
| 느린 랩 | #1~6, #10, #11 | 11.3~16.1 s (#10 21.7 s) | 있음(랩 시작 후 5.5~6.2 s) |
| 빠른 랩 | #7, #8, #9, #12 | 6.71~7.66 s | 없음 |

#10에는 별도로 동적 대상(id 40/63/68, gap 최소 0.27 m)이 있었다. 10랩 평균 횡오차 0.37 m / 최대 0.91 m.

## 2. STOP 발생 과정 (느린 랩마다 동일)
1. s≈1.2: s≈4.1 RIGHT 정적 장애물이 매 랩 **새 id**로 검출 → TRAILING 상한으로 `braking_recovery`, 설정점 2.2→1.8 m/s. 이후 OVERTAKE(회피 스플라인)로 통과.
2. s≈6.5~7.5(코너2 진입, 실제 3.06~3.13 m/s): `OVERTAKE`→`TRAILING` (`reason=NO_STATIC_PATH`, `FREE False`).
3. 같은 순간 MPC `braking_envelope_infeasible` + `outside_heading_envelope` STOP → 소프트 STOP 감속 → `envelope_recovery` 0.5 m/s 크리프 약 2.9 s.
4. GB_TRACK/TRAILING 번갈아 → `trailing_without_target` → 설정점 0, 1~2 s 정지 후 재출발.
5. s≈12(코너2 탈출): `TTC EStop`(simple_mux) 설정점 0이 30~200 ms씩 2~4회 → VESC 전류제동, −0.3~0.9 m/s.

2번 TRAILING 원인은 **추정**: 방금 지난 s≈3.1~4.1 장애물이 트랙 래핑(13.9 m)으로 약 10~11 m 앞으로 재계산됨. 관심 거리 상한이 track_length−2 = 11.9 m라 범위 안. 장애물 로그는 4 m 이내만 찍혀 확정 불가(단계 21 래핑 문제의 잔존 가능성).

## 3. 구간별 목표/설정점/실제 (Step 49)
| 구간 | s [m] | 목표 vx | MPC 설정점 | 실제 |
|---|---|---|---|---|
| 메인 직선→코너1 진입 | 0.4~1.5 | 3.87→2.9 | 2.16~2.25→1.77~1.84 | 2.36~2.63→1.8 |
| 코너1 | 3~4 | 2.0~2.2 | 1.73 | 1.72~1.92 |
| 직선2 끝→코너2 진입 | 6.5~7.5 | 3.84→3.3 | 2.65~2.86 후 STOP | 최고 2.84~3.13 |
| 코너2(느린 랩) | 8.5~11 | 2.25~2.6 | 0.5 크리프→0 | 0~0.5 |
| 코너2 탈출 | ≈12 | 2.5~3.0 | 2.0~2.96 ↔ 0(TTC) | 1.9~2.9→1.5~1.7 |
| 메인 직선 | 12.5→0.4 | 3.0→3.87 | 2.0~2.35 | 1.6→2.4~2.6 |

메인 직선 실제 가속도 중앙값 1.09 m/s²(12랩, Step 48 1.06). 레이싱라인 요구 1.3~2.25, `max_accel` 5.0. 가속 중 제동 해제 순간 설정점이 실제보다 0.4~0.8 m/s 앞섬.

## 4. Step 48 대비 (분당)
| 항목 | Step 48 | Step 49 | 판정 |
|---|---|---|---|
| STOP 요청 | 78.7 | 34.6 | 개선 |
| STOP 묶음(0.5 s 간격) | 25.0 | 17.1 | 개선 |
| 계산 계열 STOP(deadline, osqp) | 38.5 | 9.0 | 개선 |
| `invalid_or_reverse_speed` | 7회 | 0회 | 개선 |
| VESC 설정점 0 제동 | 44.2 | 19.2 | 개선 |
| VESC 제동 전체 | 59.5 | 34.6 | 개선 |
| TTC EStop | 13.4 | 7.3 (12랩 17회) | 개선, 매 랩 잔존 |
| 최고 랩 | 7.33 s | 6.71 s | 개선 |
| 코너2 크리프·정지 랩 | 2/3 (67%) | 8/12 (67%) | 동일 |
| 랩 시간 중앙값 | 11.4 s | 11.5 s | 동일 |
| heading 계열 STOP | 13.5 | 11.9 | 거의 동일 |
| `braking_envelope_infeasible` | 3.8 | 5.1 | 악화 |
| `trailing_without_target` | 5.8 | 7.3 | 악화(대부분 ≤0.5 m/s라 소프트 감속 무효) |
| 관측 최고속도 | 3.27 | 3.13 m/s | 개선 없음 |
| 메인 직선 실가속 | 1.06 | 1.09 m/s² | 개선 없음 |
| 비영 제동 초과량(실제−설정점) | 0.20 | 0.19 m/s | 동일 |

## 5. 증상별 원인
1. **코너 진입 순간 정지**: 2절. MPC 파라미터가 아니라 고속 OVERTAKE→TRAILING 전환(상태머신)과 그 순간의 MPC STOP. 래핑 부분은 추정.
2. **불연속 가감속**: TTC EStop(s≈12, 거의 매 랩), s≈1.2 정적 장애물 TRAILING 상한, 크리프→0→재출발 반복.
3. **직선 가속 부족(추정)**: 설정점 ≤ 측정+0.5 m/s(`mpc_cmd_max_lead_mps`)가 병목으로 보임. 이 상한이 걸리면 Step 48 `r_accel` 0.15→0.05가 효과를 못 낸다(48/49 실가속 동일과 일치). 가속도 명령 미기록으로 확정 못 함.

## 6. 확정에 필요한 데이터
s≈7 TRAILING 대상·gap, STOP 순간 heading 오차, 가속도 명령, s≈4.1 장애물의 실체(`/scan`). 다음 런은 `jsnd_int/run/step45_rec.sh mpc`로 bag 기록.
