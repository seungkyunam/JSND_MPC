# Step 53b 저속 정적 장애물 회피 실차 bag 분석 (2026-11-26, 분석만, 코드 무변경)

대상: `~/bench_logs/bags/step45_mpc_20261126_193127` (분석 시점 녹화 중 → 사본 reindex). 자율 6.75~82.23 s(75.5 s), 82.24 s humandrive. 참고: `192651`(자율 30 s). JSND_INT `73a6f90`, 상한 1.0 m/s.

## 요약
| 항목 | 주행 | 사전점검(STEP53B_CHECK, 정지) |
|---|---|---|
| MPC solved | 85.8% (+late 0.8%) | 97% |
| 계산 p50/p99/max | 5.9/17.1/29.7 ms (>20 ms 14틱) | p99 18.4 ms |
| 주기 | 48.6 Hz (p99 35, max 53 ms) | 50 Hz |
| 랩 | 21.7 / 14.7 / 20.1 s (이상치 ≈14.6 s) | – |

## 회피 흐름
- OVERTAKE 중 대부분 solved, 헤어핀(s≈3~4) 회피에서 실제 d −1.1~−1.4로 이동 → 회피 경로는 MPC에 반영됨.
- PF↔local_wpnts 최근접: OVERTAKE p50 0.18/p95 0.48 m, GB p50 0.14 m (기준점 차이 포함 가능).
- 복귀 GB_TRACK: s≈6.3(2회), 4.6, 5.3(TRAILING 경유). OVERTAKE↔TRAILING 0.02~0.3 s 채터링 다수.
- 정적 장애물 검출이 s≈1~2.6, 2.6~4.5, 5~6.3, 8, 11, 13로 흩어짐, s≈3~4 묶음은 d −1.4~+0.14로 흔들림. 실제 배치 기록 없어 정·오검출 판정 불가. 사전점검 위치 s≈7.86은 4랩 모두 GB_TRACK, 통과 중 검출 0회.

## 여유 거리
- LiDAR 원점 최소 0.099 m (24.4~28.6 s, s≈3.1~3.4, 좌측 +65°). <0.25 m: s≈1.5~1.6 전방/우 0.13~0.16 m(76.6~77.5 s 전방 0.14 m에서 휠 구동), s≈7.4~8.0 좌측 ~90° 0.12~0.23 m(매 랩, GB_TRACK).
- 차체 외곽 여유는 측정 불가(LiDAR 오프셋·외형 미기록). 0.10~0.14 m < 반폭 0.145 → 접촉 여부 육안 확인 필요. 59.2 s 휠 0.14 m/s·60 A(가능성만).
- frenet 여유는 비신뢰(192651 s≈8에서 frenet 겹침인데 LiDAR <0.25 m 없음).

## 속도·조향
- 명령 최대 1.0, vs p50 0.93/max 1.11, vs<0.1 비율 17%.
- 조향 포화(≥0.395): OVERTAKE 18.7%, TRAILING 8.3%, GB 0%. 변화율 p95 3.26 rad/s(설정 3.2).

## MPC 실패 (자율 3671틱)
heading envelope 계열 306(8.3%: outside 200, recovery 62, nonlinear 44), trailing_without_target 79, braking_recovery 50, outside_tracking 36, deadline 9 + plan_continuation 4, OSQP 7(time limit 3, inaccurate 2, primal infeasible 2).

## 정지 원인 (bag 확인분)
1. s≈3.4, 24.9~28.2 s(3.2 s): 회피 중 heading envelope STOP → 0.5 크리프(−0.4 포화) → TRAILING(앞 0.2 m 같은 d blocked) → envelope 유지로 2.8 s 0 → 27.6 s OVERTAKE 재출발. LiDAR 0.099 m 시점.
2. s≈1.4/1.64, 19.9 s·57.5~59.3 s: TRAILING blocked → braking_recovery/heading STOP. 직전 PF 0.5 m 역방향 점프.
3. s≈5.3~6.0, 65.4~68.1 s: trailing_without_target 2회 정지(장애물 d 0.58/0.80, 차 d +0.5 같은 쪽) — Step 53b 수정 범위 밖.
4. s≈12.7, 37.9~39.2 s: `dyn/nopred (id_mismatch)` blocked, PREDICTION_STALE, GB↔TRAILING 교대.
5. s≈3.5, 45~46 s: d −1.1~−1.3에서 GB_TRACK 전환 → outside_tracking_envelope.
6. s≈0.8~1.6, 76~82 s: OSQP primal infeasible/time limit, PF 0.76 m 역점프, dyn/pred blocked → 수동 전환.

## PF
- 39.9 Hz, 간격 max 65 ms. 휠 대비 초과 이동 >0.10 m 40회, 역방향 0.5~0.76 m(s≈1.2~1.8: 20.2/56.2/76.3 s), 헤어핀 0.44 m(29.4 s). 정지 중 0.45 m 드리프트(78.7~80.2 s).
- vs /car_state/odom p50 0.034/p95 0.19/max 0.71 m.
- 지연 측정 불가: `/pf/pose/odom` 스탬프가 스캔 시각 아님(일치 0.1%). 헤더→도착 p50 1/max 13 ms는 전송 지연.

## 개선 우선순위 (미적용)
1. [안전] s≈3.1~3.4 좌측·s≈1.5 전방 접촉 흔적 확인, 장애물 실제 위치 기록.
2. [정지] TRAILING 중 heading envelope 고착(크리프 복귀가 OVERTAKE만 동작 추정) + "앞 0.2 m 같은 d" blocked 판정 실재 여부.
3. [PF] s≈1.2~1.8 역점프·정지 드리프트(장애물 가림/맵 불일치는 가설). envelope STOP과 시점 일치.
4. [상태머신] s≈5.3~6.0 같은 쪽 라인 밖 장애물 trailing_without_target, s≈12.7 id_mismatch 오인.
5. [관측] MPC 예측 xy 발행, PF 스캔 스탬프, LiDAR→base_link 오프셋 기록.
6. [성능] OVERTAKE 포화 18.7%, OSQP 7건, >20 ms 14틱(정지 주원인 아님).

스크립트: `jsnd_int/run/step53b_bag/` (ext.py로 pickle 추출 → an/ep/win/pf/cl/s8/trk.py, gw.py는 global_waypoints).
