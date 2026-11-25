# Step 48 — 통합 MPC 병목 수정 (2026-11-25)

JSND_INT `0f5e086` (기준 `4ca035e` = Step 47). controller만 빌드. 실차 미구동.

## 입력 데이터
- 실차 로그 `step45_mpc_5p0_20261125_144520.log`(통합 MPC, Step 47 코드): 랩 7.19~9.6 s,
  STOP: prediction heading 58, deadline 46, osqp 시간한도/inaccurate 73, braking envelope 27, tracking 31.
- bag `step45_improved_20261125_142036`(11_18_6, 14:30 통합 3p0 구간 포함), `5p0_20261124_205612`.
- 14:50 PP 런은 **맵 11_16_3**(15.8 m, κmax 0.80, 섹터 1.0)이라 11_18_6(13.9 m, κmax 1.11) MPC와 랩 비교 불가.
  같은 맵 PP 기준은 11-18 20:01 실차 5.45~6.0 s.
- 15:00 런은 jsnd 속도 소스(PP 속도 + 조향 MPC) + 11_18_6 speed_scaling 0.6 코너 섹터 → 느림의 원인은 섹터 0.6(통합 MPC에는 미적용). 사용자 지시로 이 방향은 중단.

## 병목과 수정
| 병목 | 근거 | 수정 |
|---|---|---|
| 직선 가속 부족 | 오프라인 직선 s5~7 MPC 가속 +1.0/+0.1 m/s² (마찰 여유 충분, ay 1.6) → a² 비용이 위치·진행 보상과 상쇄 | `mpc_r_accel` 0.15→0.05 |
| 계산 실패(시간한도/inaccurate/deadline) | OSQP eps 1e-5, 반복 p95 230~1000, 최대 1400 | eps 1e-4, adaptive rho 10; 행 검사 허용 2e-3 (비선형 검증은 그대로) |
| 계산 실패→STOP→감속 램프 | 연속계획 속도 허용 0.30 < 실차 명령-측정 p90 0.48 | 연속계획 속도 허용 0.50 (cap·횡가속·마찰·추종은 측정 상태로 재검증) |
| 제동 엔벨로프 STOP | 도달 위치에서 <2 mm/s 초과(QP 잔차)로 STOP | 속도 검증 허용 2 mm/s, 0.30 m/s 이내 초과는 braking_recovery(최대 제동+추종) |
| 코너 탈출 좌우 진동 | 실차 14:30 bag exit: 조향 −0.31~+0.40, d ±0.4 m, 주기 ~1 s | `mpc_q_heading` 8→16 |

안전장치·물리 제약 무변경: max_steer 0.4, 조향속도 3.2, accel/decel 5, ay 4.5(계획 0.85), 추종 1.0 m/heading 0.7 rad 엔벨로프, soft STOP, TTC/estop, VESC 상한.

## 오프라인 폐루프(lap_sim, 11_18_6, 5p0, 서보 지연 60 ms, 자세 잡음 2 cm)
| 조건 | 전 | 후 |
|---|---|---|
| 기본: 랩 / 직선 최고 / STOP / p99 / 반복 max | 5.38 s / 3.30 / 0 / 7.0 ms / 1414 | 5.22 s / 3.69 / 0 / 5.7 ms / 700 |
| 자세 지연 140 ms + 헤어핀 자세 편향 0.15 m: 랩 / STOP / 횡오차 max | 5.74 / 4 / 0.55 m | 5.48 / 2 / 0.33 m |
| 위 + 예산 절반(4/8 ms): 랩 / STOP | 5.90 / 9 | 5.68 / 8 |

테스트: test_corner_stall + test_envelope_recovery 8 passed / 2 failed — 수정 전과 동일한 기존 실패.

## 맵 11_18_6 품질
- 레이싱라인: 간격 0.099 m 균일, json κ = 기하 κ (RMS 0.002), 코너 탈출부 곡률 부호 반전 없음, dκ/ds max 0.72 (s=3.58) → 정상.
- 스캔-맵 정합(PF 자세, laser x=0.27): 직선 잔차 중앙값 0.00 m, 헤어핀 s 3.0~3.7 0.11~0.16 m, s 8.5 0.10 m → 헤어핀 국부 불일치(이전 PF 정체 s≈2.83과 일치).
- 결론: 진동의 주원인은 제어기(지연 대비 감쇠 부족), 헤어핀 PF 오차가 유발 요인. 재맵핑은 필수 아님. 필요 시 새 이름으로만 생성.

## 실행 / 복구
```
bash ~/JSND_MPC_v3/jsnd_int/run/step45_run.sh mpc 11_18_6 5p0
# 복구(가중치만): mpc_car.yaml 의 mpc_q_heading 8.0, mpc_r_accel 줄 삭제 → 노드 재기동(심링크, 재빌드 불필요)
# 전체 복구: cd ~/JSND_INT/src/racing_stack && git revert --no-edit 0f5e086 && cd ~/JSND_INT && colcon build --base-paths src --packages-select controller
```
