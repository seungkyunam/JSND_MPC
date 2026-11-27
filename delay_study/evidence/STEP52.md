# Step 52 — Step 51 실차 분석 기반 수정 (실차 미구동)

JSND_INT racing_stack `d0654e6` (기준 Step 51 배포 `c8620be`). controller / state_machine / stack_master 빌드 완료(symlink).
근거: [STEP51_VEHICLE_ANALYSIS.md](STEP51_VEHICLE_ANALYSIS.md). PF 관련 미확인 원인은 수정하지 않음.

| # | 파일 | 변경 | 끄는 법 |
|---|---|---|---|
| 1 | `Controller.py`, `controller_manager.py`, `mpc_car.yaml` | `mpc_cmd_accel_lead_mps: 1.2` — 상태 `solved`/`solved_late`, 가속>0, 예측 속도열이 horizon 내 감소 없음일 때만 선행 제한 0.5→1.2. 계획 최고속도·max_speed 상한, anti-windup(0.5) 유지. 진단 `lead_mps` 추가 | yaml 0 (노드 재기동) |
| 2 | `integrated_mpc.py` | braking_recovery 감속 바닥을 기준 조향(|delta_ref|, 1단계는 조향속도 한계 잔량 이상)의 횡 몫을 남긴 16각 마찰 다각형으로 계산. caps 상향은 BRAKING_RECOVERY_TOL(0.30) 이내. infeasible 판정·비선형 검증(coupled/lateral/tracking/heading) 무변경 | `BRAKING_RECOVERY_STEER_RESERVE=False` |
| 3 | `state_transitions.py` | `ObstacleTransition`: 라인 이탈 + 복구경로 없음 + 추월 불가 + GB 경로 free일 때 |d|≤1.2 m이고 차~레이싱라인 횡 통로(±gb_ego_width)에 전방 2.5 m/후방 0.5 m 장애물 없으면 TRAILING 대신 LOSTLINE(GB 추종, NonObstacleTransition과 동일) | `GB_RETURN_MAX_D_M=0` |

## 검증 (변경 부분만)
- 코너 내부 제동복구(κ=1, v0 2.2, prev 0.15): 기존 조향 0.087/감속 −4.70 → 0.214/−2.10, 상태 braking_recovery(검증 통과). 직선 구간 제동은 동일(조향 0 정상). `jsnd_int/tests_offline/step51_vehicle/check52.py`.
- 선행 제한: 가속 solved 1.2 적용, 계획 내 감속/braking_recovery/끔은 0.5, 계획 최고속도 상한 유지.
- 복귀 조건: bag 사례(d −0.9, 장애물 d +0.83 전방 1.8 m) 복귀, 통로 내 장애물(전방/후방 0.3 m)·|d| 1.4는 TRAILING 유지.
- pytest test_envelope_recovery/test_mpc_trailing/test_corner_stall: 17 passed, 2 failed(기존 실패, 플래그 OFF에서도 동일).

## 실차 확인 포인트
`/controller/mpc_diagnostics` lead_limited·lead_mps, braking_recovery 조향, s 3~5 trailing_without_target, s≈9 heading STOP, TTC 횟수.
롤백: `git -C ~/JSND_INT/src/racing_stack revert d0654e6` 후 3패키지 빌드.
