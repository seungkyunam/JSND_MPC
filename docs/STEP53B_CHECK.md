# Step 53b 저속 회피 준비 점검 (2026-11-26, 읽기 전용, 코드 무변경)

실행 스택: race.launch controller:=mpc mpc_speed_source:=integrated mpc_rate_hz:=50, map 11_18_6, JSND_INT 73a6f90. mux 래치(estop), 차량 정지.

## 결과
- `/behavior_strategy` 49 Hz, 전부 OVERTAKE. local_wpnts 70점(s 5.1~12.0), d: +0.406(차 위치) → −0.124 (s≈7.9) → 0 (s≈9.3).
- 배선: `controller_manager.behavior_cb`(local_wpnts → waypoint_array_in_map) → `Controller.integrated_loop` → `lateral_mpc.solve()` (x,y 추종). wpnt vx(2.7~3.2)는 미사용, 상한 = 프로파일 1.0 m/s (predicted_peak 1.0).
- MPC: solved 97% / solved_late 2% / plan_continuation:deadline_exceeded 1~2%, p50 5.9 ms / p99 18.4 ms (주기 20 ms). 진단 조향 −0.137 rad(회피 방향).
- 장애물 s 7.72~8.00, d 0.20~0.24 ~ 0.475 (검출 흔들림 ~0.04 m). 경로 중심→장애물 가장자리 0.29~0.32 m, 차체 측면 여유 ~0.15~0.18 m (ego 0.29). 오른쪽 벽 여유 ≥0.65 m.

## 판단
1.0 m/s 저속 회피 시험 가능. A(estop) 즉시 대기.

## 문제점 / 제안 (모두 미적용)
1. 측면 여유 < evasion_distance 0.35 (사다리 완화). PF 지터 p95 0.08~0.10 m 고려 시 실여유 ~0.05 m. 접촉 위험 관찰 시 ego_width/하한 margin 상향 검토.
2. 통합 MPC 내부에 장애물·벽 제약 없음(경로 추종 + tracking/heading envelope만). 충돌 회피는 플래너 경로 + TTC estop 의존(설계대로).
3. MPC 예측 xy 미발행(`/controller/mpc_diagnostics`는 속도만) → 예측 경로-장애물 여유 직접 측정 불가. 진단에 `result.predicted` x,y 추가 제안.
4. OVERTAKE 중 `overtaking_targets` 비어 있음 — 통합 MPC 미사용이라 영향 없음.

프로브: `step53b_probe.py` (rclpy 5 s 샘플, domain 173).
