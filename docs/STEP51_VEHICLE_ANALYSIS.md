# Step 51 실차 분석 (5p0, 14랩) — 코드 무변경

bag: `~/bench_logs/bags/step45_mpc_20261126_182005` (Jetson 로컬, 256 MiB, 미포함)
로그: `~/bench_logs/step45_mpc_5p0_20261126_181959.log`
랩: 최고 6.607 s(L11), 최악 36.245 s(L14, 수동 개입 포함). 느린 랩 8/12/13/14, 빠른 랩 6/11.

## bag 무결성
- metadata 정상, 22토픽, 294,149 메시지 역직렬화 실패 0, 0.2 s 초과 공백 0.
- `/controller/mpc_diagnostics` 47.5 Hz(목표 50), `/scan` `/imu_corrected` `/pf/pose/odom` 모두 기록.

## 1. 직선 가속 병목 (확인)
- 직선 가속 틱: MPC 최적 가속 p50 2.4 / p95 3.8 m/s², 적용 2.35.
- 명령−측정 p50 = 0.50 m/s, `lead_limited` 62% (jerk 6%, plan 20%) → `mpc_cmd_max_lead_mps` 0.5가 병목.
- 실가속 1.4~1.6 m/s², 최고 휠속도 3.18 m/s(레이싱라인 3.9). 모터 전류 p95 27 A, duty p95 0.19 → 하드웨어 여유.
- 미확인: VESC 속도루프 τ(≈0.35 s 추정, 회귀 불안정).

## 2. 코너 버벅임·정지 (확인)
1. 코너2 진입 s≈8.3~9.2: `braking_recovery` 조향 p50 0.001 rad(65% < 0.02) vs solved 0.128 — 최대 제동 시 마찰원 결합으로 조향 0.
2. 같은 구간 PF yaw가 IMU 대비 0.4~0.6 rad 지연 후 점프(0.7~0.95 m/0.1 s) → `outside_heading_envelope` STOP(느린 랩 75~121틱, 빠른 랩 0~12) → 0.5 m/s 크리프 2~3 s.
3. 정지 후 PF 정체(휠 1.9~2.7 m/s, PF s≈8.9 / 13.3~13.8 고정, IMU 회전 ~2 rad) → TTC EStop ~1 s 간격 4~5회.
4. 헤어핀 출구 s≈3.4~4.9: 프레네 d≈−0.9(매 랩) → close_to_raceline=False + 비차단 정적 장애물(d≈0.83) → `ObstacleTransition`이 TRAILING 반환 → `trailing_without_target` STOP 1.7 s(454틱 전부 s 3~5).
5. Step 51 반 랩 horizon 수정 효과: 반 랩 초과 gap 0건, Step 50의 s≈6.5~7.5 오인 TRAILING 소멸.
- 미확인: PF 지연·정체 근본 원인(스캔-맵 불일치 vs 모션 모델), 실제 추종 경로 대비 횡오차.
- 조향 명령→IMU yaw 응답 지연 0.10 s(상관 0.98).

## 3. 최소 수정 방향(미적용)
1. 헤어핀 TRAILING: `ObstacleTransition`에서 GB 경로 free면 라인 이탈이어도 GB_TRACK, 또는 MPC `trailing_without_target` → GB 추종.
2. `braking_recovery`: 기하 조향(곡률) 유지, 마찰원은 감속을 줄여 맞춤 + 코너2 제동 선행(`mpc_brake_lead_s` 상향).
3. 직선: `mpc_cmd_max_lead_mps` 1.0~1.5, 가속 중·cap 이하에서만.
4. heading STOP: `imu_corrected` yaw rate 단기 보정 또는 PF 점프 시 계획 이어쓰기(PF 원인 확인 후).

분석 스크립트: `jsnd_int/tests_offline/step51_vehicle/` (`ext.py`로 추출 → `a2.py` 랩 집계, `a7.py` 구간 상세, `a8.py` 상태별 조향, `a11.py` TRAILING, `a12.py` 가속 계층).
