# Step 46 — Step 45 직선 조향 진동 진단 (2026-11-25)

코드·파라미터 변경 없음. 실차 bag 비교만 수행.

## 비교 (lowspeed 프로파일, 자율 주행 구간)

| 지표 | 기존 MPC (11-24 20:44) | Step 45 improved (11-25 14:20) |
|---|---|---|
| 조향 std | 0.068 rad | 0.284 rad |
| 조향 ±0.39 포화 비율 | 0.0 % | 40.7 % |
| 조향 진동 | 2.6 Hz, rms 0.024 | 1.4 Hz, rms 0.083 |
| heading envelope STOP/복구 | 0 틱 | 198 틱 |
| 명령 yaw rate ↔ EKF yaw rate 상관 | 0.88 | 0.24 |
| EKF–PF yaw 차이 p95 | 0.080 rad | 0.149 rad |
| 목표 속도 std | 0.10 | 0.23 |

## 원인

1. `mpc_heading_robust`(방향각 보정)가 원인이다. 보정은 `/car_state/odom` angular.z(EKF yaw rate)로
   heading을 적분하고 혁신을 1.7 rad/s로 제한한다. 이 bag에서 EKF angular.z와 EKF yaw 미분의 상관은 0.29, 크기 비율은 0.59로
   믿을 수 없는 값이다(단계 38 관찰과 같다). 로그 odom에 보정 로직을 다시 적용하면 보정 heading과 실제 heading의 차이가
   p95 0.18 rad, 최대 0.58 rad이고, 4823개 샘플 중 260개에서 제한이 걸렸다. 결과적으로 heading이 늦고 틀린 값으로 들어가고,
   MPC가 과보정해 조향이 포화되는 bang-bang 진동이 생긴다. 진동으로 실제 yaw rate가 커지면 제한이 더 자주 걸려 진동이 스스로 커진다.
2. `mpc_history_prediction`은 0.1 s 수송지연 예측으로, 기존 `latency_comp`와 차이가 작아 주원인이 아니다.
   manager가 heading 보정 단독 사용을 금지하므로 두 기능은 함께 끈다.
3. 가감속 버벅임은 진동의 2차 결과다. 큰 횡오차(|d| 최대 0.75 m) → heading envelope STOP과 0.5 m/s PP 복구 크리프 →
   정지 중 `invalid_or_reverse_speed` 발생, 그리고 상태머신 TRAILING 진입 후 `trailing_without_target` STOP이 이어진다.
   기존 MPC 주행에는 이 상태가 없었으므로 별도 수정은 하지 않는다.

## 조치

- `step45_run.sh mpc`(heading_robust=false, history_prediction=false)로 주행한다. 재빌드는 필요 없다.
- `improved` 모드는 사용하지 않는다. EKF yaw rate의 신뢰성이 확보되기 전에는 재검토하지 않는다.
- q_position/q_heading/r_rate는 그대로 둔다. 기존 MPC에서 포화 0 %로 직선이 안정적이었으므로 근거 없이 바꾸지 않는다.
