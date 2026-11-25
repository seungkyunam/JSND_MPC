# Step 47 — 직선 진동·코너 진입/탈출·고속 프로파일 (2026-11-25)

JSND_INT racing_stack: 기준 `e16d3ef`(Step 45 배포 상태 커밋) → `4ca035e`(Step 47). controller만 재빌드, stack_master는 yaml 심링크라 재빌드 불필요. 실차 미구동.
백업: `~/JSND_backups/step47_20261125_143604` (src tgz, install tgz, Step 45 미커밋 diff, HEAD).
패치: `jsnd_int/0015-step47-corner-entry-exit-r-rate.patch`, 변경 파일 사본: `jsnd_int/racing_stack/...`.

## 변경 (통합 MPC `mpc_speed_source:=integrated`)
| 항목 | 이전 | Step 47 | 위치 |
|---|---|---|---|
| 직선 조향 진동: `mpc_r_rate` | 2.0 | 3.0 (jsnd 조향 MPC도 같은 키 사용) | mpc_car.yaml |
| 코너 선행 감속: `mpc_brake_lead_s` | 없음(0) | 0.12 — 곡률 속도 상한을 0.12·v m 앞에서부터 적용(VESC 속도 루프 지연 보상). 상한을 **낮추기만** 함 | integrated_mpc `_geometry_caps` |
| 조향 타이밍: `mpc_steer_preview_s` | 없음(0) | 0.03 — 조향 선형화·피드포워드 기준을 0.03·v m 앞 곡률로 | integrated_mpc `_reference` |
| 코너 탈출 가속: `mpc_accel_jerk_mps3` | 없음(0) | 20 — 양의 가속만 틱당 jerk·dt씩 증가(0→5 m/s² 0.25 s). 감속은 제한 없음 | Controller `_mpc_speed_command` |
| 0 = 끔 허용 | — | `MPCConfig` 양수 검사에서 위 두 키만 제외(`ZERO_ALLOWED`) | lateral_mpc.py |

유지: max_accel/decel 5.0, 횡가속 4.5·0.85 계획 여유, 조향 0.4 rad / 3.2 rad/s, 제동 envelope(종단 0 포함), 소프트 STOP, TTC/simple_mux A 래치, VESC speed_max 23250 ERPM(≈5.04 m/s), `mpc_heading_robust`/`mpc_history_prediction` 기본 false.

## 5p0/6p0/7p0 적용 확인 (install yaml을 race.launch 순서로 병합해 실제 `make_config` 생성)
```
5p0 max_speed 5.0 r_rate 3.0 brake_lead 0.12 steer_preview 0.03 jerk 20.0 heading_robust False history False
6p0 max_speed 6.0 ...
7p0 max_speed 7.0 ...
```
- 7.0 m/s 설정 자체는 가능(별도 상한 없음). 다만 실제 바퀴 속도는 VESC speed_max로 ≈5.04 m/s에서 포화(물리 제약, 무변경). 국부 경로 창(~8 m, 끝점 정지 예약) 때문에 통합 MPC 상한도 약 6 m/s 이하.
- 11_18_6은 레이싱라인 최고 3.87 m/s라 5p0 이상 상한은 이 트랙에서 거의 걸리지 않음.

## 최소 점검 (오프라인 lap_sim, 7p0 integrated, 2랩, 실제 deadline, 잡음 2 cm/0.05 m/s, 각 1회)
| | 플라잉랩 | 최대 횡오차 | ay max | MPC p99 | STOP |
|---|---|---|---|---|---|
| Step 46 설정 | 5.24 s | 0.053 m | 4.46 | 9.07 ms | 0 |
| Step 47 | 5.36 s | 0.048 m | 4.33 | 6.72 ms | 0 |
선행 감속으로 랩 +0.12 s(약 2%), 횡오차·횡가속 피크·계산시간 감소. 직선 진동은 실차 잡음 현상이라 시뮬로 판정하지 않음 → 실차 bag으로 확인. lap_sim은 Controller 저크 제한을 거치지 않음.

## 실행 (Jetson, foreground)
```
bash ~/JSND_MPC_v3/jsnd_int/run/step45_run.sh mpc 11_18_6 5p0     # 6p0 / 7p0 동일
bash ~/JSND_INT/run/step45_rec.sh mpc                               # 녹화(선택, 별도 터미널)
```

## 복구
- 파라미터만 되돌리기(재빌드 불필요, 노드 재기동): mpc_car.yaml에서 `mpc_r_rate: 2.0`, `mpc_brake_lead_s: 0.0`, `mpc_steer_preview_s: 0.0`, `mpc_accel_jerk_mps3: 0.0`
- 코드까지 Step 45/46 상태로:
```
cd ~/JSND_INT/src/racing_stack && git revert --no-edit 4ca035e
cd ~/JSND_INT && source /opt/ros/jazzy/setup.bash && source ~/JSND/install/local_setup.bash && colcon build --base-paths src --packages-select controller
```
- 백업 원본: `tar xzf ~/JSND_backups/step47_20261125_143604/racing_stack_src.tgz -C ~/JSND_INT/src/racing_stack` 후 위 빌드
