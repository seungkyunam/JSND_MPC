# NEXT_ACTION — 바퀴 띄운 실차 구동 시험 (bench)

**모터 구동은 사용자 확인 후 시작. 아래 0단계가 모두 OK가 아니면 진행하지 않는다.**

현재 상태: GitHub `a8f7524` 코드 = Jetson 실행 코드(해시 검증 완료). **사용자 GitHub 코드 검토 대기** → 승인 후 0단계부터.

## 미확인 값 (이 시험의 전제)
| 항목 | 현재 값 | 상태 |
|---|---|---|
| VESC 최소 ERPM | 가정 900 (923 ERPM에서 회전 확인) | **미확인** |
| MPC 조향 한계 / 변화율 | 0.15 rad / 0.3 rad/s | 보수값, **실측 아님** |
| servo 변환 | gain −0.99, offset 0.487 | 캘리브값, **기계 한계 미측정** |
| MPC 최고속도 | 1.0 m/s (≈4614 ERPM) | 설정값 |

## 0. 물리 안전 확인 (모터 전원 투입 전)
1. 차체를 스탠드에 고정, **4바퀴 모두 공중**, 바퀴 주변 손·케이블 없음
2. 배터리 스위치/커넥터를 즉시 끊을 담당자 1명 배치 (전용 kill 스위치 없음)
3. 게임패드 담당 1명: **LB를 누르면 수동(스틱 중립 = 0)**. A 버튼은 JSND에서 정지 기능 없음
4. 차량 전방 1 m 비움 (TTC estop 오작동 방지)
5. 차량을 **맵 위 raceline 근처(1 m 이내), 진행 방향으로** 놓는다 — 아니면 MPC가 STOP(정상)

## 1. 실행 (Jetson, ROS_DOMAIN_ID=173)
터미널 1:
```bash
source /opt/ros/jazzy/setup.bash && source ~/JSND_INT/install/setup.bash
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc use_estop:=true
```
(롤백: `controller:=pp` 또는 `source ~/JSND/install/setup.bash`)

터미널 2 (모니터):
```bash
source /opt/ros/jazzy/setup.bash && source ~/JSND_INT/install/setup.bash
ros2 topic echo /simple_mux/active_host --field data
ros2 topic echo /controller/mpc_status --field data
ros2 topic echo /vesc/high_level/ackermann_cmd --field drive
ros2 topic echo /vesc/commands/motor/speed --field data      # ERPM
ros2 topic echo /vesc/commands/servo/position --field data   # 0.339~0.636 기대
ros2 topic hz /vesc/high_level/ackermann_cmd                 # ≈40 Hz
```
터미널 3 (기록):
```bash
ros2 bag record -o ~/bench_mpc_$(date +%H%M) /vesc/high_level/ackermann_cmd /vesc/ackermann_cmd \
  /vesc/commands/motor/speed /vesc/commands/servo/position /vesc/sensors/core /vesc/odom \
  /car_state/odom /controller/mpc_status /controller/mpc_elapsed_ms /simple_mux/active_host /behavior_strategy
```
노트북 pitwall RViz에서 2D Pose Estimate로 초기 자세 지정.

## 2. 무구동 확인 (RB 누르기 전) — 모두 OK여야 다음 단계
- `active_host` = none, `mpc_status` = solved, high_level `speed` = **0.0**, `motor/speed` 메시지 없음 또는 0

## 3. 수동 전환 확인 — 사용자 확인 후
- LB 누름(스틱 중립): `active_host` = humandrive, `motor/speed` = 0, 바퀴 정지

## 4. MPC 구동 — 사용자 확인 후
1. RB 1회 → `active_host` = autodrive. ERPM이 0부터 서서히 증가(≤ 4614), servo 0.339~0.636 범위
2. 3~5 s 관찰 → **LB** → ERPM 0, 바퀴 정지 확인
3. 중단 기준: 역회전, ERPM > 5000, servo 범위 밖, 이상음/진동 → 즉시 LB, 필요 시 배터리 차단

## 5. 명령 유실 정지 — 사용자 확인 후
1. RB → 바퀴 회전 중 터미널 4에서 `pkill -INT -f lib/controller/controller_manager`
2. 기대: ~0.3 s 안에 `/vesc/ackermann_cmd` speed 0 → 바퀴 감속·정지
3. 정지 확인 후 터미널 1 Ctrl-C로 전체 종료

## 결과 보고
bag 경로, 각 단계 OK/NG, 최대 ERPM, servo 범위, 정지 시간 → WORK_STATUS.md

## ChatGPT 검토 필요
- 트랙 주행 전 조향 한계 실측 방법 (0.15 rad로는 11_18_6 코너 추종 불가)
- TTC estop 비래치 동작을 첫 트랙 주행에서 허용할지
