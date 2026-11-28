# c16bced 실차 테스트 최종 준비 절차

이 브랜치는 실차를 자동으로 구동하지 않는다. 현장 운영자가 모든 안전 확인 후
직접 launch, E-stop 해제 및 RB 자율주행 선택을 수행한다. 제어 알고리즘과 목적함수는
`c16bced` 결과로 고정되어 있으며 추가 튜닝은 없다.

## 고정 조건과 ROS2 연결

실차 명령 경로는 다음과 같다.

`race.launch.xml → controller_manager → /vesc/high_level/ackermann_cmd → simple_mux → /vesc/ackermann_cmd → VESC`

MPC 실행 시 launch와 controller가 이 조건을 각각 검사한다.

- 50 Hz (`dt=0.02 s`), N=10
- 첫 단계 0.02초 + 나머지 9단계 각각 0.1088888889초 = 정확히 1초
- integrated 속도·조향 MPC, command-history 지연 보상 유지
- VESC 응답 보상 `tau=0.12 s`
- 글로벌 후속 경로 제동 포락선 유지
- c16bced 가중치: q_position=20, q_heading=16, q_speed=20,
  r_steer=0.2, r_rate=3, r_accel=0.05
- solver time limit/deadline 모두 20 ms
- 기존 차량 물리 한계와 `simple_mux` E-stop 경로 유지

기존 `lowspeed`, `1p5`, `2p25`, `3p0`, `4p0`, `5p0` 파일은 속도 상한만
선택한다. 최종 MPC YAML이 그 뒤에 로드되므로 N, 시간격자, 가중치, VESC 보상은
속도 프로파일로 바뀌지 않는다. 다른 프로파일과 `jsnd` 속도 소스는 시작 전에 거부된다.

## 현장 배포 준비와 복구점

기존 운용 checkout을 수정하지 말고 전용 clean checkout을 준비한다. 아래 작업은
파일 stage와 빌드만 하며 차량을 구동하지 않는다.

```bash
git clone https://github.com/seungkyunam/JSND_MPC.git "$HOME/JSND_MPC_vehicle_final"
git -C "$HOME/JSND_MPC_vehicle_final" switch codex/mpc-real-vehicle-test-final-20261128

mkdir -p "$HOME/JSND_INT_FINAL/src"
target="$HOME/JSND_INT_FINAL/src/racing_stack"
git clone --no-local "$HOME/JSND_INT/src/racing_stack" "$target"
backup="$HOME/bench_logs/final_stage_backup_$(date +%Y%m%d_%H%M%S)"
python3 "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/vehicle_final_stage.py" check --target "$target"
python3 "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/vehicle_final_stage.py" apply \
  --target "$target" --backup "$backup"
printf '%s\n' "$backup" > "$HOME/bench_logs/final_stage_last_backup"

source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
cd "$HOME/JSND_INT_FINAL"
colcon build --base-paths src --packages-select controller spliner state_machine stack_master --symlink-install
```

전용 overlay를 사용할 때 실행 터미널마다 설정한다.

```bash
export JSND_INT_OVERLAY="$HOME/JSND_INT_FINAL"
```

## 실차 전 무구동 확인

1. 차량을 스탠드에 올리고 구동륜을 완전히 띄운다.
2. 물리 E-stop, 조이스틱 A 정지 래치, VESC watchdog을 먼저 확인한다.
3. 다른 race/VESC launch와 `race`/`rec` tmux가 없는지 확인한다.
4. 배터리, 조향 영점, 타이어, LiDAR/PF 정합, 트랙 내 장애물을 확인한다.
5. PP로 기존 경로가 정상인지 먼저 확인한다.

PP 실행:

```bash
export JSND_INT_OVERLAY="$HOME/JSND_INT_FINAL"
bash "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/step45_run.sh" pp 11_18_6 lowspeed
```

MPC는 최초에 반드시 `lowspeed`로 실행한다.

```bash
export JSND_INT_OVERLAY="$HOME/JSND_INT_FINAL"
bash "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/step45_run.sh" mpc 11_18_6 lowspeed
```

아직 E-stop을 해제하거나 RB를 누르지 않은 상태에서 별도 터미널에서 읽기 전용 검사를 한다.

```bash
export JSND_INT_OVERLAY="$HOME/JSND_INT_FINAL"
bash "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/step45_check.sh" mpc
```

`FINAL CONFIG CHECK: PASS`와 `/controller/mpc_status`, odom, 명령 토픽의 정상 주기를
확인한다. PF 초기 자세를 설정하고 레이싱라인 위치·방향, `GB_TRACK`, 장애물 인식과
`solved` 상태를 확인한다.

## rosbag 기록

RB engage 전에 recorder를 시작한다.

```bash
export JSND_INT_OVERLAY="$HOME/JSND_INT_FINAL"
bash "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/step45_rec.sh" mpc
```

기록 항목에는 다음이 포함된다.

- MPC 요구/저크 적용 가속도, IMU 측정 가속도, VESC 명령·측정 속도:
  `/controller/mpc_diagnostics`, `/imu/data`, `/vesc/sensors/imu_corrected`,
  `/vesc/high_level/ackermann_cmd`, `/vesc/ackermann_cmd`, `/vesc/odom`, `/car_state/odom`
- 예측 경로: `/controller/mpc_predicted_path`
- 계산 시간/실패 원인: `/controller/mpc_elapsed_ms`, `/controller/mpc_cycle_ms`,
  `/controller/mpc_loop_period_ms`, `/controller/mpc_status`
- 장애물 회피 상태: `/tracking/obstacles`, `/local_waypoints`, `/behavior_strategy`,
  `/state_machine`, `/state_machine/debug`, `/state_machine/trailing_target`, `/scan`
- E-stop/권한: `/simple_mux/active_host`, `/simple_mux/estop_state`

정상 종료:

```bash
tmux send-keys -t rec C-c
sleep 3
tmux kill-session -t rec
ros2 bag info "$HOME/bench_logs/bags/실제_bag_디렉터리"
```

## VESC 시정수 측정

스탠드 또는 통제된 직선 저속 구간에서 `lowspeed`로 여러 차례 완만한 가속·정속·감속
입력을 기록한다. E-stop 시험 데이터와 바퀴 슬립 구간은 시정수 추정에 사용하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
source "$HOME/JSND_INT_FINAL/install/local_setup.bash"
python3 "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/vesc_tau_report.py" \
  "$HOME/bench_logs/bags/실제_bag_디렉터리"
```

도구는 post-mux `v_cmd`, `/car_state/odom` 속도 및
`dv/dt=(v_cmd-v)/tau`를 사용해 전체/가속/감속 tau와 R²를 출력한다. 현재 0.12초와
비교만 하며 이 최종 브랜치의 값을 자동 변경하지 않는다.

## 현장 테스트 순서

1. 물리 E-stop과 A 버튼 래치 시험. 실패하면 종료한다.
2. PP `lowspeed` wheels-up, 저속 지상 1랩. 기존 동작 확인 후 정지한다.
3. MPC `lowspeed` 시작, config check, recorder 시작.
4. 자세/상태/solver 확인 후에만 `/simple_mux/reset_estop` 호출, RB 선택.
5. 짧은 직선 가감속 후 정지하여 bag과 tau를 먼저 확인한다.
6. MPC 저속 1랩, 장애물이 있으면 보행 속도 접근부터 회피 상태를 확인한다.
7. 각 단계 후 bag 분석과 안전 검토가 통과한 경우에만 1p5 → 2p25 → 3p0 → 4p0 → 5p0
   순서로 별도 재시작한다. 이 문서는 속도 상승을 자동 승인하지 않는다.
8. 이상 시 A/물리 E-stop, launch 종료, PP 또는 기존 overlay로 복구한다.

## 비상정지와 복구

- 즉시 정지: 조이스틱 A 및 독립 물리 E-stop.
- 소프트웨어 래치는 정지·버튼 해제·센서 정상 상태 확인 후에만 운영자가
  `/simple_mux/reset_estop`을 호출한다.
- 빠른 제어기 복구: launch 종료 후 같은 overlay에서 `step45_run.sh pp ...` 실행.
- 전체 소스 복구: 모든 launch 종료와 차량 정지 후 아래를 실행하고 재빌드한다.

```bash
backup=$(cat "$HOME/bench_logs/final_stage_last_backup")
python3 "$HOME/JSND_MPC_vehicle_final/jsnd_int/run/vehicle_final_stage.py" restore --backup "$backup"
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
cd "$HOME/JSND_INT_FINAL"
colcon build --base-paths src --packages-select controller spliner state_machine stack_master --symlink-install
```

stage 이후 파일이 바뀌었거나 backup checksum이 다르면 복구 도구는 덮어쓰지 않고 중단한다.
