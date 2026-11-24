# Step 45 — 2026-11-25 Jetson 비교 주행 준비

기준 버전은 `7ecdc30`(Step 44), 업데이트 브랜치는 `integrated-mpc-v3`다. 기존 PP, 기존 integrated MPC, 개선 integrated MPC를 선택할 수 있다. **개선 MPC는 방향 innovation 제한과 조향 이력 예측을 함께 사용**한다. 방향 개선만 켠 조합은 복합 오차 실험에서 실패하여 ROS 시작 시 거부한다. 두 기능은 기본 OFF이며 실행 중 동적 전환하지 않고 launch를 재시작한다.

오늘은 로컬 오프라인 실험과 관련 테스트만 실행했다. 실차 연결·Jetson Jazzy 빌드·실차 주행은 수행하지 않았다. 아래 명령은 내일 Jetson 터미널에서 실행한다. 실차 모델/제한 YAML, EKF 설정, PP 코드, simple_mux와 E-stop은 업데이트 대상이 아니다.

## 구현과 채택 근거

- Kinematic LTV-MPC solver, N=12, 50 Hz, preview_dt=0.05 s를 유지했다. 기존 `mpc_car.yaml`의 종방향 5/5 m/s², 횡한계 4.5 m/s², 조향 0.4 rad/3.2 rad/s, production 곡률 계획 계수 0.85는 그대로다. 실험용 8.88 m/s·6/6·8·계수 1.0 override는 실행 스크립트 안에서만 적용되며 Jetson에 배포하지 않는다.
- 저장소 `stack_master/config/ekf_pf.yaml`에서 EKF가 PF x/y/yaw + 휠 속도 + `/vesc/sensors/imu_corrected` yaw rate를 이미 융합하는 것을 확인했다. 원시 AHRS orientation은 드리프트 때문에 제외하고, 휠 odom yaw rate는 gyro 이중 입력을 피하려고 제외하고 있다. 이 설정은 이번에 변경하지 않았다.
- 개선 경로는 `/car_state/odom`의 **동일 메시지** pose, header stamp, `twist.angular.z`를 사용한다. 기존 `/imu/data` callback의 부호 반전된 yaw rate를 재융합하지 않는다. EKF yaw rate로 직전 보정 yaw를 현재 sample 시각까지 전파한 뒤, 절대 yaw innovation만 `±1.7 rad/s × sample_dt`로 제한한다. 50 Hz에서 ±0.034 rad이며, Step 38 p95 0.17 rad/기존 0.1 s 시간척도로 사전에 정한 값이다. 추가 EKF·저역통과 필터·샘플 대기 큐는 없다. 정상 회전은 rate를 통해 즉시 반영하지만, 큰 절대 자세 보정은 제한되므로 그 보정 자체에는 정착 시간이 생긴다. 이를 완전한 무지연 필터로 주장하지 않는다.
- 같은 timestamp는 다시 적분하지 않는다. rate가 비정상/누락, sample age 또는 gap이 0.1 s 초과, 시간 역행, 위치 점프 >0.5 m이면 원래 pose로 재기준화한다. 기존 odom freshness·mux·STOP 보호는 그대로 적용된다. 위치 점프 없는 진짜 큰 yaw 재정합도 제한될 수 있다는 한계가 있다.
- `record_applied_steering`에서 **실제 publish된 조향만** timestamp와 함께 저장한다. Controller 내부 후보 계산은 이력에 넣지 않는다. 예측 시 최근 100 ms 명령을 오래된 순서부터, 각 명령의 유지시간만큼 적분한다. 기존 마지막 조향 고정 100 ms 보상과 `if/else`로 배타적이다. 추가 100 ms 보상을 덧붙이지 않는다. 시작/오래된 이력 공백은 0 rad로 취급하며 최근 명령을 긴 공백 전체에 외삽하지 않는다.
- 이것은 고정 transport-delay 가정의 명령 큐 예측이다. 실제 바퀴각 피드백, 서보 rate/타이어 동역학 식별, pose 지연 별도 식별을 대신하지 않는다. 예측 속도 고정 가정과 100 ms 값은 기존대로이며 새 지연 파라미터 탐색은 하지 않았다.

## 실험 결과

`jsnd_int/tests_offline/step45_results.jsonl`은 사전 고정한 기존/방향만/이력만/결합 × 정상/복합의 8회 ablation이다. 이후 실제 publish만 기록하도록 연결을 정리하고, 최종 코드에서 기존/결합 × 정상/복합 4회만 검증했다. 최종 결과는 `step45_final_results.jsonl`이다. 추가 seed·지연값·필터 gain 탐색은 없다.

모든 실행은 Step 44의 test0, 50 Hz, seed=0, 최대 2랩/60초와 동일하다. 위치 sigma=0.0367685 m(x/y), 방향 sigma=0.0867363 rad, 복합 조향 큐 총 110 ms, 정상 큐 20 ms, 속도 잡음 0. MPC 설정과 플랜트 조건도 Step 44와 같다. `controller_p99_ms`는 `Controller.main_loop` 전체로 입력 처리와 MPC를 포함하고 플랜트/ROS publish는 제외한다. 이전 Step 44에는 이 계측이 없어 동일 조건 기준 실행을 다시 측정했다.

### 최종 코드 비교

| 조건 | 버전 | 완료 랩 | 1랩 / 2랩(s) | 평균 / 최대 횡오차(m) | STOP 구간 / 틱 | 제어 연산 P99(ms) |
|---|---|---:|---:|---:|---:|---:|
| 정상 | 기존 MPC | 2 | 9.54 / 8.56 | 0.074 / 0.279 | 2 / 2 | 11.714 |
| 정상 | 개선 MPC | 2 | 9.54 / 8.52 | 0.060 / 0.233 | 7 / 10 | 13.221 |
| 복합 오차 | 기존 MPC | 0 | 미완주(60초) | 1.236 / 1.430 | 8 / 2591 | 8.016 |
| 복합 오차 | 개선 MPC | 2 | 9.90 / 8.74 | 0.147 / 0.726 | 7 / 12 | 15.283 |

- 복합에서 개선 MPC는 완주하고 STOP 2591→12틱(51.82→0.24 s)으로 줄었다. 기존 실패는 예측 tracking 20틱, 현재 tracking envelope 2562틱, 현재 heading envelope 1틱, OSQP 8틱이다. 개선은 예측 tracking 9틱과 OSQP runtime limit 3틱이다. 현재 상태가 허용 범위 밖에서 지속 거부되는 현상은 최종 개선 실행에서 없었다.
- 정상에서도 횡오차는 줄었지만 STOP 2→10틱, P99 11.714→13.221 ms로 악화했다. 개선 정상 STOP은 runtime limit 9틱·solved inaccurate 1틱이다. 따라서 모든 지표가 개선되거나 무정지 주행이 확인된 것은 아니다.
- 실패한 기존 복합의 P99 8.016 ms에는 긴 입력 거부/정지 구간이 포함된다. 개선의 15.283 ms와 이를 직접 비교해 제어 계산이 두 배 느려졌다고 해석하면 안 된다. P99는 20 ms 제어 주기 이내이나 단발 초과는 남고 Jetson CPU 성능을 검증한 수치가 아니다.
- STOP은 solver 실패 틱/연속 구간이다. soft-stop 및 기존 recovery가 있어 실제 정차/E-stop 횟수와 다르다. 평균 횡오차는 완주 약 18초와 미완주 60초의 서로 다른 시간 창이므로 감소율을 순수 추종 성능 향상률로 해석하지 않는다.

### 선택 근거가 된 최초 ablation도 보존

| 복합 조건 variant | 완료 랩 | 2랩(s) | 평균 / 최대 횡오차(m) | STOP 구간 / 틱 | P99(ms) |
|---|---:|---:|---:|---:|---:|
| 기존 | 2 | 8.90 | 0.157 / 0.730 | 24 / 32 | 14.136 |
| 방향 제한만 | 0 | 미완주 | 1.236 / 1.429 | 2 / 2593 | 7.247 |
| 이력 예측만 | 2 | 9.00 | 0.148 / 0.675 | 21 / 24 | 13.367 |
| 두 개선 결합 | 2 | 8.76 | 0.139 / 0.698 | 13 / 18 | 15.192 |

- 이력 예측은 복합 평균/최대 오차 및 STOP을 줄였다. 방향 제한은 **이력 예측과 결합했을 때에만** 랩타임·평균 오차·STOP 추가 개선이 관찰됐다. 이력만 대비 최대 오차와 P99는 증가했다. 따라서 방향 단독은 채택하지 않고, 두 기능을 결합한 비교 모드만 내일 실행 명령으로 제공한다. 이력만 조합은 재현/분리 확인용으로 남지만 추가 실험은 하지 않았다.
- 기존 MPC는 최초 ablation에서는 완주했고 최종 확인 및 Step 44 복합에서는 미완주했다. 실제 wall-clock solver deadline과 warm start/soft-stop/경로의 민감성 때문에 단일 seed도 결과가 달라질 수 있다. 개선 결합은 두 실행 모두 완주했으나 통계적 신뢰도나 실차 성공 보장은 아니다. 유리한 실행만 고르지 않고 두 결과 파일을 모두 보존했다.
- offline yaw rate는 기존 harness의 ground truth다. EKF rate의 실제 지연/잡음과 PF 시간 상관 잡음은 재현하지 않았다. local path/Frenet도 ground truth다. 내일 실차 비교에서 `/car_state/odom`의 rate·header와 계산 주기를 함께 기록해야 하는 이유다.

## Jetson 업데이트와 빌드

아래는 **Jetson에서** 기존 주행 launch를 종료한 뒤 실행한다. `~/JSND` 원본은 건드리지 않고 기존 `~/JSND_INT` overlay의 지정된 5개 파일만 갱신한다. 이 저장소 전체를 `~/JSND_INT/src/racing_stack`에 덮어쓰지 않는다.

```bash
# Jetson terminal
set -e
if pgrep -f '[v]esc_driver_node|[r]ace.launch.xml'; then
  echo '기존 차량 launch를 먼저 종료하세요.'
  exit 1
fi
if [ ! -d "$HOME/JSND_MPC/.git" ]; then
  git clone --branch integrated-mpc-v3 --single-branch \
    https://github.com/seungkyunam/JSND_MPC.git "$HOME/JSND_MPC"
fi
cd "$HOME/JSND_MPC"
git switch integrated-mpc-v3
git pull --ff-only origin integrated-mpc-v3
git log -1 --oneline

python3 jsnd_int/run/step45_deploy.py check \
  --target "$HOME/JSND_INT/src/racing_stack"
step45_backup="$HOME/bench_logs/step45_backup_$(date +%Y%m%d_%H%M%S)"
python3 jsnd_int/run/step45_deploy.py apply \
  --target "$HOME/JSND_INT/src/racing_stack" --backup "$step45_backup"
printf '%s\n' "$step45_backup" > "$HOME/bench_logs/step45_last_backup"

source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
cd "$HOME/JSND_INT"
colcon build --base-paths src --packages-select controller stack_master \
  --event-handlers console_direct+
source install/local_setup.bash
PYTHONPATH="$HOME/JSND_INT/pydeps:${PYTHONPATH:-}" python3 -c \
  'import inspect, osqp; from controller.mpc.pose_prediction import PosePrediction; print(inspect.getfile(PosePrediction)); print("osqp", osqp.__version__)'
```

배포 도구는 대상 4개 기존 파일이 Step 44의 SHA256과 같고 신규 helper 파일이 아직 없는지 먼저 확인한다. 모든 파일을 백업한 뒤 복사하며 물리 설정·맵·EKF·E-stop 파일은 복사하지 않는다. 다른 현장 수정본이면 쓰기 전에 중단한다. 이 경우 강제 덮어쓰지 말고 표시된 파일을 비교해야 한다. 이번 작업은 Jetson 파일의 현재 내용을 원격 확인하지 않았으므로 그 차이까지 자동 병합했다고 주장하지 않는다. 배포 성공 후 실패한 빌드는 아래 백업 복구 명령으로 되돌릴 수 있다.

## 내일 실행 명령

각 명령은 foreground다. 하나를 Ctrl-C로 종료한 다음 다음 모드를 실행한다. 스크립트는 중복 vehicle stack을 거부하며 기존 ROS domain 173/CycloneDDS 설정을 사용한다. 기본 맵은 기존 `11_18_6`, MPC 프로파일은 기존 `lowspeed`(1 m/s)다. 실제 사용할 기존 맵/프로파일을 인자로 지정하고 두 MPC에는 동일하게 사용한다. **PP는 원래 경로 속도/섹터 스케일링을 사용하며 `mpc_profile`이 적용되지 않는다.** 아래 lowspeed MPC와 PP의 랩타임을 같은 속도 조건으로 해석하지 않는다.

```bash
# 1) 기존 PP — 원래 PP 설정, 기존 use_estop 유지
bash "$HOME/JSND_MPC/jsnd_int/run/step45_run.sh" pp 11_18_6

# 2) 기존 integrated MPC — 두 개선 OFF, 기존 마지막 명령 100 ms 예측
bash "$HOME/JSND_MPC/jsnd_int/run/step45_run.sh" mpc 11_18_6 lowspeed

# 3) 개선 integrated MPC — 두 개선 ON, 명령 이력 100 ms 예측만 적용
bash "$HOME/JSND_MPC/jsnd_int/run/step45_run.sh" improved 11_18_6 lowspeed
```

실행 파일은 환경 설정 후 아래 launch 인자를 전달한다. 두 MPC의 `mpc_speed_source:=integrated`를 생략하지 않는다(기존 default는 steering-only `jsnd`).

```bash
# 공통 ROS 환경이 이미 source된 터미널에서 직접 실행할 때
ros2 launch stack_master race.launch.xml map:=11_18_6 localization:=pf \
  controller:=pp use_estop:=true

ros2 launch stack_master race.launch.xml map:=11_18_6 localization:=pf \
  controller:=mpc use_estop:=true mpc_speed_source:=integrated \
  mpc_profile:=lowspeed mpc_rate_hz:=50 mpc_monitor:=false \
  mpc_heading_robust:=false mpc_history_prediction:=false

ros2 launch stack_master race.launch.xml map:=11_18_6 localization:=pf \
  controller:=mpc use_estop:=true mpc_speed_source:=integrated \
  mpc_profile:=lowspeed mpc_rate_hz:=50 mpc_monitor:=false \
  mpc_heading_robust:=true mpc_history_prediction:=true
```

기존 MPC의 A E-stop 래치, LB 수동, RB engage 및 odom/mux freshness 보호를 변경하지 않았다. PP의 기존 safety-mode 기본값도 바꾸지 않았다. 실행 로그는 `~/bench_logs/step45_<mode>_<profile>_<timestamp>.log`에 저장된다.

다른 터미널에서 실제 적용값과 EKF 연결을 확인하고 비교 데이터를 기록한다. 여기서도 동일한 ROS 환경을 source한다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
source "$HOME/JSND_INT/install/local_setup.bash"
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/JSND/src/racing_stack/cyclonedds.xml"
ros2 param get /controller_manager mpc_heading_robust
ros2 param get /controller_manager mpc_history_prediction
ros2 param get /controller_manager mpc_horizon
ros2 param get /controller_manager control_loop_rate_hz
ros2 param get /ekf_localization imu0
# 기대: 개선 true/true, 기존 MPC false/false, N=12, 50 Hz,
# EKF imu0=/vesc/sensors/imu_corrected (PP에서는 MPC param 조회 생략)

step45_mode=improved  # 실행 모드에 맞춰 pp / mpc / improved
mkdir -p "$HOME/bench_logs/bags"
ros2 bag record -s mcap \
  -o "$HOME/bench_logs/bags/step45_${step45_mode}_$(date +%Y%m%d_%H%M%S)" \
  /car_state/odom /car_state/odom_frenet /pf/pose/odom \
  /vesc/sensors/imu_corrected /vesc/high_level/ackermann_cmd \
  /vesc/ackermann_cmd /state_machine /simple_mux/active_host /simple_mux/estop_state \
  /controller/mpc_status /controller/mpc_elapsed_ms \
  /controller/mpc_cycle_ms /controller/mpc_loop_period_ms
# 기록 종료: Ctrl-C. 새 모드마다 새 bag을 사용한다.
```

## 복구

빠른 기능 복구는 launch를 종료한 뒤 `step45_run.sh mpc ...`로 재시작하면 된다. 두 기능 OFF가 기존 계산 경로이며 PP도 그대로 남는다. 소스까지 원복할 때는 아래를 사용한다.

```bash
# Jetson, 주행 launch 종료 후
set -e
if pgrep -f '[v]esc_driver_node|[r]ace.launch.xml'; then
  echo '기존 차량 launch를 먼저 종료하세요.'
  exit 1
fi
step45_backup=$(cat "$HOME/bench_logs/step45_last_backup")
python3 "$HOME/JSND_MPC/jsnd_int/run/step45_deploy.py" restore \
  --backup "$step45_backup"
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
cd "$HOME/JSND_INT"
colcon build --base-paths src --packages-select controller stack_master \
  --event-handlers console_direct+
source install/local_setup.bash
# 원복한 구형 launch에는 신규 flag를 넘기지 않는다.
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/JSND/src/racing_stack/cyclonedds.xml"
ros2 launch stack_master race.launch.xml map:=11_18_6 localization:=pf \
  controller:=mpc use_estop:=true mpc_speed_source:=integrated \
  mpc_profile:=lowspeed mpc_rate_hz:=50 mpc_monitor:=false
# PP 복구는 위 launch의 controller:=pp 경로를 사용한다.
```

복구는 백업 원본 해시와 현재 파일이 배포본/기준본인지 검사한 뒤 적용한다. 배포 뒤 별도 수정된 파일은 자동 삭제/덮어쓰기하지 않는다. `git reset --hard`나 맵/물리 설정 전체 복사는 필요 없다.

## 로컬 검증 기록

- 입력 innovation 제한, 정상 회전 무위상지연, ±π, duplicate stamp, stale/invalid rate, 위치 재정합, 이력 순서/한 번의 horizon/공백, publish 경계, 기존 예측 보존, invalid/unsupported STOP, 기존 odom/mux guard, 배포/복구와 현장 수정 보호: **17 passed**. `jsnd_int/tests_offline/step45_tests.xml`.
- `bash -n`으로 실행 스크립트 문법 확인. XML은 관련 테스트에서 파싱했다. 초기 pytest는 환경에 설치된 외부 plugin 호환성 문제로 수집 전 실패해 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`로 실행했다. 테스트용 fake manager의 `name` 누락 1건을 수정한 뒤 통과했다. 제어기 실패를 테스트에서 무시한 것이 아니다.
- 재현 명령:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
  /tmp/jsnd-step39-venv/bin/python -m pytest -q \
  jsnd_int/tests_offline/test_step45_pose_prediction.py \
  jsnd_int/tests_offline/test_step45_deploy.py
OPENBLAS_NUM_THREADS=1 /tmp/jsnd-step39-venv/bin/python \
  jsnd_int/tests_offline/compare_step45.py --final
```

오늘 개발 범위는 이 구현·결과·실행/복구 준비로 종료한다. 추가 파라미터 탐색이나 실차 연결은 수행하지 않았다.
