# Step 51 적용·기록·롤백 (사용자 승인 후에만)

이 문서는 명령을 준비한 것이며, 차량에 실행하지 않았다. 기본 수정은 obstacle half-lap guard와 진단 기록이다. `5p0`는 가속 비용 변경 OFF다. `step51_5p0`는 비활성 기본값의 실험 후보이며 지연·결합제약 검증 부족으로 고속 사용 승인을 권하지 않는다. 상세 근거는 [검토 보고서](STEP51_FINAL_OFFLINE_REVIEW.md).

## 1. 승인 후 Jetson에 별도 checkout 및 preflight

기존 `~/JSND_MPC_v3`/`~/JSND_INT` 구조를 유지한다. 활성 launch를 운영자가 종료한 상태에서 수행한다. 다른 로컬 수정이 있으면 manifest 검사에서 중단되며 덮어쓰지 않는다.

```bash
git -C "$HOME/JSND_MPC_v3" fetch origin step51-mpc-final-validation
git -C "$HOME/JSND_MPC_v3" worktree add "$HOME/JSND_MPC_step51" origin/step51-mpc-final-validation
step51_repo="$HOME/JSND_MPC_step51"
python3 "$step51_repo/jsnd_int/run/step45_deploy.py" check \
  --manifest "$step51_repo/jsnd_int/run/step51_deploy_manifest.json" \
  --target "$HOME/JSND_INT/src/racing_stack"
```

검사는 Step50의 6개 기존 소스 파일 hash 및 신규 프로파일의 부재를 확인한다. 파일은 controller의 Controller/manager/configuration/integrated_mpc, state_machine node/params, 신규 프로파일이다. 맵·mpc_car.yaml·기존5p0·차량 설정·mux는 배포 대상이 아니다. 실제 Jetson 소스가 GitHub Step50과 다르면 현장 변경 차이를 보존하고 확인해야 하며, force-copy하지 않는다.

## 2. 승인된 기본 수정 적용/빌드

```bash
step51_repo="$HOME/JSND_MPC_step51"
mkdir -p "$HOME/bench_logs"
step51_backup="$HOME/bench_logs/step51_backup_$(date +%Y%m%d_%H%M%S)"
python3 "$step51_repo/jsnd_int/run/step45_deploy.py" apply \
  --manifest "$step51_repo/jsnd_int/run/step51_deploy_manifest.json" \
  --target "$HOME/JSND_INT/src/racing_stack" --backup "$step51_backup"
printf '%s\n' "$step51_backup" > "$HOME/bench_logs/step51_last_backup"
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
cd "$HOME/JSND_INT"
colcon build --base-paths src --packages-select controller state_machine stack_master --symlink-install
```

빌드 성공 후 별도 터미널에서 기존 launcher를 사용한다. 이 명령 또한 사용자 구동 승인 후 실행한다. 최초 확인 속도는 승인된 프로파일을 사용한다.

```bash
bash "$HOME/JSND_MPC_step51/jsnd_int/run/step45_run.sh" mpc 11_18_6 lowspeed
# 동일한 구동 승인의 속도 범위가 5p0까지 포함된 경우에만:
# bash "$HOME/JSND_MPC_step51/jsnd_int/run/step45_run.sh" mpc 11_18_6 5p0
```

`mpc` 모드는 integrated + history_prediction true, heading_robust false다. `improved` 모드는 과거 Step45의 heading_robust 토글이므로 **Step51 비용 개선과 다른 이름**이다. 비용 개선 후보는 프로파일명 `step51_5p0`이며 자동 선택하지 않는다. `use_estop:=true` 유지. 파일 적용은 `step45_deploy.py`만 수행하고 `run/` 스크립트는 새 checkout의 것을 직접 호출한다.

## 3. 기록 (launch 후, RB engage 이전)

```bash
bash "$HOME/JSND_MPC_step51/jsnd_int/run/step45_rec.sh" mpc
ros2 topic hz /controller/mpc_diagnostics
ros2 param get /controller_manager mpc_speed_source
ros2 param get /controller_manager mpc_time_weighted_accel_cost
```

ROS CLI 터미널 환경:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
source "$HOME/JSND_INT/install/local_setup.bash"
export ROS_DOMAIN_ID=173 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/JSND/src/racing_stack/cyclonedds.xml"
```

진단 주제에 각 제어 tick의 측정속도·최적가속·저크 적용가속·예측 속도열·제한 전후 속도·trailing cap·guard 뒤 명령·solver 상태/시간/반복수를 기록한다. `/behavior_strategy`, 장애물, local/global waypoints, state_machine debug/target, `/scan`, odom 및 mux 전후 명령도 함께 기록한다. heading recovery/STOP은 오래된 최적화 진단값을 재사용하지 않는다.

기록 중단 및 보고:

```bash
tmux send-keys -t rec C-c
# recorder가 정상 종료되어 metadata.yaml 작성된 것을 확인한 뒤:
tmux kill-session -t rec
ros2 bag info "$HOME/bench_logs/bags/실제_기록_디렉터리"
python3 "$HOME/JSND_MPC_step51/jsnd_int/run/speed_report.py" \
  "$HOME/bench_logs/bags/실제_기록_디렉터리" --vmax 5.0
```

기존 `speed_report.py`에 `[MPC LAYERS]` 출력 추가: 최적가속·예측속도·제한 전후 속도의 p50/p95 및 lead/jerk/plan 제한율. 이전 bag에 진단이 없으면 복원 불가라고 출력한다. 명령/실속도 차이만 보고 모터 부족 또는 MPC 부족을 단정하지 않는다. 정지 원인은 state, behavior target, MPC status, mux/TTC를 같은 시간으로 대조한다.

## 4. 소스 롤백

운영자가 launch를 종료하고 차량이 정지한 뒤 수행한다. 신규 프로파일을 포함해 적용 전 파일로 복원하고 재빌드한다. 배포 후 변경된 파일은 도구가 복원을 거부하여 보존한다.

```bash
step51_repo="$HOME/JSND_MPC_step51"
step51_backup=$(cat "$HOME/bench_logs/step51_last_backup")
python3 "$step51_repo/jsnd_int/run/step45_deploy.py" restore --backup "$step51_backup"
source /opt/ros/jazzy/setup.bash
source "$HOME/JSND/install/local_setup.bash"
cd "$HOME/JSND_INT"
colcon build --base-paths src --packages-select controller state_machine stack_master --symlink-install
```

프로파일만 복구하려면 launch 종료 후 기존 `5p0`로 재시작하면 비용 개선은 OFF다. 이것은 장애물/진단 코드 롤백과 다르다. 전체 rollback은 위 backup restore를 사용한다.

## 5. 로컬에서 완료한 것과 미완료 경계

- Step50 최신 소스/로그 확인, 원인별 단일요인 실험, 동일 상태 PP 비교, 2배 출력 반증, 지연·응답 민감도, 구간별/물리제약 감사 완료.
- 관련 solver/통합/안전 회귀 68 tests, 임시 디렉터리 적용/롤백 3 tests 통과.
- ROS launch/rosbag는 이 로컬 Python 환경에서 구동하지 않았다. CLI 명령의 셸 문법/배포 manifest는 확인했으나 실제 ROS2 빌드·topic 발행·Jetson timing 검증은 승인 후 남는다.
- 실차 track map와 bag 없는 상태에서 코너 정지 67% 해소, 빠른 실차 랩타임 또는 비침범을 인증하지 않았다. 승인 전 배포·모터 구동은 없다.
