# ROS2 Jazzy 실차 MPC: shadow → 바퀴 공중 bench → 저속 drive

> 2026-11-18 통합 MPC 변경의 최신 구현·검증 결과와 실행 명령은 [INTEGRATED_MPC_STATUS.md](INTEGRATED_MPC_STATUS.md)를 참조한다. 아래 이전 업로드 시점 결과는 현 통합 MPC의 실차 검증 결과가 아니다.

이 문서는 새 **`mpc_vehicle` 실행 경로**의 사용법이다. 기존 `run_mpc.sh`와 race launch의 시뮬레이션 차단은 그대로 유지한다. 기존 Pure Pursuit, `simple_mux` 구현 및 `/home/sk/JSND`는 수정하지 않는다.

**기본값은 shadow이며 실제 VESC 명령 publisher나 차량 드라이버를 생성하지 않는다.** 소프트웨어 경로는 구현했지만 실제 Jazzy 차량의 장치·비상정지·watchdog·맵 정합성 검증을 완료한 상태는 아니다. 미검증 플래그를 true로 바꾸는 것만으로 검증을 대신할 수 없다. 바퀴 공중 시험을 포함한 모든 실제 출력은 물리 조건 확인과 명시적인 재무장이 필요하다.

## 1. 제공하는 기능과 제한

| 모드 | 하드웨어 노드 | 출력 | 조건 |
|---|---|---|---|
| `shadow` (기본) | 시작하지 않음 | `/mpc_vehicle/shadow_output`만 | 기존 센서 데이터 또는 별도 재생 데이터; 실제 출력을 만들지 않음 |
| `bench` | VESC, LiDAR, PF/EKF, map server | 안전 게이트 → VESC | geometry/독립 정지/watchdog 확인, 바퀴 공중 확인, 실제 정지 허가 입력, 재무장; 한 번에 최대 5초 |
| `drive` | bench와 동일 | 안전 게이트 → VESC | bench 검사 전 항목 통과 기록이 현재 코드·설정·맵과 일치해야 함; 통제된 저속 시험용 |

초기 프로파일의 속도 제한은 0.2m/s, 조향 0.15rad, 조향 변화 0.3rad/s, 가속 0.2m/s²다. **예시값이며 실제 장치의 검증값이 아니다.** 소프트웨어는 속도 0.5m/s, 조향 0.3rad, 각속도 1rad/s, 가속 0.5m/s²보다 큰 프로파일을 거부한다. 후진은 전달하지 않는다. 실차 고속 레이싱 기능을 승인하는 모드는 없다.

새 노드는 공통 `controller.mpc.integrated_mpc.IntegratedMPC`로 속도와 조향을 함께 최적화한다. 가속·감속·횡가속도 상한도 프로파일에서 검증한다. `11_16_3/global_waypoints.json`의 IQP 글로벌 경로를 폐곡선으로 읽어 가까운 지점부터 로컬 구간을 추출한다. 기존 JSON 속도에 sector scaling의 최솟값을 보수적으로 적용하고 저속 상한을 씌운다(배율로 원래 JSON 속도를 높이지 않음). sector YAML의 `only_FTG=true`는 거부한다. 이 실차 경로는 전역 경로 추종 전용이며 기존 state_machine의 동적 회피·추월·FTG로 넘어가지 않는다. 장애물 앞에서는 정지한다. 기존 PP/state_machine의 실행 동작을 바꾸지 않는다.

## 2. 터미널 1: SSH, 소스 준비, Jazzy 의존성

GitHub `seungkyunam/JSND_MPC`의 main에서 새 작업공간을 만든다. 기존 `~/mpc_ws`나 기존 JSND를 덮어쓰지 않는다. 인증·권한 오류가 나면 중단한다. 이 문서가 포함된 커밋인지 clone 후 확인한다.

노트북에서 SSH:

```bash
read -r -p '차량 SSH 사용자: ' VEHICLE_USER
read -r -p '차량 IP/hostname: ' VEHICLE_HOST
ssh "${VEHICLE_USER}@${VEHICLE_HOST}"
```

차량 SSH 터미널에서 clone한다. 이미 `~/mpc_ws`가 있으면 여기서 중단하며 자동 reset/pull로 기존 변경을 덮어쓰지 않는다.

```bash
set -e
if [ -e "$HOME/mpc_ws" ]; then
  echo '중단: ~/mpc_ws가 이미 있습니다. 별도 검토 없이 덮어쓰지 않습니다.'
  exit 1
fi
git clone --branch main --single-branch \
  https://github.com/seungkyunam/JSND_MPC.git "$HOME/mpc_ws"
cd "$HOME/mpc_ws"
git remote -v
git log -1 --format='%H %s'
git status --short
```

새 작업본 준비 후 같은 차량 터미널에서:

```bash
set -e
cd "$HOME/mpc_ws"
test -f run_mpc_vehicle.sh
test -f mpc_vehicle/launch/vehicle_mpc.launch.py
export PYTHONDONTWRITEBYTECODE=1
read -r -p '기존 Jazzy workspace 절대 경로 (예: /home/sk/JSND): ' MPC_UNDERLAY
export MPC_UNDERLAY
source ./mpc_deployment/ros_distro.bash
mpc_detect_distro "$MPC_UNDERLAY" jazzy
source /opt/ros/jazzy/setup.bash
source "$MPC_UNDERLAY/install/local_setup.bash"
test "$ROS_DISTRO" = jazzy
sudo apt update
sudo apt install -y git build-essential cmake python3-dev python3-venv python3-pip \
  python3-packaging python3-yaml python3-colcon-common-extensions \
  ros-jazzy-ament-cmake ros-jazzy-ackermann-msgs ros-jazzy-joy \
  ros-jazzy-tf-transformations ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-robot-localization ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager
for pkg in f110_msgs frenet_conversion vesc_driver vesc_ackermann urg_node particle_filter; do
  ros2 pkg prefix "$pkg"
done
```

Jazzy를 설치한 기존 underlay를 읽어서 사용한다. 기존 JSND에서 rosdep/build를 실행하지 않는다. 기존 binary의 메시지/드라이버 API가 이 저장소와 다른 경우 먼저 검토해야 한다. 기존 실행 셸에 MPC overlay를 source하지 않는다. 새 프로세스의 Python bytecode는 비활성화한다.

## 3. 터미널 1: 기존 `11_16_3`와 차량 설정을 독립 복사

아직 MPC overlay를 source하지 않은 underlay 셸에서:

```bash
MPC_OLD_SHARE="$(ros2 pkg prefix stack_master)/share/stack_master"
export MPC_ASSETS="$HOME/mpc_ws/mpc-build.vehicle-assets"
test ! -e "$MPC_ASSETS"
mkdir -p "$MPC_ASSETS"
cp -aL "$MPC_OLD_SHARE/maps/11_16_3" "$MPC_ASSETS/11_16_3"
cp -L "$MPC_OLD_SHARE/config/CAR/vehicle_config.yaml" "$MPC_ASSETS/vehicle_config.yaml"
cp -L "$MPC_OLD_SHARE/config/CAR/static_transforms.yaml" "$MPC_ASSETS/static_transforms.yaml"
python3 - <<'PY'
import json, os, pathlib
root = pathlib.Path(os.environ['MPC_ASSETS'])
p = json.loads(pathlib.Path('mpc_vehicle/config/vehicle.example.json').read_text())
p['map_dir'] = str(root/'11_16_3')
p['vehicle_config'] = str(root/'vehicle_config.yaml')
p['static_transforms'] = str(root/'static_transforms.yaml')
(root/'vehicle.json').write_text(json.dumps(p, indent=2)+'\n')
print('확인·편집할 새 프로파일:', root/'vehicle.json')
PY
```

원본은 읽기만 하며 심볼릭 링크 대신 실제 파일을 복사한다. 새 `mpc-build.*` 디렉터리는 기존 .gitignore에 의해 Git 업로드에서 제외된다. `11_16_3.yaml`의 `image`는 폴더 내부 상대 경로여야 한다. 글로벌 JSON과 맵 origin/resolution/좌표계·주행 방향이 맞는 세트인지 확인한다. 저장소에는 이 실차 맵이 없으며 이름만 바꾼 다른 맵으로 대신하지 않는다.

프로파일에서 실측할 항목:

- `vehicle_id`: 실제 차량 식별자.
- `wheelbase`, `pose_to_rear_axle_m`: localization의 base_link 위치에서 후륜축까지 **차체 x축 부호를 가진 거리**. 후륜축이 뒤에 있으면 음수다. CAR YAML의 wheelbase와 일치해야 한다.
- `max_steer_rad`, `max_steer_rate_radps`, 속도/가속 상한: 실제 로드휠 각도와 구동기 범위에 맞춰 확인한다. 코드가 CAR의 servo gain/offset/limits 및 ERPM 범위와 대조한다.
- `geometry_confirmed`: 위 확인을 실제로 마친 후에만 true.
- `joy_source_node`: 실제 `/joy` 발행 노드의 정확한 전체 이름. USB joy 기본은 `/joy_node`, ELRS는 `/elrs_joy_node`.
- `estop_source_node`: 실제 독립 정지 장치를 읽어 아래 safety 계약을 구현하는 노드 이름.

`mpc_vehicle/config/vehicle.example.json`은 미검증 기본값을 유지하고, **복사본 `vehicle.json`만** 편집한다. 기존 CAR YAML은 이번 변경에서 수정하지 않는다.

## 4. 독립 비상정지와 watchdog: 없으면 bench/drive 중단

새 gate는 `/mpc_vehicle/estop_clear` (`std_msgs/msg/Bool`)를 구독한다. 실제 안전 장치 인터페이스는 차량마다 다르므로 이 프로젝트는 임의의 GPIO/수신기 값을 만들어 내거나 항상 true를 발행하는 노드를 제공하지 않는다.

필수 계약:

1. `estop_source_node`에 지정한 단일 노드가 **실제 장치가 정상이며 정지가 해제된 동안만** `true`를 최소 20Hz로 보낸다(ROS QoS: reliable, volatile, depth 1). 정지/접점 오류/장치 통신 오류에는 false 또는 발행 중단이어야 한다.
2. false 수신 또는 0.2초 동안 갱신이 없으면 수동·자동 모두 소프트웨어 정지가 latch된다. true로 복구되어도 자동 재출발하지 않는다.
3. 물리 정지는 ROS/SSH/프로세스와 독립적으로 동력을 차단하거나 제동해야 한다. 이 Bool 토픽은 실제 독립 정지를 **대신하지 않는다**.
4. gate/mux/controller를 강제 종료하거나 컴퓨터 통신이 멈춰도 VESC/독립 하드웨어가 0.25초 이내 마지막 구동 명령을 더 이상 유지하지 않는지 실제로 시험한다. 펌웨어 설정을 코드가 조회·설정·입증해 주지 않는다.

차량의 정지 스위치 종류와 watchdog 설정은 아직 제공되지 않아 검증되지 않았다. 해당 인터페이스가 없으면 현재 출력은 열리지 않는다. `ros2 topic pub ... true` 같은 가짜 허가 신호로 우회하지 않는다.

구동기를 비활성화한 상태에서 독립 정지/watchdog를 검증하고, 장치 설정·측정 정지시간·시험 방법·담당자를 텍스트 파일로 `MPC_ASSETS` 아래에 기록한다. 그 뒤에만 다음 필드를 확인해 설정한다:

- `hardware_verification_record`: 그 기록 파일의 절대 경로.
- `independent_estop_verified`, `actuator_watchdog_verified`: 실제 시험 후 true.
- `watchdog_timeout_s`: 실제 확인한 시간(0 초과, 0.25초 이하).
- `bench_authorized`: 차량을 안전하게 고정하고 공중 시험 준비를 확인한 후 true.

수동 정지 보조 수단은 Joy `buttons[0]`와 `/mpc_vehicle/stop` 서비스다. 소프트웨어 서비스는 독립 물리 비상정지가 아니다.

## 5. 터미널 1: 새 Jazzy overlay 빌드

```bash
cd "$HOME/mpc_ws"
bash ./build_mpc_vehicle.sh "$MPC_UNDERLAY"
```

기존 `build_mpc.sh`를 변경하지 않고 호출하여 새 controller/stack_master isolated build를 만든 뒤, 같은 install에 `mpc_vehicle`를 추가한다. MPC Python 의존성은 그 빌드의 venv에 설치된다. 성공 출력의 `Vehicle build completed: ...` 경로를 사용한다.

```bash
read -r -p 'Vehicle build completed의 절대 경로: ' MPC_BUILD
export MPC_BUILD
source "$MPC_BUILD/venv/bin/activate"
source "$MPC_BUILD/install/local_setup.bash"
ros2 pkg prefix mpc_vehicle
ros2 pkg prefix controller
ros2 pkg prefix stack_master
export MPC_PROFILE="$HOME/mpc_ws/mpc-build.vehicle-assets/vehicle.json"
```

첫 빌드가 성공했다고 실차 검증이 완료된 것은 아니다. 이 작업 환경에는 Jazzy가 없어 실제 Jazzy 빌드 결과는 차량에서 확인해야 한다.

## 6. 터미널 1: 출력 없는 shadow 확인

```bash
read -r -p '확인한 로컬 차량 ROS_DOMAIN_ID (0..232, 모든 터미널 동일): ' ROS_DOMAIN_ID
export ROS_DOMAIN_ID
bash "$HOME/mpc_ws/run_mpc_vehicle.sh" "$MPC_BUILD" "$MPC_PROFILE" shadow
```

기존 차량의 도메인과 대상 센서·joy·안전 입력 도메인을 확인하여 같은 값을 입력한다. 스크립트는 도메인을 임의로 추정하지 않고 명시된 0..232만 허용한다. Jazzy LOCALHOST discovery를 사용하므로 입력 노드는 차량과 같은 호스트에서 실행한다. 원격 joystick/Wi-Fi 제어는 지원하지 않는다.

shadow는 하드웨어 노드를 전혀 띄우지 않는다. 기존 센서 그래프가 없다면 입력 대기 상태가 정상이다. 데이터가 있어도 gate는 시작 시 disarmed다. Shadow와 기존 PP의 데이터 비교는 가능하지만, 기존 PP 자체가 보내는 물리 출력까지 shadow가 차단하는 것은 아니다. 먼저 물리적으로 구동기를 비활성화한다.

실제 VESC 출력 없이 관찰 가능한 토픽:

```text
/mpc_vehicle/raw_command
/mpc_vehicle/mux_command
/mpc_vehicle/shadow_output
/mpc_vehicle/controller_status
/mpc_vehicle/gate_status
```

경로 입력은 ROS `/global_waypoints` 구독이 아니라 프로파일에 지정된 파일이다. 다음을 터미널 4의 공통 환경 설정 후 실행하여 실제 선택 파일과 내용을 확인한다.

```bash
export MPC_PROFILE="$HOME/mpc_ws/mpc-build.vehicle-assets/vehicle.json"
python - <<'CHECK_PATH'
import json, os, pathlib, hashlib, yaml
p=json.loads(pathlib.Path(os.environ['MPC_PROFILE']).read_text())
m=pathlib.Path(p['map_dir'])
f=m/'global_waypoints.json'
d=json.loads(f.read_text())
w=d['global_traj_wpnts_iqp']['wpnts']
print('Map:', m)
print('Global path:', f, 'waypoints:', len(w))
print('JSON SHA256:', hashlib.sha256(f.read_bytes()).hexdigest())
print('JSON speed range:', min(x['vx_mps'] for x in w), max(x['vx_mps'] for x in w))
print('Sector profile:', yaml.safe_load((m/'speed_scaling.yaml').read_text()))
print('Commissioning speed cap:', p['speed_limit_mps'])
CHECK_PATH
ros2 param get /mpc_vehicle_controller mode
ros2 param get /mpc_vehicle_controller profile
ros2 topic echo /mpc_vehicle/controller_status --once
ros2 topic echo /mpc_vehicle/solver_ok --once
ros2 topic echo /mpc_vehicle/raw_command --once
ros2 topic info -v /vesc/ackermann_cmd
```

Shadow 계산 확인의 기준은 `mode=shadow`, 올바른 map/profile 경로, 신선한 map/base_link odom, `controller_status`의 `ok=true`와 `reason=solved`, 유한한 raw command다. 토픽이 존재하거나 대기 상태 메시지만 나오는 것은 MPC 계산 성공이 아니다. 실제 액추에이터 출력 없이 계산 결과만 관찰하며, 정지 입력이 없으면 gate가 disarmed/ready=false인 것이 정상이다. 여기에 기재된 명령은 차량에서 아직 실행되지 않았다.

새 경로에는 `/vesc/high_level/ackermann_cmd` 발행자가 없고 shadow에서는 새 `/vesc/ackermann_cmd` 발행자도 없다. 해당 토픽에 기존 PP 발행자가 있다면 그것은 별도 기존 프로세스다.

## 7. 터미널 2/3/4: 환경, joy, 안전 입력, 관찰

동시 세션은 통상 **4개**다: 스택(1), joy(2), 실제 안전 장치 bridge(3), 관찰/arm/stop(4). 안전 bridge가 이미 검증된 시스템 서비스로 돌아가면 사용자 터미널은 3개다. 모든 세션은 같은 차량에서 실행한다.

각 새 SSH 터미널의 공통 환경:

```bash
export PYTHONDONTWRITEBYTECODE=1
read -r -p '선택한 Vehicle build 절대 경로: ' MPC_BUILD
source /opt/ros/jazzy/setup.bash
MPC_UNDERLAY="$(cat "$MPC_BUILD/underlay-workspace.txt")"
source "$MPC_UNDERLAY/install/local_setup.bash"
source "$MPC_BUILD/venv/bin/activate"
source "$MPC_BUILD/install/local_setup.bash"
read -r -p '확인한 로컬 차량 ROS_DOMAIN_ID (0..232, 모든 터미널 동일): ' ROS_DOMAIN_ID
export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

터미널 2: 장치가 연결된 차량에서 **하나만** 사용한다. 실제 버튼/축 매핑 확인 전 무장하지 않는다.

```bash
# USB gamepad인 경우
ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=50.0
```

또는 기존 차량의 ELRS인 경우(프로파일의 joy_source_node도 변경):

```bash
ros2 launch elrs_joy elrs_joy.launch.xml port:=/dev/ELRS_FT232 enable_crc:=true
```

터미널 3: 실제 정지 장치에 맞는 검증된 bridge를 실행한다. **장치가 식별되지 않아 실행 명령을 임의로 제시하지 않는다.** 앞 절의 실제 heartbeat 계약이 충족되지 않으면 게이트의 `ready`는 false다.

터미널 4: 각 echo/hz는 필요한 것만 실행하고 Ctrl-C로 종료한다.

```bash
ros2 node list --no-daemon
ros2 topic echo /mpc_vehicle/gate_status
ros2 topic echo /mpc_vehicle/controller_status
ros2 topic echo /joy
ros2 topic info -v /mpc_vehicle/estop_clear
ros2 topic hz /car_state/odom
ros2 topic hz /scan
ros2 topic echo /mpc_vehicle/shadow_output
```

하드웨어 모드에서 `/vesc/ackermann_cmd`나 `/mpc_vehicle/actuator/*`를 echo/bag으로 구독하지 않는다. gate는 실제 출력 subscriber 소유권도 검사하므로 임의 구독자가 생기면 정지한다. 관찰은 `gate_status`와 `applied_command`를 사용한다. 후자는 **명령 값**이며 실제 조향 측정 피드백은 아니다.

## 8. 터미널 1: 바퀴 공중 bench 시작

기존 PP race/quali/base/localization/mux/VESC/LiDAR 프로세스를 **원래 실행 방법으로 모두 종료**한다. 이름만 변경하거나 다른 도메인으로 옮기는 것은 같은 물리 장치의 소유권을 해결하지 않는다. shadow도 종료한다. 실제 joy와 safety bridge만 남긴다. 차량은 모든 바퀴를 띄우고 차체를 고정하고, 담당자가 물리 정지 장치 옆에 있어야 한다.

```bash
bash "$HOME/mpc_ws/run_mpc_vehicle.sh" "$MPC_BUILD" "$MPC_PROFILE" bench
```

프롬프트에서 실제 조건을 확인한 뒤 `WHEELS_LIFTED`를 입력한다. launch를 직접 실행해도 동일 프로파일/preflight 검사가 적용된다. 기존 센서/control 발행자나 PP 노드가 있으면 하드웨어 노드를 만들기 전에 거부한다. launch는 PF/EKF 기준 새 센서 스택, 별도 mux, MPC, gate만 시작한다. 한 프로세스라도 종료되면 전체 launch를 종료하며 마지막 명령 차단은 독립 watchdog이 담당해야 한다.

PF 초기 자세는 실제 `11_16_3` 위의 차량 위치로 설정한다. RViz의 2D Pose Estimate 또는 이미 검증한 초기 위치 절차를 사용한다. 위치·yaw 값을 임의로 생성한 명령은 제공하지 않는다. `map→odom→base_link`와 odom frame/child frame(`map`, `base_link`)이 맞아야 한다. 잘못된 자세나 오래된 센서는 무장/자동 출력을 막는다.

터미널 4에서 상태가 정상이고 버튼을 모두 놓은 상태를 0.5초 이상 유지한 후:

```bash
ros2 service call /mpc_vehicle/arm std_srvs/srv/Trigger '{}'
```

`success: true` 이후:

- **LB (`buttons[4]`)를 계속 누른 채** throttle `axes[1]`, steering `axes[3]`로 저속 전진과 좌우 조향을 먼저 검사한다. 후진 입력은 0이다.
- **RB (`buttons[5]`)를 계속 누르면** MPC 자동 명령을 선택한다. solver가 정상이며 입력·경로가 유효해야 한다.
- LB+RB는 수동 우선이다. mux의 이전 모드 샘플과 실제 선택 입력이 다르면 그 샘플을 전달하지 않는다.
- 주행 중 버튼을 놓으면 latch stop, 조이스틱 단절도 latch stop이다. 다시 버튼을 누르는 것만으로 움직이지 않는다. 버튼을 놓고 0.5초 후 arm을 다시 호출해야 한다.
- bench는 첫 누름부터 최대 5초가 지나면 정지한다. 재시험도 neutral→arm 절차를 반복한다.

즉시 소프트웨어 정지:

```bash
ros2 service call /mpc_vehicle/stop std_srvs/srv/Trigger '{}'
```

`buttons[0]`도 latch stop이다. 물리 정지 장치를 별도로 시험한다. 통신 단절/프로세스 kill 시험은 물리 정지가 즉시 가능한 상태에서만 한다. Gate의 0속도 요청을 실제 제동 완료로 해석하지 않는다.

## 9. simple_mux–VESC 연결 및 PP 중복 방지

```text
11_16_3 JSON + /car_state/odom → 공통 IntegratedMPC solver
  → /mpc_vehicle/raw_command ───────────┐
/joy (검증된 단일 입력) ────────────────┤
                                     ↓
                    원본 simple_mux 실행 파일 (이름: mpc_vehicle_mux)
                                     ↓ /mpc_vehicle/mux_command
실제 estop_clear + scan/odom + graph → mpc_vehicle_gate
                                     ↓ /vesc/ackermann_cmd (shadow에는 없음)
                           ackermann_to_vesc_node
                                     ↓ /mpc_vehicle/actuator/{motor/speed,motor/brake,servo/position}
                               vesc_driver_node → VESC
```

VESC driver의 6개 command 입력을 모두 private actuator 경로로 remap한다. 사용하지 않는 current/duty_cycle/motor-position 입력은 발행자가 있으면 무장/출력을 거부한다. 일반 PP의 `/vesc/commands/*`는 새 driver에 직접 연결되지 않는다.

preflight + 실행 중 10Hz graph 검사로 기존 PP, 중복 controller/mux, 허가하지 않은 출력 발행자·구독자, 중복 센서 발행자를 검사하고 이상 시 latch stop한다. 프로세스 중복은 같은 호스트 lock으로도 막는다. 이는 보안 경계나 분산 시스템의 원자적 소유권 보증이 아니다. 악의적인 동일 이름 위장, DDS 발견 지연, 별도 프로세스가 동일 직렬 포트를 직접 여는 것까지 소프트웨어 토픽 검사만으로 막을 수 없다. 원래 PP 서비스를 종료하고 물리 포트의 단일 소유권을 운영상 보장해야 한다.

Gate는 joystick·scan·map odom·wheel odom·estop 입력의 0.2초 freshness, 자동 모드의 명령/solver 상태, ROS header age와 monotonic 수신 시간을 검사한다. 전방 ±35도 안의 유효 scan이 부족하거나 거리가 `max(0.35m, |속도|×0.5s+0.2m)` 미만이면 정지한다. 이는 3D 장애물·사각지대까지 보장하는 안전 센서가 아니다. 정지는 속도 가감속 ramp를 기다리지 않고 0 요청을 보낸다. 정상 출력은 속도·조향·증가율을 제한한다.

## 10. bench 완료 기록과 저속 drive

다음 항목은 실제 차량에서 확인해야 하며 자동 테스트가 대신 체크하지 않는다:

1. shadow에 실제 actuator 발행자가 없음.
2. 물리 E-stop 및 driver/firmware watchdog 실제 정지시간.
3. 전진 속도와 좌우 조향 부호/크기.
4. 자동→수동 우선 전환.
5. joystick 분리, controller 명령 단절, scan/odom 단절.
6. 중복 PP/출력 노드 투입 시 정지(실제 PP로 차량을 구동하는 시험이 아님; 구동기 안전 확보).
7. gate 강제 종료 시 독립 watchdog 정지.
8. 오류가 없어져도 자동 재출발하지 않음. 실행 중 driver/mux/localization 파라미터를 변경하면 재시작 전까지 차단됨.
9. 실제 맵 좌표·초기 자세·진행 방향 정합성.

실제 완료 후 터미널 4에서만 다음 기록 도구를 사용한다. 각 항목에 실제 시험을 마친 경우에만 PASS를 입력한다.

```bash
export MPC_PROFILE="$HOME/mpc_ws/mpc-build.vehicle-assets/vehicle.json"
python -m mpc_vehicle.approve "$MPC_PROFILE" \
  "$HOME/mpc_ws/mpc-build.vehicle-assets/wheels_up_approval.json"
```

생성한 파일 절대 경로를 새 프로파일의 `drive_approval`에 넣는다. 기록은 현재 profile(그 경로 필드 제외), 맵 파일, 차량 설정, 하드웨어 기록 및 설치된 MPC/게이트/launch/stack 설정의 해시에 묶인다. 관련 파일을 바꾸면 다시 검증해야 한다. 기록은 운영자의 진술이며 서명된 인증이나 실제 시험의 자동 증명이 아니다.

bench를 종료하고 바퀴를 내린 통제된 저속 시험 환경에서만 터미널 1:

```bash
bash "$HOME/mpc_ws/run_mpc_vehicle.sh" "$MPC_BUILD" "$MPC_PROFILE" drive
```

실제 조건을 확인하여 `CONTROLLED_LOW_SPEED_TEST`를 입력한다. 이전 승인 파일이 없거나 불일치하면 launch가 하드웨어를 만들기 전에 실패한다. 시작 후 다시 neutral→arm→LB/RB hold가 필요하다. 이 명령은 기존 PP를 자동 종료하지 않는다. drive에서도 0.5m/s 이하의 하드 상한과 모든 watchdog/정지 검사가 유지된다.

## 11. 이번 변경과 검증 범위

추가:

- `mpc_vehicle/`: ament_python 패키지, 실제 경로 MPC adapter, 안전 정책/gate, PF 기반 차량 launch, 기본 미검증 프로파일, bench 승인 기록 도구, 테스트.
- `build_mpc_vehicle.sh`, `run_mpc_vehicle.sh`: 기존 시뮬레이션 스크립트와 별개인 Jazzy 준비/실행 경로.
- 이 가이드 갱신.

원래 PP 소스와 조향 전용 MPC는 보존하며, 원래 simple_mux/VESC 코드, 기존 시뮬레이션 guard, `/home/sk/JSND`는 수정하지 않는다. 차량별 승인 파일이나 실제 자격증명을 저장소에 추가하지 않는다. 업로드 커밋은 `git log -1 --format=%H`로 확인한다.

새 안전 로직의 단위 검사와 localhost DDS shadow 검사를 수행한다. DDS 검사는 가짜 센서/명령만 사용하고 VESC/실차 드라이버/기존 Humble 주행 시뮬레이션을 띄우지 않는다. 시험 환경이 Humble인 경우 이는 새 adapter의 통신 호환 검사일 뿐 Jazzy 차량 검증 결과가 아니다.

**아직 미검증:** 실제 Jazzy build/launch 및 기존 underlay 호환성, 실물 `11_16_3`, PF 초기화와 프레임, 차량 geometry/servo/ERPM, 물리 joystick 매핑과 ELRS fail-safe, 실제 독립 정지 bridge, watchdog/제동 거리, CPU 최악 실행시간, wheels-up 및 저속 주행. 이 중 필수 하드웨어 검증이 준비되지 않았으면 shadow에서 멈춘다.

## 12. 업로드 시점 검증 상태와 진행 중단 기준

| 항목 | 확인 결과 |
|---|---|
| 새 안전 정책·프로파일·graph/preflight 단위 검사 | 로컬 31건 통과 |
| 별도 localhost shadow DDS 검사 | 이전 로컬 실행 1건 통과; 가짜 센서/명령, 실차·시뮬레이터 드라이버 없음 |
| 새 패키지 설치 검사 | 로컬 Humble에서 통과; 실제 Jazzy 빌드는 미수행 |
| 실제 차량 SSH | 대상 사용자/주소/workspace 미확정; 접속 미수행 |
| 실제 `11_16_3` 및 속도 프로파일 복사·검증 | 미수행; 위 명령으로 차량 원본을 새 공간에 복사해야 함 |
| 차량 Shadow / Bench / Drive | 모두 미수행 |
| 실제 주행 승인 | 불가: 필수 실차 검증과 안전 장치 연결을 확인하지 못함 |

실패 시 승인 플래그를 바꾸거나 안전 토픽을 가짜로 발행하지 않는다. Bench/Drive를 시작하지 말고 실패 로그, profile의 미완료 항목, 실제 장치 종류와 검증 기록을 확인한다. 원격 접속이 가능해지더라도 바퀴 공중/물리 정지 시험에는 현장 담당자가 필요하다. SSH 연결만으로 물리 상태를 확인했다고 간주하지 않는다.
