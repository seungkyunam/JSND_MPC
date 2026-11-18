# JSND_MPC 차량 준비 및 실행 상태 안내

작성 기준: 2026-11-18, 초기 업로드 커밋 `7ce2091cfd29ea4d2feb582e046ccc1c823321b8`의 실제 코드.

**현재 실차 MPC 주행은 차단되어 있다.** 이 문서는 차량에서 가능한 독립 빌드·맵 준비와 관찰 절차를 제공한다. 실차 주행 시작 명령은 제공하지 않는다. Jazzy 지원 분기는 있지만 실제 Jazzy 차량 검증을 완료했다는 뜻은 아니다. 기존 MPC 개발 및 Humble 검증은 반복하지 않는다.

## 1. 터미널 수와 현재 가능한 범위

| 터미널 | 역할 | 현재 상태 |
|---|---|---|
| 1 | SSH, clone, 의존성, 맵 복사, 빌드 | 아래 순서로 수행 가능 |
| 2 | 동일 환경을 로드한 ROS 그래프 관찰 | 관찰 대상 그래프가 실행 중일 때 가능 |
| 3 | 물리 조이스틱 입력 | 장치가 연결된 컴퓨터에서만; 격리된 입력 검사 조건부 |

준비만 하면 터미널 1개면 된다. 향후 검증된 주행 구성이 마련되면 스택 1개, 조이스틱 1개, 관찰 1개로 통상 3개가 필요하지만, **현재 버전에는 실차 MPC 스택을 시작할 수 있는 명령이 없다.** RViz는 별도 GUI 세션이 필요할 수 있다. SSH에 접속했다고 로컬 노트북 USB 조이스틱이 차량에 연결되는 것은 아니다.

## 2. 터미널 1 — SSH 및 clone

노트북에서 실행한다. 실제 차량 사용자와 주소를 입력한다.

```bash
read -r -p '차량 SSH 사용자: ' VEHICLE_USER
read -r -p '차량 IP 또는 hostname: ' VEHICLE_HOST
ssh "${VEHICLE_USER}@${VEHICLE_HOST}"
```

차량 SSH 셸에서 실행한다. 기존 `~/mpc_ws`가 있으면 중단하며 덮어쓰지 않는다.

```bash
set -e
export PYTHONDONTWRITEBYTECODE=1
if [ -e "$HOME/mpc_ws" ]; then
  echo '중단: ~/mpc_ws가 이미 있습니다. 기존 내용을 먼저 확인하세요.'
  exit 1
else
  git clone --branch main --single-branch \
    https://github.com/seungkyunam/JSND_MPC.git "$HOME/mpc_ws"
fi
```

기존 디렉터리가 있었다면 아래로 진행하지 말고 저장소/변경사항을 확인한다. 인증·권한 오류는 우회하지 않고 중단한다. 새 clone이 완료된 경우:

```bash
cd "$HOME/mpc_ws"
git remote -v
git log -1 --format='%H %s'
git status --short
read -r -p '기존 Jazzy workspace 절대 경로 (예: /home/sk/JSND): ' MPC_UNDERLAY
export MPC_UNDERLAY
test -r "$MPC_UNDERLAY/install/setup.bash"
test -r "$MPC_UNDERLAY/install/local_setup.bash"
test -r /opt/ros/jazzy/setup.bash
source ./mpc_deployment/ros_distro.bash
mpc_detect_distro "$MPC_UNDERLAY" jazzy
source /opt/ros/jazzy/setup.bash
source "$MPC_UNDERLAY/install/local_setup.bash"
test "$ROS_DISTRO" = jazzy
```

Humble/Conda 등 다른 환경이 섞여 있으면 새 셸에서 Jazzy만 로드한다. `MPC_UNDERLAY`는 소스 패키지 디렉터리가 아니라 `install/setup.bash`가 있는 **기존 workspace 루트**다. 새 저장소는 기존 workspace 바깥인 `~/mpc_ws`에 둔다. 기존 JSND의 소스·설정·install은 수정하거나 재빌드하지 않는다.

## 3. Jazzy 의존성 설치 및 underlay 확인

아래는 ROS Jazzy와 ROS apt 저장소가 이미 설치된 차량용이다. OS 패키지 설치는 시스템을 변경하지만 기존 JSND 파일을 수정하지 않는다. 전체 apt upgrade나 기존 workspace의 전체 rosdep/build는 수행하지 않는다.

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-dev python3-venv python3-pip \
  python3-packaging python3-yaml python3-colcon-common-extensions \
  ros-jazzy-ament-cmake ros-jazzy-rclcpp ros-jazzy-rclpy \
  ros-jazzy-ackermann-msgs ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf-transformations ros-jazzy-joy \
  ros-jazzy-robot-localization ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager
```

이는 핵심 빌드/공통 런타임 패키지 목록이며 전체 차량 구성의 설치 보증이 아니다. 이미 설치된 프로젝트 전용 패키지는 Jazzy underlay에서 가져온다. 다음 중 하나라도 없으면 기존 설치를 검토하고 중단한다. 이 스크립트가 해당 패키지들을 새로 빌드해 주지는 않는다.

```bash
for pkg in f110_msgs frenet_conversion gb_optimizer sector_tuner \
  overtaking_sector_tuner frenet_odom_republisher frenet_conversion_server \
  perception state_machine spliner recovery_spliner lane_change_planner \
  vesc_driver vesc_ackermann urg_node particle_filter lap_analyser; do
  ros2 pkg prefix "$pkg"
done
python3 -c 'import rclpy, ackermann_msgs.msg, f110_msgs.msg, frenet_conversion; print("Core imports OK")'
```

목록은 기본 PF/lane_change 경로 기준이다. Cartographer/KISS 또는 GP 예측을 선택하면 각각 추가 패키지가 필요하다. 설치본과 이 저장소의 메시지 정의, 플래너 API가 일치하는지도 확인해야 한다. 특히 이 overlay는 `controller`, `stack_master`만 교체한다.

Python MPC 의존성은 `build_mpc.sh --install-deps`가 **새 venv 안에** 설치한다: `numpy>=1.23,<2`, `scipy>=1.9`, `osqp>=1.0,<2`. 시스템 Python에 `sudo pip`로 설치하지 않는다. 아키텍처별 wheel/컴파일 가능 여부는 차량에서 확인해야 한다.

참고: [ROS Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [colcon 설치](https://colcon.readthedocs.io/en/released/user/installation.html).

## 4. `11_16_3` 맵과 기존 글로벌 패스 복사

저장소에는 `8437`, `test0..2`, `retest0..1`만 있으며 `11_16_3`는 없다. overlay의 `stack_master`가 underlay의 maps도 가리므로 기존 설치에 맵이 있다는 것만으로는 충분하지 않다. **빌드 전에** 기존 설치에서 전체 맵 폴더를 새 소스에 복사한다. 기존 폴더로 연결하는 심볼릭 링크는 만들지 않는다. 일부 튜너는 YAML을 저장할 수 있기 때문이다.

터미널 1, 아직 overlay를 source하지 않은 상태에서:

```bash
export MPC_MAP_SOURCE="$(ros2 pkg prefix stack_master)/share/stack_master/maps/11_16_3"
export MPC_MAP_DEST="$HOME/mpc_ws/stack_master/maps/11_16_3"
python3 - <<'PY'
import json, os, pathlib, shutil, yaml
src = pathlib.Path(os.environ['MPC_MAP_SOURCE']).resolve()
dst = pathlib.Path(os.environ['MPC_MAP_DEST'])
assert not dst.exists(), f'이미 존재함: {dst}'
for name in ('11_16_3.yaml', 'global_waypoints.json', 'speed_scaling.yaml', 'ot_sectors.yaml'):
    assert (src/name).is_file(), f'누락: {src/name}'
cfg = yaml.safe_load((src/'11_16_3.yaml').read_text())
image = pathlib.Path(cfg['image'])
assert not image.is_absolute() and '..' not in image.parts, '외부 이미지 경로: 새 복사본 경로 정리 필요'
assert (src/image).is_file(), f'이미지 누락: {image}'
data = json.loads((src/'global_waypoints.json').read_text())
for name in ('map_info_str', 'est_lap_time', 'centerline_markers', 'centerline_waypoints',
             'global_traj_markers_iqp', 'global_traj_wpnts_iqp',
             'global_traj_markers_sp', 'global_traj_wpnts_sp', 'trackbounds_markers'):
    assert name in data, f'현재 reader와 JSON 형식 불일치: {name}'
assert len(data['global_traj_wpnts_iqp']['wpnts']) >= 2, '글로벌 경로가 비어 있음'
shutil.copytree(src, dst, symlinks=False)
print('복사 완료:', dst)
PY
```

맵을 찾지 못하면 원본 맵의 실제 경로를 확인하여 `MPC_MAP_SOURCE`만 바꾼다. 임의 맵을 `11_16_3`로 이름만 바꾸지 않는다. 위 검사는 파일/형식 검사이며 맵 origin·resolution·주행 방향·좌표 정합성 및 속도 안전성을 검증하지 않는다. Cartographer는 해당 맵의 pbstream, KISS는 해당 PGM 등 localization별 자산도 필요하다. JSON과 맵은 동일한 검증된 세트여야 한다.

실제 경로 흐름:

```text
stack_master/maps/11_16_3/global_waypoints.json
  → gb_optimizer/global_trajectory_publisher (노드 global_republisher)
  → /global_waypoints → sector_tuner → /global_waypoints_scaled
  → state_machine → /behavior_strategy.local_wpnts → controller_manager
```

`/local_waypoints`도 상태 머신의 출력이지만, 현재 controller는 이를 직접 구독하지 않고 `/behavior_strategy`의 `local_wpnts`를 읽는다. `race.launch.xml`의 `control_topic`은 선언만 되어 있어 `control_topic:=/global_waypoints`를 추가해도 입력 경로를 전환하지 않는다. 별도 MPC용 패스를 생성할 필요는 없지만 기존 속도 프로파일이 실차 MPC에 안전하다고 간주해서는 안 된다.

## 5. `build_mpc.sh` 사용법

터미널 1:

```bash
cd "$HOME/mpc_ws"
bash ./build_mpc.sh "$MPC_UNDERLAY" --install-deps --ros-distro jazzy
```

정확한 문법은 `build_mpc.sh ABSOLUTE_UNDERLAY_WORKSPACE [--install-deps] [--ros-distro auto|humble|jazzy]`다. 이번 차량에서는 `jazzy`를 명시한다. 이 스크립트는 실행마다 새 `mpc-build.XXXXXX`를 만들며, 그 안에 venv/build/install/log를 두고 `controller`, `stack_master`만 빌드한다. `--install-deps`를 생략하면 필요한 Python 버전이 이미 보여야 한다. 성공해도 `build-complete.txt`는 **sim-only**다.

성공 출력의 `Build completed: ...` 경로를 입력한다. 실패한 빌드나 단순 최신 디렉터리를 선택하지 않는다.

```bash
read -r -p 'Build completed에 나온 절대 경로: ' MPC_BUILD
export MPC_BUILD
test "$(cat "$MPC_BUILD/build-complete.txt")" = sim-only
test "$(cat "$MPC_BUILD/ros-distro.txt")" = jazzy
source "$MPC_BUILD/venv/bin/activate"
source "$MPC_BUILD/install/local_setup.bash"
ros2 pkg prefix controller
ros2 pkg prefix stack_master
python -c 'import numpy, scipy, osqp; print(numpy.__version__, scipy.__version__, osqp.__version__)'
MPC_SHARE="$(ros2 pkg prefix stack_master)/share/stack_master"
for file in 11_16_3.yaml global_waypoints.json speed_scaling.yaml ot_sectors.yaml; do
  test -f "$MPC_SHARE/maps/11_16_3/$file"
done
```

두 package prefix가 선택한 `MPC_BUILD/install` 아래여야 한다. 빌드 후 맵을 복사했다면 소스에만 있어 설치본에서는 보이지 않을 수 있으므로 새 빌드를 수행해야 한다. 표준 `~/mpc_ws/src` 전체 colcon 빌드 방식으로 대체하지 않는다.

## 6. `run_mpc.sh`의 정확한 인자와 실차 차단

문법:

```text
run_mpc.sh ISOLATED_BUILD_DIR MAP [mpc|pp] [--ros-distro auto|humble|jazzy]
```

`mpc`가 기본이고 `pp`도 시뮬레이션 비교용이다. `--sim false`, `localization:=pf` 등을 받지 않는다. 현재 스크립트의 최종 호출은 다음과 같다.

```text
ros2 launch stack_master race.launch.xml map:=<MAP> sim:=true controller:=<mpc|pp> virtual:=false gp_predictor:=false
```

차단/검사 조건:

- `SIM_ONLY_ISOLATED=YES`, `ROS_DOMAIN_ID` 1..232, 빈 ROS 도메인을 요구한다.
- DDS discovery server/profile/custom discovery 환경을 거부한다.
- 완료 마커가 `sim-only`이고 underlay/build ROS 배포판이 일치해야 한다.
- Jazzy에서는 `ROS_LOCALHOST_ONLY`를 해제하고 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 제한한다.
- 맵 YAML, global JSON, speed_scaling, ot_sectors를 검사한다.
- `validate_controller.launch.py`는 `controller=mpc`와 `sim!=true` 조합을 거부한다.

**실차에서 `SIM_ONLY_ISOLATED=YES`를 설정하는 것은 차단 해제나 실차 운행 승인이 아니다.** `controller_manager` 자체에는 동일한 sim-only 검사 대신 geometry 검사가 있으므로 노드를 직접 띄워 launch guard를 우회해서도 안 된다.

다음은 문법을 보여 주는 **격리 시뮬레이션 전용 예시**다. 이번 작업에서 Humble 검증을 반복하거나 차량에서 실행하지 않았다. 실제 구동기를 물리적으로 비활성화하고 실제 차량 그래프/브리지와 분리한 별도 검증 환경에서만 해당한다. `f1tenth_gym_ros`와 GUI 의존성은 underlay에서 별도로 필요하며, headless SSH에서는 keyboard 노드가 실패할 수 있다.

```bash
SIM_ONLY_ISOLATED=YES ROS_DOMAIN_ID=213 \
  bash "$HOME/mpc_ws/run_mpc.sh" "$MPC_BUILD" 11_16_3 mpc --ros-distro jazzy
```

213은 예시이지 차량에서 비어 있다고 확인된 값이 아니다. 같은 호스트의 다른 프로세스는 LOCALHOST 설정만으로 격리되지 않는다. **현재 준비 절차는 빌드 확인 단계에서 멈춘다.**

## 7. 터미널 2 — 실행 상태 관찰 명령

아래는 위와 같은 **격리 Jazzy 그래프가 별도로 실행 중일 때만** 사용하는 관찰 환경이다. 실행하지 않았다면 노드/토픽이 없거나 조회가 대기하는 것이 정상이다. 실제 차량 도메인에 붙이는 명령으로 혼동하지 않는다.

새 SSH 터미널에서:

```bash
export PYTHONDONTWRITEBYTECODE=1
read -r -p '선택한 mpc-build 디렉터리 절대 경로: ' MPC_BUILD
source /opt/ros/jazzy/setup.bash
MPC_UNDERLAY="$(cat "$MPC_BUILD/underlay-workspace.txt")"
source "$MPC_UNDERLAY/install/local_setup.bash"
source "$MPC_BUILD/venv/bin/activate"
source "$MPC_BUILD/install/local_setup.bash"
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=213
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 node list --no-daemon
ros2 param get /controller_manager lateral_controller
ros2 param get /controller_manager sim
ros2 param get /controller_manager measure
ros2 node info /controller_manager
ros2 topic info -v /behavior_strategy
ros2 topic info -v /vesc/high_level/ackermann_cmd
ros2 topic info -v /vesc/ackermann_cmd
ros2 topic info -v /joy
```

`lateral_controller=MPC`, 현재 허용 모드에서는 `sim=True`가 기대된다. 토픽 존재만으로 MPC 계산/실차 구동을 입증할 수 없다. 아래는 한 줄씩 실행하고 관찰 후 Ctrl-C로 끝낸다.

```bash
ros2 topic hz /car_state/odom
ros2 topic hz /car_state/odom_frenet
ros2 topic hz /behavior_strategy
ros2 topic hz /vesc/high_level/ackermann_cmd
ros2 topic echo /controller/mpc_elapsed_ms --once
ros2 topic echo /vesc/high_level/ackermann_cmd --once
ros2 topic echo /joy --once
```

`mpc_sim.yaml`의 `measure=true`일 때 `/controller/mpc_elapsed_ms`가 발행되지만 초기 입력 대기, FTGONLY 상태 또는 결과 부재에서는 안 나올 수 있다. controller가 FTGONLY이면 MPC 대신 FTG를 실행한다. 로그의 실패/정지 요청, 출력 속도·조향, publisher 개수도 함께 확인한다.

## 8. `/joy` → simple_mux → VESC 및 터미널 3

```text
물리 조이스틱 드라이버 → /joy ───────────────────┐
controller_manager → /vesc/high_level/ackermann_cmd ├→ /simple_mux
                                                  └→ /vesc/ackermann_cmd
                                                     → /vesc/ackermann_to_vesc_node
                                                     → /vesc/commands/motor/speed
                                                     → /vesc/commands/motor/brake
                                                     → /vesc/commands/servo/position
                                                     → /vesc/vesc_driver_node → VESC
```

마지막 하드웨어 경로는 `sim=false`의 low_level 구성이다. 현재 허용된 `run_mpc.sh`에서는 gym simulator가 사용되고 실제 VESC 드라이버는 시작하지 않는다. `race/base/low_level`에는 실차 `/joy` 드라이버 자동 시작이 없다.

코드상 실제 mux 버튼은 `buttons[4]`=수동, `buttons[5]`=자동이며 둘 다 눌리면 수동이 우선이다. 수동 속도는 `axes[1]`, 조향은 `axes[3]`이다. 주석의 A/H 표기보다 `_handle_joy()` 구현을 기준으로 한다. 버튼의 실제 물리 매핑은 장치에서 검증해야 한다.

mux는 초기 idle에서 발행하지 않으며, 모드 선택 후 50Hz로 동작한다. 기본 freshness는 1초이고, 자동 모드에서는 **자동 명령의 freshness만** 검사한다. `/joy`가 끊기거나 자동 버튼을 놓아도 fresh한 자율 명령이 오면 자동 모드를 유지한다. `_clip()`은 정의되어 있으나 출력 경로에서 호출하지 않는다. `use_estop=false`가 기본이고 EStop도 입력 누락 때 명령을 통과시킬 수 있다. 조이스틱 연결만으로 fail-safe가 완성되지 않는다.

터미널 3은 조이스틱이 물리적으로 연결된 컴퓨터에서 실행한다. 아래는 **실제 액추에이터와 분리된 입력 점검 전용**이다. 동일한 Jazzy/underlay 환경과 터미널 2의 격리 환경 설정 후, 드라이버는 둘 중 하나만 선택한다. 차량의 실제 장치 종류는 이번 작업에서 확인하지 못했다.

일반 USB 게임패드인 경우:

```bash
ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=20.0
```

기존 차량이 저장소의 ELRS 설정을 사용하는 경우:

```bash
test -e /dev/ELRS_FT232
ros2 launch elrs_joy elrs_joy.launch.xml port:=/dev/ELRS_FT232 enable_crc:=true
```

ELRS는 기본 420000 baud, publish_rate 100Hz, failsafe_timeout 0.5초를 선언한다. 이것이 mux의 자동 해제를 보장하지 않는다. 입력 관찰은 터미널 2의 `/joy` echo로 한다. 무선 노트북 입력은 LOCALHOST 예시로 연결되지 않으며, 실차 네트워크·QoS·통신 단절 정책을 별도로 검증한 구성 없이는 사용하지 않는다.

## 9. 기존 PP와 동시 실행 시 충돌

같은 ROS 도메인에서 PP race/quali/final/base/low_level 스택과 새 전체 스택을 동시에 실행하면 다음 소유권이 중복된다.

| 노드/기능 | 중복 토픽 또는 장치 |
|---|---|
| controller_manager | `/vesc/high_level/ackermann_cmd`, controller 파라미터/서비스 |
| simple_mux | `/vesc/ackermann_cmd` |
| vesc_driver, ackermann_to_vesc, vesc_to_odom | VESC 직렬 포트, `/vesc/commands/*`, `/vesc/odom` |
| urg_node | 같은 LiDAR 연결, `/scan` |
| PF/Cartographer/KISS, EKF, map_server, static TF | `/car_state/odom`, `/map`, `/tf`, `/tf_static` |
| global_republisher, sector/OT 튜너 | `/global_waypoints*`, 경로 및 sector 설정 |
| state_machine, perception/tracking, planners | `/behavior_strategy`, `/local_waypoints`, `/tracking/obstacles`, planner 출력 |
| joy 또는 elrs_joy 중복 실행 | `/joy` 및 같은 입력 장치 |

노드 이름만 바꿔도 절대 토픽과 물리 장치 충돌은 해결되지 않는다. ROS_DOMAIN_ID만 다르게 해도 같은 VESC/LiDAR 장치를 동시에 여는 것은 안전하지 않다. 이전 프로세스를 실제로 종료했는지 확인해야 한다. overlay를 source한 셸에서 `controller:=pp`를 선택해도 원래 JSND 설치본으로 복귀한 것이 아니다.

기존 PP로 복귀하려면 새 overlay 프로세스를 종료하고 **새 셸에서 원래 Jazzy/JSND만 source**한다. 원래 차량에서 검증한 PP 실행 절차를 사용한다. `.bashrc`에 MPC overlay source를 추가하지 않는다.

## 10. 실차 차단 해제 전 필요한 검증과 미검증 사항

현재 안전 차단과 알고리즘은 변경하지 않았다. 다음은 별도 실차 배포 변경 및 검증에 필요한 항목이다.

1. **Jazzy 대상 호환성:** 차량 OS/CPU, 기존 underlay ABI/메시지 정의, NumPy/SciPy/OSQP 설치, launch/키보드 GUI 의존성, 50Hz 주기에서 최악 계산시간과 CPU 부하. 실제 Jazzy에서 아직 검증하지 않았다.
2. **차량 전용 MPC 설정:** 실측 wheelbase, pose 원점→rear axle 부호/거리, 조향 부호·rad 단위·최대각·각속도, ERPM 속도 변환과 제동거리. `mpc_sim.yaml` wheelbase=0.3302와 CAR vehicle_config=0.324는 다르다. 시뮬레이션의 `mpc_geometry_confirmed=true`를 실차 승인으로 재사용하지 않는다.
3. **맵/경로:** 차량의 `11_16_3` 원본과 글로벌 경로는 이 작업 환경에 없어 실물 검증하지 못했다. 좌표/TF/진행 방향/속도 sector/초기 위치 및 localization 품질을 확인해야 한다.
4. **독립 정지 경로:** 조이스틱 단절, scan/odom/경로 stale, solver 실패/시간 초과, controller/mux 프로세스 종료, SSH/Wi-Fi 단절 시 액추에이터 측에서 정지하고 재무장 전까지 유지되는지 검증한다. 현재 mux/소프트웨어 E-stop만으로는 충족되지 않는다. 제로 속도 요청이 실제 차량 정지와 동일하다고 간주하지 않는다.
5. **모든 운행 상태:** 저속·정지·재시작·FTGONLY·회피/추종 전환과 수동 우선권, 속도 상한 및 실패 후 자동 재출발 여부를 점검한다.
6. **배포 경로:** 실차 전용 설정/런처와 guard 정책을 별도 검토·구현하고, 기존 workspace를 보존하는 롤백 절차를 마련한다. 그 뒤 구동기 비활성 상태의 I/O 검사부터 통제된 저속 검증으로 진행한다.

이번에 차량 SSH, Jazzy 빌드, 장치 연결, 실제 구동, 제동·E-stop, 실차 한 바퀴 주행을 수행하지 않았다. 문서의 준비 명령은 코드에 근거한 절차이며 실행 성공 기록이 아니다.

## 11. GitHub 업로드 검사

기존 코드 업로드는 `7ce2091cfd29ea4d2feb582e046ccc1c823321b8`로 main push 완료 상태에서 시작했다. 이번 변경은 이 가이드뿐이다.

초기 import diff의 경고 세 곳은 모두 7글자 제목의 정상적인 RST underline이다. 문서를 변경하지 않는다.

- `state_estimation/cartographer/README.rst:22`: `Purpose` / `=======`
- `state_estimation/cartographer/docs/source/evaluation.rst:27`: `Concept` / `=======`
- `state_estimation/cartographer_ros/README.rst:22`: `Purpose` / `=======`

이미 커밋된 import에 대한 `git show --check`에는 이 3건이 남을 수 있다. 새 문서의 `git diff --cached --check`와는 별개다. 최종 검사는 index의 파일 내용에 대해 private key/알려진 토큰 패턴/인증 URL/비밀번호·키·토큰 대입 및 후보 문맥을 확인했다. 실제 자격증명은 발견되지 않았으며 후보는 환경변수/Secrets 참조 또는 일반 변수였다. 패턴 검사는 모든 형태의 비밀 부재를 수학적으로 보장하지 않는다.

기존 1,832개 파일은 합계 67,647,650바이트, 최대 파일은 `levine.pgm` 4,194,321바이트다. 큰 파일은 지도/경로/학습·설정 자료 및 문서 이미지였고, 불필요한 대용량 빌드 결과/rosbag/archive는 발견되지 않았다. 가이드 추가 후 최종 커밋과 push 결과는 작업 완료 보고를 기준으로 한다.

## 코드 근거

- [build_mpc.sh](build_mpc.sh), [run_mpc.sh](run_mpc.sh), [배포 문서](mpc_deployment/README.md)
- [실차 MPC guard](stack_master/launch/validate_controller.launch.py), [race launch](stack_master/launch/race.launch.xml)
- [base system](stack_master/launch/base_system.launch.xml), [low level](stack_master/launch/low_level.launch.xml), [localization](stack_master/launch/localization.launch.xml)
- [controller](controller/controller/controller_manager.py), [MPC geometry 검사](controller/controller/mpc/configuration.py)
- [mux](stack_master/scripts/simple_mux_node.py), [EStop](controller/controller/estop.py), [CAR 설정](stack_master/config/CAR/vehicle_config.yaml)
- [글로벌 경로 reader](planner/gb_optimizer/gb_optimizer/readwrite_global_waypoints.py), [ELRS launch](sensor/elrs_joy/launch/elrs_joy.launch.xml)
