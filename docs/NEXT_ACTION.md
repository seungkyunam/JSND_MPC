# NEXT_ACTION — 재개 절차 (새 Claude 세션은 이 문서부터)

## 0. 현재 위치 (2026-11-22 중단 시점)
- 단계: **실차 Bench, RB 직전**. 모터·서보 구동은 아직 0회. 상세는 `WORK_STATUS.md` 맨 위 요약과 "단계 5"
- 실차에서 확인된 것: 스택 기동, 40 Hz 제어 출력, RB 전 속도 0, VESC 명령 미발행
- 막힌 지점: MPC가 안정적으로 `solved`가 되지 않음 → 원인: PF 초기 자세 미지정(P1). 자세 정합 후 P3·P4 재관찰

## 1. 환경 (Jetson)
| 항목 | 값 |
|---|---|
| ROS | Jazzy, `ROS_DOMAIN_ID=173`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file:///home/jsnd/JSND/src/racing_stack/cyclonedds.xml` (셸 프로파일에 설정됨) |
| DDS 인터페이스 | 127.0.0.1 + Wi-Fi `wlP1p1s0` (2026-11-22: 172.20.10.4, 핫스팟) → 노트북 pitwall 접속 경로 |
| 기존 JSND 원본 | `~/JSND/src/racing_stack` (HEAD `2091417` + 미커밋 launch 2건). **수정 금지**, 빌드 `~/JSND/install` |
| 통합 작업본 | `~/JSND_INT/src/racing_stack` 브랜치 `integrated-mpc` (`5fb1587` 원본 스냅샷 → `0406352` MPC 통합 → `82a1fd5` bench 전 수정) |
| 통합 빌드 | `~/JSND_INT/install` (controller, stack_master만 오버레이; 나머지는 `~/JSND/install`) |
| osqp | `~/JSND_INT/pydeps` (osqp 1.1.3만). race.launch가 MPC 모드 controller_manager에만 PYTHONPATH로 추가 |
| 시험 코드 | `~/JSND_INT/tests_offline`(pytest), `~/JSND_INT/tests_ros/run.sh <pp\|mpc> <safety\|ftg>` (domain 219, VESC 없음) |
| 장치 | VESC `/dev/ttyACM0`(FW 7.0, 60_MK6), F710 `/dev/input/js0`, Hokuyo `192.168.0.10` |
| GitHub | `seungkyunam/JSND_MPC` 브랜치 `jsnd-integrated-mpc`: `docs/`, `jsnd_int/`(Jetson 실행 코드 사본·패치·시험). `main` 수정 금지 |
| 보고서 push 작업본 | 이전 세션은 JSND_MPC worktree(브랜치 `jsnd-integrated-mpc-docs`, origin 추적)를 사용. 새 세션은 `git clone -b jsnd-integrated-mpc` 후 작업해도 됨. push는 `gh auth status`(계정 seungkyunam) 확인 후 |
| 롤백 | `controller:=pp` (기본값, 기존 PP 노드 그대로) 또는 `source ~/JSND/install/setup.bash` |

JSND_INT를 처음부터 재구성해야 할 때: `git clone ~/JSND/src/racing_stack` → `jsnd_int/0000-*.patch` `git apply` → `git am jsnd_int/0001-*.patch jsnd_int/0002-*.patch` → `colcon build --base-paths src --packages-select controller stack_master` (`~/JSND/install` source 후). osqp는 `pip install --target ~/JSND_INT/pydeps --no-deps osqp==1.1.3` 또는 기존 `~/JSND_MPC/mpc-build.WFANmz/venv`에서 복사.

## 2. 데이터 흐름 (통합 후)
JSND 센서·PF/EKF → `/global_waypoints`(iqp) → state_machine `/behavior_strategy` → controller_manager(MPC: 속도+조향, FTGONLY는 FTG)
→ `/vesc/high_level/ackermann_cmd` → simple_mux(RB=autodrive, LB=수동, TTC estop, 명령 0.25 s 끊기면 0, `/simple_mux/active_host` 발행)
→ `/vesc/ackermann_cmd` → ackermann_to_vesc(speed→ERPM ×4614, servo = −0.99·δ + 0.487) → VESC

MPC 모드 정지 조건: solver 실패/시간초과/경로이탈 → 0, odom 0.2 s 끊김 → 0, behavior 유실 → 0(기존), TRAILING 등 미지원 상태 → 0, `active_host`≠autodrive → 0(RB 시 0부터 1.0 m/s² 램프)

## 3. 미확인 값 (시험 전제)
| 항목 | 현재 값 | 상태 |
|---|---|---|
| VESC 최소 ERPM | 가정 900 (11-20: 923 ERPM 명령 → 실제 ~460–530 ERPM 회전) | 미확인 |
| MPC 조향 한계/변화율 | 0.15 rad / 0.3 rad/s (11-20 bench 값) | 실측 아님. 트랙 코너 추종엔 부족 |
| servo 변환 | gain −0.99, offset 0.487 | 기계 한계 미측정 |
| MPC 최고속도 | 1.0 m/s (≈4614 ERPM) | 설정값 |
| 비상정지 | LB(스틱 중립=0), 명령 끊김(mux 0.25 s, VESC watchdog 0.2 s), 배터리 차단. **A 버튼 정지 없음**(JSND) | 실차 미확인 |

## 4. 재개 절차 (Bench) — 모터 구동은 각 단계 사용자 승인 후
### 4-0. 물리 (사용자 확인)
스탠드 고정, 4바퀴 공중, 주변 무인·전방 1 m 비움, 배터리 차단 담당, 패드 담당(LB). 배선 정리 후 LiDAR 케이블·VESC USB 확인.
차량을 raceline 근처, **경로 진행 방향**으로 놓기 (11_18_6).

### 4-1. 사전 확인 (읽기 전용)
```bash
echo $ROS_DOMAIN_ID   # 173
ls -l /dev/ttyACM0 /dev/input/js0; ping -c1 192.168.0.10
pgrep -af "ros2|vesc"  # 비어 있어야 함
git -C ~/JSND_INT/src/racing_stack log --oneline -1   # 82a1fd5
```

### 4-2. 실행 (Jetson)
```bash
mkdir -p ~/bench_logs
source /opt/ros/jazzy/setup.bash && source ~/JSND_INT/install/setup.bash
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc use_estop:=true \
  > ~/bench_logs/race_mpc_$(date +%Y%m%d_%H%M%S).log 2>&1
```
VESC 드라이버가 열린다(RB 전에는 명령 없음).

### 4-3. 초기 자세 (노트북, 같은 핫스팟)
```bash
export ROS_DOMAIN_ID=173
ros2 launch stack_master pitwall.launch.xml
```
RViz 2D Pose Estimate로 실제 위치·방향 지정 → LiDAR 스캔이 지도와 겹치는지 확인.

### 4-4. RB 전 확인 (Jetson, 다른 터미널; 같은 source 후)
```bash
ros2 topic echo /controller/mpc_status --field data          # solved 가 안정적으로 유지
ros2 topic echo /behavior_strategy --field state             # GB_TRACK (TRAILING이면 주변 정리)
ros2 topic echo /simple_mux/active_host --field data         # none
ros2 topic echo /vesc/high_level/ackermann_cmd --field drive # speed 0.0
ros2 topic hz /vesc/high_level/ackermann_cmd                 # ≈40 Hz
ros2 topic echo /vesc/sensors/core                           # fault 0, voltage_input
```
`solved`가 안정적이지 않으면(P3) RB 금지 → `/controller/mpc_status`, `/controller/mpc_elapsed_ms` 기록 후 진단.

### 4-5. 기록 시작
```bash
ros2 bag record -o ~/bench_logs/bag_mpc_$(date +%H%M) /vesc/high_level/ackermann_cmd /vesc/ackermann_cmd \
  /vesc/commands/motor/speed /vesc/commands/servo/position /vesc/sensors/core /vesc/odom /car_state/odom \
  /controller/mpc_status /controller/mpc_elapsed_ms /simple_mux/active_host /behavior_strategy
```

### 4-6. 사용자 승인 후 순서
1. **LB**(스틱 중립): `active_host`=humandrive, `motor/speed`=0
2. **RB** → 3~5 s: ERPM 0부터 증가(≤ ~4614), servo 0.339–0.636 → **LB** 정지
3. RB → 회전 중 `pkill -INT -f lib/controller/controller_manager` → ~0.3 s 내 0
4. 중단 기준: 역회전, ERPM > 5000, servo 범위 밖, 이상음 → LB, 필요 시 배터리 차단

### 4-7. 기록할 값 → WORK_STATUS.md
MPC 명령속도(high_level speed), VESC 명령 ERPM(`motor/speed`), 실제 ERPM(`sensors/core` speed), 조향(steer, servo), 정지까지 시간, mpc_status/elapsed

## 5. Bench 이후
1. 조향 기계 한계·변화율 실측 → `mpc_car.yaml` 반영 (0.15 rad로는 11_18_6 코너 STOP)
2. VESC Tool에서 최소 ERPM 확인
3. 통제 트랙 저속(≤1.0 m/s) 주행 — 물리 정지 수단·관찰자 확보 후

## 6. ChatGPT 검토 필요
- P3(자세 정합 후에도 osqp inaccurate/max iter/deadline 발생 시) 대응: solver 예산(`mpc_max_iter`, `mpc_solver_time_limit_s`) 조정 vs 입력 안정화
- P4 TRAILING 빈발 시 bench 환경 정리로 충분한지
- TTC estop 비래치 동작을 첫 트랙 주행에서 허용할지
