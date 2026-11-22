# NEXT_ACTION — 재개 절차 (새 Claude 세션은 이 문서부터)

## 0. 현재 위치 (2026-11-22 단계 9 — WORK_STATUS 참고)
- 코드: JSND_INT `c8727bb` (= 이 브랜치 `jsnd_int/0003-*.patch`), `~/JSND_INT/install` 빌드 완료. 실차 모터 구동 없음
- 해결(오프라인·ROS 모의): P8 OSQP 고착, STOP 시 조향 고착, 100 Hz(모의 100.1 Hz, MPC p99 2.5 ms), 설정 분리(기본 1.0 m/s)
- 구현·모의 통과, **실차 미검증**: E-stop 래치 + 재시작 승인 (MPC 모드 기본 on). LB는 이제 정지 버튼(수동주행 없음)
- 남은 위험: 조향 0.15 rad(P9), 정지 odom 잡음(P6), 실차 100 Hz 여유, 물리 차단은 계속 필요
- 다음: 아래 "4. 다음 실차 시험 절차" (bench, 4바퀴 공중, 1.0 m/s)

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
→ `/vesc/high_level/ackermann_cmd` → simple_mux(MPC 모드 safety_mode: RB 유지=autodrive, LB·A=E-stop 래치, 승인 후 재시작; PP 모드: 기존 RB/LB 수동. TTC estop, 명령 0.25 s 끊기면 0, `/simple_mux/active_host` 발행, MPC 100 Hz / PP 50 Hz)
→ `/vesc/ackermann_cmd` → ackermann_to_vesc(speed→ERPM ×4614, servo = −0.99·δ + 0.487) → VESC

MPC 모드 정지 조건: solver 실패/시간초과/경로이탈 → 0, odom 0.2 s 끊김 → 0, behavior 유실 → 0(기존), TRAILING 등 미지원 상태 → 0, `active_host`≠autodrive → 0(RB 시 0부터 1.0 m/s² 램프)

## 3. 미확인 값 (시험 전제)
| 항목 | 현재 값 | 상태 |
|---|---|---|
| VESC 최소 ERPM | 가정 900 (11-20: 923 ERPM 명령 → 실제 ~460–530 ERPM 회전) | 미확인 |
| MPC 조향 한계/변화율 | 0.15 rad / 0.3 rad/s (11-20 bench 값) | 실측 아님. 트랙 코너 추종엔 부족 |
| servo 변환 | gain −0.99, offset 0.487 | 기계 한계 미측정 |
| MPC 최고속도 | lowspeed 1.0 m/s (≈4614 ERPM) / exp2mps 2.0 m/s(bench 전용) | 설정값 |
| 비상정지 | MPC 모드: safety_mode 래치(A/LB/joy 끊김/토픽) + 재시작 승인, 명령 끊김(mux 0.25 s, VESC watchdog 0.2 s), **배터리 차단(필수 유지)** | ROS 모의만 통과, 실차 미확인 |

## 4. 다음 실차 시험 절차 (Bench, 4바퀴 공중, lowspeed 1.0 m/s) — 모터 구동 단계마다 사용자 승인
### 4-0. 물리 (사용자 확인)
스탠드 고정·4바퀴 공중, 주변 무인·전방 1 m 비움, **배터리 차단 담당 1명(소프트웨어 E-stop은 전원 차단을 대체하지 않음)**, 패드 담당 1명, 배터리 전압 확인(11-22 종료 시 11.4 V).
차량을 raceline 근처, 경로 진행 방향으로 (맵 11_18_6).

### 4-1. 사전 확인 (읽기 전용)
```bash
echo $ROS_DOMAIN_ID                                   # 173
ls -l /dev/ttyACM0 /dev/input/js0; ping -c1 192.168.0.10
pgrep -af "ros2|vesc"                                 # 비어 있어야 함
git -C ~/JSND_INT/src/racing_stack log --oneline -1   # c8727bb
git -C ~/JSND_INT/src/racing_stack status --short     # 비어 있어야 함 (2.0 수정 없음)
grep -n max_speed ~/JSND_INT/install/stack_master/share/stack_master/config/mpc_profile_lowspeed.yaml  # 1.0
```

### 4-2. 실행 (Jetson) — VESC 드라이버가 열리지만 mux는 래치 상태(속도 0)로 시작
```bash
source /opt/ros/jazzy/setup.bash && source ~/JSND_INT/install/setup.bash
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc use_estop:=true \
  > ~/bench_logs/race_mpc_$(date +%Y%m%d_%H%M%S).log 2>&1
# 기본값: mpc_profile:=lowspeed  mpc_rate_hz:=100  safety_mode:=True  mux_rate_hz:=100
```
로그 확인: `safety_mode ON`, `ESTOP LATCHED: startup`, `IntegratedConfig(... max_speed=1.0 ... dt=0.01 ...)`.

### 4-3. 초기 자세 — 노트북 pitwall(domain 173, 핫스팟) RViz 2D Pose Estimate, 스캔·지도 정합 확인

### 4-4. 모터 없이 확인 (래치 상태 그대로)
```bash
ros2 topic echo /simple_mux/active_host --field data        # estop
ros2 topic echo /simple_mux/estop_state --field data        # latched: startup...
ros2 topic hz /vesc/ackermann_cmd                           # ≈100 Hz, speed 0.0 / steer 0.0
ros2 topic echo /vesc/commands/motor/speed --field data     # 0.0
ros2 topic echo /controller/mpc_status --field data         # solved 안정
ros2 topic echo /joy --field buttons                        # A 누름 → [1,0,...] (buttons[0]) 인지 확인. 아니면 F710 X/D 스위치 확인 후 중단
```
**100 Hz 실측 (30 s 기록 → WORK_STATUS)**: `/controller/mpc_loop_period_ms`(출력 주기), `/controller/mpc_cycle_ms`(루프 계산), `/controller/mpc_elapsed_ms`(MPC 계산), `ros2 topic hz /vesc/high_level/ackermann_cmd`. 판정: 주기 p99 ≤ 12 ms, MPC p99 ≤ 8 ms(deadline), `deadline_exceeded` 비율 < 1 %. 미달이면 `mpc_rate_hz:=40`으로 재기동 후 진행하고 결과 기록.

### 4-5. rosbag 기록
```bash
ros2 bag record -o ~/bench_logs/bag_mpc_$(date +%H%M) /vesc/high_level/ackermann_cmd /vesc/ackermann_cmd \
  /vesc/commands/motor/speed /vesc/commands/servo/position /vesc/sensors/core /vesc/odom /car_state/odom \
  /controller/mpc_status /controller/mpc_elapsed_ms /controller/mpc_loop_period_ms /controller/mpc_cycle_ms \
  /simple_mux/active_host /simple_mux/estop_state /behavior_strategy /joy
```

### 4-6. 순서 (각 단계 사용자 승인 후)
1. **재시작 승인**: 모든 버튼 놓고 `ros2 service call /simple_mux/reset_estop std_srvs/srv/Trigger` → `success: true`, host `none`
2. **RB 누르고 유지** 3 s → ERPM 0부터 증가(≤ ~4614 = 1.0 m/s), servo 0.339–0.636 → **RB 놓기** → host `none`, 0 (soft stop)
3. 재승인 불필요: RB 다시 유지 → 회전 중 **LB** → host `estop`, ERPM 0, servo 중앙. (LB+스틱을 움직여도 0이어야 함 — 스틱은 건드리지 말 것)
4. 회전 중 리셋 요청 → **거부**되는지 확인. 정지 1 s 후 리셋 → 성공
5. RB 유지 → **A** → 래치 확인 → 리셋
6. RB 유지 → 패드 USB 분리 또는 전원 OFF → 0.5 s 내 `joy lost` 래치 (선택, 패드 담당 동의 시)
7. RB 유지 → `ros2 topic pub --once /simple_mux/estop std_msgs/msg/Bool "{data: true}"` → 래치
8. 회전 중 `pkill -INT -f lib/controller/controller_manager` → mux freshness 0.25 s 후 0
- 중단 기준: 역회전, ERPM > 5000, servo 범위 밖, 래치 중 ERPM ≠ 0, 이상음 → **배터리 차단**
- `exp2mps`(2.0 m/s)는 위 1~8 통과 후에만, 같은 bench 조건에서 `mpc_profile:=exp2mps`로 별도 실행

### 4-7. 기록 → WORK_STATUS.md
각 단계: 래치 시각→ERPM 0 도달 시간, 리셋 응답 메시지, 명령/실제 ERPM, servo, mpc_status 분포(특히 `maximum iterations` 0건인지), 100 Hz 실측 3종

## 5. Bench 이후
1. 조향 기계 한계·변화율 실측 → `mpc_car.yaml` 반영 (0.15 rad로는 11_18_6 코너 STOP)
2. VESC Tool에서 최소 ERPM 확인
3. 통제 트랙 저속(≤1.0 m/s) 주행 — 물리 정지 수단·관찰자 확보 후

## 6. ChatGPT 검토 필요
- P3(자세 정합 후에도 osqp inaccurate/max iter/deadline 발생 시) 대응: solver 예산(`mpc_max_iter`, `mpc_solver_time_limit_s`) 조정 vs 입력 안정화
- P4 TRAILING 빈발 시 bench 환경 정리로 충분한지
- TTC estop 비래치 동작을 첫 트랙 주행에서 허용할지
