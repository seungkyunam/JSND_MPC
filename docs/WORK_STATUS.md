# WORK_STATUS

> **현재 상태 요약 (2026-11-22 단계 12 — 조향 0.4 rad · MPC 모드 LB 1.0 m/s 적용, 모터 구동 없음)**
> - 코드: JSND_INT `91d0e26` = install. MPC 조향 ±0.4 rad / 변화율 0.3 rad/s 유지, MPC 모드 LB 수동 최고 1.0 m/s(PP 모드 4.0 그대로). RB 자율 / LB 수동 / A 비상정지 래치
> - 실차 점검 미실시: VESC·LiDAR 전원 상태 미확인(단계 11 기준 꺼짐). 다음은 전원 투입 → launch → bench_check
> - 남은 확인: 서보 기계 한계·바퀴 실제 각도 미측정, 0.3 rad/s 변화율 실차 추종 검증 필요

## 2026-11-22 단계 1: 진단 (코드 수정·모터 구동 없음)

### 확인된 사실
1. Integrated MPC는 속도와 조향을 **동시에** 계산한다 (`integrated_mpc.py`, 입력 `[accel, steer]`). 조향 전용 아님.
2. 기존 JSND `controller_manager`에는 **MPC가 연결되어 있지 않다**. MPC 연결본(`JSND_MPC/controller`)은 `sim=false`일 때 MPC를 예외로 거부하고(`controller_manager.py`, `validate_controller.launch.py`), 실차는 별도 `mpc_vehicle` Gate로만 가도록 설계됨.
3. MPC 입력: `controller_manager`가 받는 기존 `/behavior_strategy` 경로점과 `/car_state/odom` 속도 사용 → 기존 JSND 입력 그대로 사용 가능. (`/global_waypoints` 직접 구독은 `mpc_vehicle`에만 있음.)
4. MPC 출력은 기존 `create_ack_msg()` → `/vesc/high_level/ackermann_cmd`로 나간다 (sim 검증 완료, 실차 미검증).
5. 정지 우선순위: `simple_mux`가 MPC 명령 **뒤에서** LB 수동 우선, 명령 freshness(stale→0), 선택적 TTC estop 적용 → MPC 통합으로 우선순위가 깨지지 않음.
6. 과거 bench 로그(`~/mpc_check/step*.log`, 654행): **arm된 적 없음, `cmd_v`와 ERPM 항상 0**. 조향(servo 0.612)만 움직였음.

### 실차 구동을 막는 문제
| # | 문제 | 근거 | 영향 |
|---|---|---|---|
| B1 | **정지 상태에서 속도 명령이 사실상 0** | `result.speed = v_meas + dt×a` (dt=제어주기 0.02~0.025 s). 정지 시 1.5 m/s²×0.025 = **0.04 m/s ≈ 170 ERPM** → VESC 속도 PID 최소 ERPM 미만 가능성 높음 → 바퀴 정지 → odom v=0 → 다음 주기도 0.04 → 영구 정지 | "solved인데 바퀴 안 돎"의 1순위 원인 (명령 변환 단계) |
| B2 | 실차에서 MPC 선택 자체가 차단됨 | `controller_manager.py`: `MPC and not sim → ValueError`, `validate_controller.launch.py` 동일 | 기존 `race.launch.xml`로 MPC 실행 불가 |
| B3 | 실행 환경: `osqp`가 시스템 python에 없음 | `python3 -c "import osqp"` 실패. `mpc-build.WFANmz/venv`에만 osqp 1.1.3 | controller_manager 기동 실패 |
| (참고) | MPC 모드 `FTGONLY` 상태에서 FTG 대신 정지 | `controller_manager.py` `elif lateral_controller != 'MPC'` | 지시사항(FTG 유지)과 불일치 |

### Gate 대비 안전 공백 (기존 JSND에 없음)
- simple_mux 명령 freshness 1.0 s (Gate 0.2 s), autodrive는 RB 래치(hold-to-run 아님), TTC estop 기본 `use_estop=false`.
- 물리 비상정지: LB 수동 전환 + 스틱 중립 / VESC 전원 차단뿐.

### 변경 파일
- 없음 (코드). 문서만: `docs/PROJECT_BASELINE.md`, `docs/WORK_STATUS.md`, `docs/NEXT_ACTION.md`.
- 스냅샷: `~/JSND_backups/JSND_MPC_20261122_jetson/`
- 브랜치: `JSND_MPC`에 `jsnd-integrated-mpc` 생성(미커밋 변경 유지, 커밋·push 안 함).

## 2026-11-22 단계 2: JSND_INT 통합 구현 + 오프라인/ROS 시험 (모터 구동 없음)

### 작업본
- `~/JSND_INT/src/racing_stack`: `~/JSND/src/racing_stack` 독립 clone, 브랜치 `integrated-mpc`
  - `5fb1587` 원본의 미커밋 launch 수정·맵 스냅샷, `0406352` MPC 통합 (로컬 커밋, push 안 함)
- 오버레이 빌드: `~/JSND_INT/install` (controller, stack_master만; 나머지는 `~/JSND/install`)
- 원본 `~/JSND` 변경 없음 (git status 동일 확인)

### 변경 파일 (`0406352`)
| 파일 | +/- | 내용 |
|---|---|---|
| `controller/controller/mpc/*` | 신규 | JSND_MPC 원본 그대로 복사 (sha256 동일) |
| `combined/src/Controller.py` | +93/-0 | MPC 분기, 가속도→속도 setpoint 변환. PP 코드 삭제·수정 없음 |
| `controller_manager.py` | +57/-2 | PP/MPC 선택, 실차 제한 해제, FTGONLY=FTG 유지, MPC 모드 odom 0.2 s 끊김 시 0 |
| `stack_master/launch/race.launch.xml` | +30/-1 | `controller:=pp`(기본, 기존 노드 그대로) / `mpc` |
| `stack_master/config/mpc_car.yaml` | 신규 | 1.0 m/s, 40 Hz, 조향 0.4 rad, FTG 속도 1.0 |

### 해결 내용
- **B1**: setpoint `v_cmd ← v_cmd + a·dt` (MPC 피드백은 계속 odom 실측 속도).
  상한 = min(MPC 계획 최고속도, `mpc_max_speed_mps`, 실측+0.5 m/s), 하한 0. STOP/FTG/입력 유실/0 명령 시 초기화.
  최소 출발속도 강제 없음. `acceleration` 필드는 `ackermann_to_vesc`가 사용하지 않음(코드 확인).
- **B2**: 실차 MPC 거부 제거. **B3**: osqp 1.1.3만 `~/JSND_INT/pydeps`(1.5 MB)에 두고 MPC 모드 controller_manager에만 PYTHONPATH 적용. 시스템 numpy 1.26.4/scipy 1.11.4 유지.
- Jetson 미커밋 변경 중 컨트롤러 관련: 정지 odom 노이즈(−0.02~0 m/s)를 0으로 처리 → 반영.

### 시험 결과
- 빌드: `colcon build --packages-select controller stack_master` 성공
- 오프라인 (`~/JSND_INT/tests_offline`, 맵 11_18_6 iqp): **8 passed**
  - 기존 방식(v+a·dt): 정지에서 못 벗어남 재현 / 새 방식: 0.175 s에 900 ERPM 도달, 10 s 모두 solved
  - 속도 ≤ 1.0, 조향 ≤ 0.4, 바퀴 막힘 시 setpoint ≤ 0.5, 역주행·NaN·경로이탈 → 0
  - solver p50 3.0 / p99 3.2 / max 6.1 ms
- ROS (`~/JSND_INT/tests_ros`, 격리 domain 219, VESC·센서 없음, race.launch 컨트롤러 블록 그대로 + simple_mux, use_estop=true)
  | 시나리오 | 결과 |
  |---|---|
  | RB→MPC | 속도·조향 동시 출력, mux 통과, 3.05 s(RB 직후) 900 ERPM 이상 |
  | LB 수동(스틱 중립) | mux 0 (MPC는 계속 계산) ✓ |
  | odom 0.6 s 유실 | 컨트롤러 0 ✓ |
  | 전방 0.15 m 장애물 | TTC estop 작동. 단 기존 estop은 래치가 아니라 감속 후 TTC>0.35 s면 다시 통과 (기존 동작) |
  | behavior_strategy 유실 | 기존 waypoint safety로 0 ✓ |
  | TRAILING | MPC 미지원 → 0 ✓ |
  | controller 강제 종료 | mux freshness(1.0 s) 후 0 ✓ |
  | FTGONLY | 기존 FTG 1.0 m/s 동작 ✓. 복귀 시 경로에서 벗어나 있으면 MPC STOP(heading/tracking envelope) |
  | PP 회귀 (`controller:=pp`) | 기존 PP 50 Hz 주행 ✓ |
  - 제어 주기 40.1 Hz, p99 28 ms / MPC 계산 p99 6.7 ms (fake world 동시 실행 부하)

### 남은 위험 / 미확인
- `mpc_max_steer_rad 0.4`, `mpc_max_steer_rate_radps 3.2`는 실측값 아님 (bench 프로파일은 0.15 / 0.3)
- VESC 속도 PID 최소 ERPM 실제값 미확인 (시험은 900 가정)
- simple_mux 명령 freshness 1.0 s 그대로 (설정만으로 0.25 s 가능, 미적용)
- RB 전 대기 중에도 setpoint가 최대 0.5 m/s까지 쌓여 RB 즉시 0.5 m/s 명령 (PP는 즉시 프로파일 속도)

## 2026-11-22 단계 2b: 통합 코드 GitHub 반영
- 이 브랜치 `jsnd_int/`에 JSND_INT 수정 파일 4개 + 패치 2개 + 시험 코드 추가 (루트 `controller/`, `stack_master/` 및 Jetson 미커밋 변경은 건드리지 않음)
- push 직전 재시험: 오프라인 8 passed (solver p99 3.5 ms), ROS safety 시나리오 전 항목 기대대로 (제어 40.1 Hz, 주기 p99 28 ms, MPC p99 6.3 ms)

## 2026-11-22 단계 3: bench 전 최소 수정 (JSND_INT `82a1fd5`, 모터 구동 없음)

### 변경 (+28/-4)
| 파일 | 내용 |
|---|---|
| `stack_master/scripts/simple_mux_node.py` | `/simple_mux/active_host`(none/autodrive/humandrive) 상태 발행만 추가. 선택·estop 로직 무변경 |
| `controller/controller/controller_manager.py` | MPC 모드: autodrive가 아니거나 host 상태가 0.25 s 이상 끊기면 속도 0 + setpoint 초기화 → RB 시 0부터 출발 |
| `stack_master/launch/low_level.launch.xml` | `cmd_freshness` 인자(기본 0.25 s) → simple_mux `joy_freshness_threshold` (PP/MPC 공통) |
| `stack_master/config/mpc_car.yaml` | 조향 0.4→**0.15 rad**, 3.2→**0.3 rad/s** (2026-11-20 bench 검증값, servo 0.339~0.636) |

### 시험
- 오프라인 **9 passed**: bench(바퀴 공중, 조향 0.15) 10 s 모두 solved, 0부터 출발, 틱당 증가 ≤ 1.0 m/s²×0.025 s, 조향 ≤ 0.15 rad·변화율 ≤ 0.3 rad/s.
  트랙 폐루프는 미확인 0.4 rad로만 통과 (0.15 rad로는 11_18_6 코너 추종 불가 → STOP)
- ROS bench 시나리오 (domain 219, VESC 없음):
  - RB 전 controller 속도 **0.000** / RB 후 mux 0, 0, 0.025, 0.050, 0.075 … (램프)
  - LB 수동: controller·mux 모두 0
  - controller 강제 종료 → mux **0.311 s** 후 0 (마지막 비0 0.291 s)
  - odom 유실·behavior 유실·TRAILING → 0, TTC estop 동작(비래치, 기존 동작)
  - 제어 40.1 Hz, 주기 p99 27.6 ms, MPC p99 6.3 ms
- PP 회귀: 주행, LB 0, 종료 시 0 ✓

### 확인되지 않은 값 (실차 시험 전 명시)
- **VESC 최소 ERPM 미확인**: 11-20 기록상 923 ERPM 명령 → 실제 ~460~530 ERPM 회전(무부하). 그 이하 미시험
- **실제 조향 한계·변화율 미측정**: 0.15 rad / 0.3 rad/s는 보수값. servo gain −0.99 / offset 0.487도 검증 기록 없음
- JSND에는 Gate의 A 버튼 E-stop이 **없음**. 정지 수단 = LB(스틱 중립=0), controller/mux 명령 끊김(0.25 s + VESC watchdog 0.2 s), 배터리 스위치

## 2026-11-22 단계 3b: GitHub ↔ Jetson 실행 코드 동일성 검증 (코드 변경 없음)
- 대상: GitHub `jsnd-integrated-mpc` `a8f7524` (새로 clone) ↔ Jetson `~/JSND_INT` `82a1fd5` 소스 ↔ `~/JSND_INT/install` 설치본(실제 실행)
- sha256 일치 (앞 12자리):
  | 파일 | 해시 |
  |---|---|
  | `combined/src/Controller.py` | be24290c1b1e |
  | `controller_manager.py` | fa5e0ede4755 |
  | `launch/race.launch.xml` | be47b8c308e5 |
  | `launch/low_level.launch.xml` | 16bfc164fc4e |
  | `config/mpc_car.yaml` | 8f9da23dfb0c |
  | `scripts/simple_mux_node.py` | 42a2397d5315 |
  | `mpc/integrated_mpc.py` (루트 `controller/controller/mpc/`) | e1bbcc6a0691 |
  | `mpc/configuration.py` / `lateral_mpc.py` / `bicycle.py` / `__init__.py` | 6c1ce57cf60e / 772bed10520c / 841f983909bd / f963a5520473 |
- 패치 재현: JSND `2091417` + `0000` + `git am 0001 0002` → JSND_INT의 변경 파일 12개(mapping.launch 포함) 모두 동일
- 이 브랜치가 main 대비 추가한 것: `docs/` 3개, `jsnd_int/` 14개뿐. 맵·빌드·venv/pydeps·인증정보 없음
- JSND 원본(미커밋 2건 그대로), `~/JSND_MPC` 미커밋 18건, `main`(`c8f84e2`) 무변경

## 2026-11-22 단계 4: 실차 Bench — launch 및 RB 전 확인 (모터 구동 없음)
- 물리 안전(스탠드 고정, 4바퀴 공중, 주변 무인, 배터리 차단 담당, 패드 담당): 사용자 확인
- 환경: ROS_DOMAIN_ID 173 / cyclonedds, `controller`·`stack_master` = `~/JSND_INT/install`(`82a1fd5`), VESC `/dev/ttyACM0` fault 0, **배터리 11.6 V**, F710 `/dev/input/js0`, Hokuyo 192.168.0.10 응답
- 실행: `ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc use_estop:=true` → 노드 ~30개 기동, 종료된 프로세스 없음

### RB 전 명령 경로 상태
| 단계 | 토픽 | 결과 |
|---|---|---|
| behavior | `/behavior_strategy` | GB_TRACK |
| MPC | `/controller/mpc_status` | **`outside_heading_envelope` (STOP)** |
| controller 출력 | `/vesc/high_level/ackermann_cmd` | speed 0.0, steer 0.0, 40.0 Hz ✓ |
| mux 선택 | `/simple_mux/active_host` | none ✓ |
| mux 출력 | `/vesc/ackermann_cmd` | 미발행 ✓ |
| VESC 명령 | `motor/speed`, `servo/position` | 미발행 ✓ |
| VESC 상태 | `/vesc/sensors/core` | duty 0, speed 0 ✓ |

### 문제: MPC STOP 원인 (코드 문제 아님)
- 추정 자세 x=2.06, y=3.21, yaw=−0.54 rad → raceline 79번까지 0.39 m(허용 1.0), 경로 방향 −3.03 rad → **방향 오차 2.49 rad (허용 0.7)**
- `/initialpose` 수신 기록 없음 → PF 초기 자세 미지정 또는 차량이 경로 반대 방향

## 2026-11-22 단계 5: 실차 Bench 중간 결과 및 중단 (모터 구동 0회)
요약 로그: `docs/logs/bench_20261122_launch_summary.txt` (전체 로그는 Jetson `~/bench_logs/race_mpc_20261122_120041.log`, 미업로드)

### 실제 관찰·로그로 확인된 것 (성공)
- race.launch(`map:=11_18_6 controller:=mpc use_estop:=true`)로 JSND 전체 스택 + MPC controller_manager 기동, 약 12분 동안 시험 중 종료된 프로세스 없음
- controller_manager MPC 모드 선택, osqp 로드(PYTHONPATH 범위 한정) 정상
- `/vesc/high_level/ackermann_cmd` **40.0 Hz**, RB 전 speed **0.0** (MPC STOP 중에도)
- `/simple_mux/active_host` = **none**, `/vesc/ackermann_cmd`·`motor/speed`·`servo/position` **미발행** → RB 전 VESC 명령 없음
- VESC 연결: fault 0, duty 0, 입력 전압 11.6 V

### 확인하지 못한 것 (미검증)
- MPC 안정적 solved 상태, RB 후 모터 ERPM 상승·서보 구동, LB 수동 정지(실차), controller 종료 시 실차 정지, 실제 ERPM 추종
- VESC 최소 ERPM, 실제 조향 기계 한계·servo 변환, 배터리 차단 절차의 실제 동작
- 실차 Jetson 부하에서의 MPC 계산 여유

### 발생한 문제
| # | 현상 | 근거 | 판단 / 상태 |
|---|---|---|---|
| P1 | MPC `outside_heading_envelope` 지속 (0~9분) | 추정 yaw −0.54 vs 경로 −3.03 rad (오차 2.49, 허용 0.7). 정지 차량인데 추정 자세가 x 2.06→2.92, yaw −0.54→+1.22로 변함 | PF 초기 자세 미지정(`/initialpose` 발행자 0, 노트북 RViz 미연결). **미해결** — 다음 시작 시 2D Pose Estimate 필요 |
| P2 | TF 오류 `map→laser` 209회, EKF `odom→base_link` 43회 | 로그, 시작~약 3.5분 구간 | 위치추정 미수렴과 연관 추정. 미확인 |
| P3 | 후반 약 3분(540 s~) solved와 STOP이 번갈아 발생 | `tracking local path` 41회 vs `nonlinear_speed_profile_violation` 19, `osqp: solved inaccurate` 20, `osqp: maximum iterations` 11, `deadline_exceeded` 3, `nonlinear_prediction_outside_envelope` 5 | 자세 불안정 중 발생이라 원인 분리 안 됨. **자세 정합 후 재관찰 필요** (sim·오프라인에서는 0회) |
| P4 | state_machine 대부분 `TRAILING` (+ OVERTAKE) | `[STATIC_OT] state=` 로그 | 주변 물체/스탠드를 장애물로 인식 추정. MPC는 TRAILING에서 정지(설계). 주변 정리 후 재확인 |
| P5 | Hokuyo `Error count exceeded limit, reconnecting` 1회 | 로그 ~554 s | 재연결됨. 배선 정리 시 LiDAR 케이블 확인 |

### 해결 과정
- RB 전 0.5 m/s 누적, freshness, 조향 한계 → 단계 3에서 수정·시험 완료(코드 `82a1fd5`)
- P1은 원인 진단까지 완료(코드 문제 아님), 사용자 2D Pose Estimate 대기 중 중단
- 코드 수정 없음


## 2026-11-22 단계 6: 배선 정리 후 Bench 재개 — RB 전 점검 (모터 구동 0회, 코드 변경 없음)
요약 로그: `docs/logs/bench_20261122_resume_notes.txt` (전체 로그 Jetson `~/bench_logs/race_mpc_20261122_121742.log`, 미업로드)

### 사전 확인
- DOMAIN 173 / cyclonedds, `/dev/ttyACM0`·`/dev/input/js0` 존재, Hokuyo 192.168.0.10 ping OK, 잔여 ROS 프로세스 없음, JSND_INT `82a1fd5`
- 물리 안전(스탠드 고정, 4바퀴 공중, 배터리 차단 담당, 패드 담당, 주변 무인·LiDAR 시야 정리): 사용자 확인
- 12:17 `ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc use_estop:=true` 기동, MPC 모드·osqp 정상

### RB 전 상태
| 항목 | 결과 |
|---|---|
| `/behavior_strategy` | **GB_TRACK** 501/501 (10 s) ✓ — 이전 P4(TRAILING)는 사람이 차량 앞을 막은 영향으로 판단 |
| `/vesc/high_level/ackermann_cmd` | speed 0.0, 39.98 Hz ✓ (steer는 MPC 값, 예 −0.126) |
| `/simple_mux/active_host` | none ✓ |
| `/vesc/ackermann_cmd`, `motor/speed` | 미발행 ✓ |
| VESC | fault 0, duty 0, **11.5 V** |
| MPC 계산시간 (15 s, n=601) | p50 4.73 / p99 11.71 / max 17.61 ms (deadline 20 ms) |
| `/controller/mpc_status` | ✗ solved 508 / `invalid_local_control_input` 93 (15 s); solved 319 / 83 (10 s) |
| 위치추정 | ✗ `/initialpose` 발행자 0, rviz 노드 없음 → PF GLOBAL INITIALIZATION 상태. 추정 x=1.02 y=−0.03 yaw=−0.01, 정지 중 10 s 변동 x 0.09·y 0.03 m, wp119까지 0.20 m, 경로 방향 −0.59 → 방향 오차 0.57 rad(허용 0.7) |

### 문제
| # | 현상 | 원인(진단) | 상태 |
|---|---|---|---|
| P6 | `invalid_local_control_input` 15~21% | 정지 중 `/car_state/odom` twist.x 잡음 8 s n=400: min −0.0468, max +0.0404, **−0.02 미만 72건(18%)** → `Controller.py` `STANDSTILL_NOISE_MPS=0.02` 초과 → `invalid_or_reverse_speed` → reject(속도 0, 안전측) | 미해결. 거부 시 속도 램프가 0으로 리셋되어 정지 출발 시 최소 ERPM 도달 전 반복 리셋 가능(추정). 안전 한계라 임의 완화 안 함 → 자세 정합 후 재측정, 남으면 대응 결정 필요 |
| P1 | 초기 자세 미지정 (재발) | 노트북 pitwall/RViz 미접속 | 사용자 2D Pose Estimate 대기 |
| P2 | TF 관련 로그 165건 (`detect` 노드) | 미분석 | 관찰 |

## 2026-11-22 단계 7: 2D Pose Estimate 후 재측정 및 구동 준비 (모터 구동 0회, 코드 변경 없음)
| 항목 | 결과 |
|---|---|
| `/initialpose` | 12:29:25 수신 x=1.17 y=0.04 yaw=−0.008 (노트북 RViz) |
| `/behavior_strategy` (5 s) | GB_TRACK 250/250 ✓ |
| `/controller/mpc_status` (5 s) | solved 181 / `invalid_local_control_input` 20 → **solved 90%** |
| 정지 odom vx (5 s, n=249) | min −0.0299, max +0.0283, −0.02 미만 24건(9.6%) → reject 비율과 일치 |
| 위치추정 | x=1.04 y=−0.03 yaw=−0.01, 5 s 변동 0.05/0.03 m, wp119 0.19 m, 방향오차 0.58 rad (허용 0.7, 여유 작음) |
| RB 전 | `active_host` none, high_level speed max 0.000, mux 출력 0건, VESC 11.4 V fault 0 ✓ |
| 기록 | rosbag `~/bench_logs/bag_mpc_*` (12:34~) |

### 판단
- P6 원인 = **정지 odom 잡음 확정**. 코드 수정 안 함: reject는 속도 0(안전측)이고, `mpc_cmd_max_lead_mps=0.5`로 reject 사이 0.5 m/s까지 램프 가능. 바퀴 회전 후 odom>0이면 reject 소멸 예상(실차 확인 필요). 출발 시 단속 가능성 있음
- 방향오차 0.58 rad: bench에서 바퀴 회전 시 추정 자세가 전진해 envelope 이탈 → STOP 가능(안전측)
- 배터리 11.4 V로 저하 중

## 2026-11-22 단계 8: 실차 Bench MPC 실제 구동 (4바퀴 공중, max_speed 2.0)
로그: `docs/logs/bench_20261122_drive_monitor.log`(0.5 s 샘플), `docs/logs/bench_20261122_resume_notes.txt`. rosbag `~/bench_logs/bag_mpc_1230`(402 MB, 미업로드), launch 로그 `~/bench_logs/race_mpc_20261122_123708.log`, `race_mpc_20261122_124530.log`

### 설정 변경 (코드 변경·재빌드 없음)
- `stack_master/config/mpc_car.yaml` `mpc_max_speed_mps: 1.0 → 2.0` (사용자 지시, `~/JSND_INT` src와 install 둘 다). JSND_INT git에는 미커밋
- race.launch 재기동 2회(설정 반영, 조향 고착 해소). 로그에서 `IntegratedConfig(... max_speed=2.0 ...)` 확인

### 구동 구간
| 시각 | 모드 | VESC 명령 ERPM 최대 | 실제 ERPM 최대 | 비고 |
|---|---|---|---|---|
| 12:37:50~12:37:58 | autodrive (MPC) | 9228 (=2.0 m/s) | 7924 | 0부터 램프, solved 위주, 조향 0.15 한계 고착(servo 0.338). 12:37:59부터 `invalid_local_control_input`→`osqp:maximum iterations` → 속도 0 |
| 12:38:17~12:38:19 | **humandrive (LB)** | **18456** | **13781** | MPC 아님. simple_mux `joy_max_speed` 4.0 × 스틱 축 1.0. RB(autodrive, MPC STOP)로 0 |
| 12:40:33~12:41:03 | autodrive | 0 | 0 | MPC `osqp:maximum iterations` 100% (858+253회) |
| 12:43:37~12:43:54 | autodrive (MPC) | 9228 | 8945 | 12:43:02 2D Pose Estimate 후 구동(약 17 s, 실시간 보고 누락) |
| 12:46:00~12:46:12 | autodrive (MPC) | 9217 | 8687 | 재기동 후. solved 위주 + `deadline_exceeded`·`nonlinear_speed_profile_violation` 혼재로 명령 단속 |
| 12:46:13~ | autodrive | 0 | 0 | 다시 `osqp:maximum iterations`, 조향 −0.040 고정 |
- 전 구간 VESC fault 0, 입력 11.4 V, 역회전 없음, 중단 기준(10000 ERPM)은 humandrive 구간만 초과

### 확인된 것
- JSND race.launch + controller:=mpc로 **MPC 속도+조향 명령이 VESC까지 전달되어 실제 회전**: high_level speed → mux → ackermann_to_vesc(×4614) → 명령 ERPM 일치(2.0 m/s → 9228)
- 실제 ERPM은 명령의 약 86~97% 추종(무부하)
- RB 전 0, mux autodrive 전환 시 0부터 램프

### 문제
| # | 현상 | 원인 (진단) | 상태 |
|---|---|---|---|
| P7 | LB 수동 모드에서 18456 ERPM | `simple_mux` humandrive = `axes[1] × joy_max_speed(4.0)`. LB 누른 채 왼쪽 스틱 전진. 현재 /joy 중립값 정상(axes 0, 트리거 1.0) | **LB는 정지 수단이 아님**. 정지 수단 재정의 필요(controller 종료, 배터리 차단, joy_max_speed 제한 검토) |
| P8 | MPC 실패 후 영구 `osqp:maximum iterations` | 오프라인 재현(실차 입력 그대로): 2.0 m/s·이전 조향 −0.102 → 400 iter 실패, 1000 iter solved 12.6 ms(한도 12 ms 경계). 1.0 m/s·조향 0 → 400 iter solved 8.8 ms. 전역 경로 정렬·정지 출발: 2.0 m/s 조향 0 → 66/71 solved, 조향 −0.102 → 전부 실패. reject 시 조향을 직전 값으로 유지 → 같은 실패 반복 | 미해결. 재기동으로만 해소. solver 예산(`mpc_max_iter`)·reject 시 조향 처리 검토 필요 (ChatGPT 검토) |
| P9 | 조향 0.15 rad 한계 고착 | bench에서 바퀴만 돌아 추정 자세가 경로에서 벗어남 + 0.15 rad 보수 한계 | 랩 주행 불가 (오프라인에서도 0.15로 코너 STOP) |
| P6 | `invalid_local_control_input` | 정지 odom 잡음 (단계 6·7) | 회전 중에는 미발생 |

### 판단: 랩 주행 보류
- 사용자 랩 완주 요청에 대해 보류: ① 조향 0.15 rad로 코너 불가 ② 2.0 m/s solver 불안정·고착 ③ 정지 수단(LB) 위험 확인, 2.0 m/s 제동거리 미검증
- 제안 순서: 조향 기계 한계·servo 실측 → `mpc_max_steer_rad` 결정 → 1.0 m/s 복귀 및 solver 안정 조건 오프라인 확인 → 정지 수단 확정 → 바닥 1.0 m/s 직선 일부 → 1랩


## 2026-11-22 단계 9: OSQP 안정화 · 100 Hz · 안전 정지 · 설정 분리 (실차 모터 구동 없음)
JSND_INT `82a1fd5` → `c8727bb` (patch `jsnd_int/0003-*.patch`). 배터리 충전 중 오프라인/ROS 모의로만 진행.

### 1. 백업 / 실행본 차이
- 백업: Jetson `~/JSND_backups/bench_20261122_full/` (bench_logs 전체 454 MB 포함 bag, SHA256SUMS, 실행 yaml 3종). 업로드: `docs/logs/bench_20261122_notes.txt`, `docs/logs/jetson_runtime_2p0_vs_82a1fd5.diff`
- 실행 코드 ↔ GitHub(`89122a3`) sha256 비교: Controller/controller_manager/race·low_level launch/simple_mux **일치**. 차이는 `mpc_car.yaml` 1줄뿐: `mpc_max_speed_mps 1.0 → 2.0`(src·install 모두, 미커밋, sha 83861a74… vs GitHub 8f9da23d…). 이번 커밋에서 기본은 1.0으로 복귀, 2.0은 `mpc_profile_exp2mps.yaml`로만 존재

### 2. P8 OSQP `maximum iterations` 고착 — 원인과 수정
재현 데이터: rosbag `bag_mpc_1230`(12:30~12:50)에서 `/car_state/odom`, `/behavior_strategy` 70 local wpnts, 직전 발행 조향을 추출해 MPC에 그대로 재생 (`jsnd_int/tests_offline/bench_20261122_mpc_inputs.npz`, 고착 3구간 243틱 + 주행 3구간 368틱).

| 조건 (같은 실차 입력) | 결과 |
|---|---|
| 배포 코드·설정(40 Hz, iter 400) | 고착 구간 >90% `maximum iterations` 재현. 고착 틱 **전부 v<0.05 m/s(정지)** |
| 배포 코드, iter만 20000 | 고착 틱 **전부 solved** (최대 >400회, 1~2 ms) → 실행불가능 문제가 아니라 반복 예산 문제 |
| 배포 코드, 직전 조향 유지 vs 0 | solved 0.4% vs 40% → 조향 유지가 크게 악화시키지만 단독 원인 아님 |
| bag 전체 cold start 4795틱 | 실패 2125건 중 2050건이 정지 상태 |

메커니즘: 실패 → 속도 0 → 바퀴 정지 → 정지 입력에서 다시 실패 (재기동 전까지 반복).

수정 (`integrated_mpc.py`, `Controller.py`, `mpc_car.yaml`)
- `mpc_max_iter 400→4000`, `solver_time_limit 4 ms`, `deadline 8 ms` (≤ 1/100 s, 시작 시 검증)
- OSQP 쌍대 변수 warm start (반복 중앙값 510→110)
- `maximum iterations`/`run time limit`/`solved inaccurate`이면 이번 주기는 STOP, 다음 주기는 그 중간 해에서 이어서 풀기 (예산 400·200회로 줄여도 ≤5주기에 복구)
- MPC STOP 시 조향: 직전 값 무기한 유지 → **변화율 한계(0.3 rad/s)로 0 복귀**, 다음 solve의 직전 조향도 실제 발행값
- 결과: bag 전체 연속 재생 16,969 유효 틱 OSQP 실패 **0** (남은 거부는 경로/방향 이탈·비선형 검증 등 안전 검사)

### 3. 100 Hz — 출력 주기와 최적화 계산 구분
최적화(수식·horizon 10·preview 0.1 s)는 그대로, 구현만 최적화: 상수인 제약행렬 A 1회 생성(속도 행은 선형화와 무관, 시험으로 확인), `np.r_`/스칼라 numpy 제거, `np.gradient` 동일 공식 직접 구현. 기존 대비 차이는 solver 허용오차 이내(조향 ≤ 6.6e-5 rad, 8605틱 중 상태 불일치 1).
| 항목 | 값 | 환경 |
|---|---|---|
| MPC 1회 계산 (Python+OSQP) | 3.2 → **1.8~2.2 ms p50**, p99 2.6~3.7 ms | 오프라인, bag 입력, Jetson 유휴 |
| OSQP 자체 | p50 0.2 ms, p99 1.6 ms | 〃 |
| 제어 출력 (controller_manager 발행) | **100.1 Hz**, 타이머 주기 p50 10.0 / p99 10.6~10.8 / max 13 ms | ROS 모의 (domain 219, fake world 동시 실행) |
| simple_mux → `/vesc/ackermann_cmd` | **100.0 Hz** (MPC 모드 `mux_rate_hz` 100; PP는 기존 50) | 〃 |
| MPC 계산 | p50 2.1 / p99 2.4~2.5 / max 7.9~9.2 ms (첫 1회 `deadline_exceeded`) | 〃 |
| 제어 루프 전체 | p50 2.9~3.0 / p99 3.6~4.1 / max 10.7~12.2 ms | 〃 |
| 최적화 수행률 | 매 틱 계산, solved 95.5~98 % (나머지 TRAILING·odom/behavior 유실 시나리오 등 의도된 STOP) | 〃 |
- `mpc_rate_hz:=40`으로 기존 주기 복귀 가능 (설정 유효성 시험 포함)
- 미검증: 실차 Jetson(PF·LiDAR·perception 동시 부하)에서의 100 Hz. 11-22 bench에서는 40 Hz에서 MPC p99 11.7 ms였음(구 코드). 실차에서 `/controller/mpc_loop_period_ms`, `/controller/mpc_cycle_ms`, `/controller/mpc_elapsed_ms` 측정 필요
- 관찰(분석 안 함): FTGONLY 구간에서 제어 루프 최대 104 ms(FTG 처리), 그 구간 출력 88 Hz. FTG 코드는 무변경

### 4. 안전 정지 (simple_mux `safety_mode`, `controller:=mpc` 기본 on, PP 기본 off)
| 항목 | 동작 |
|---|---|
| 래치 조건 | A(buttons[0]), LB(buttons[4]), autodrive 중 `/joy` 0.5 s 끊김, `/simple_mux/estop`(Bool true) |
| 래치 중 | `active_host=estop`, 속도 0·조향 0을 매 주기 발행(능동 제동), MPC도 0 |
| 수동주행 | 없음 (LB+스틱 = 정지). 11-22 18456 ERPM 사건 경로 제거 |
| 기동 | 래치 상태로 시작 → 첫 주행 전 승인 필수 |
| 재시작 승인 | `ros2 service call /simple_mux/reset_estop std_srvs/srv/Trigger` + 조건: 래치 ≥2 s, 모든 버튼 놓음(fresh /joy), `/vesc/odom` |v|<0.05 m/s 1 s 이상 → `none`. RB를 **새로 눌러** 0부터 램프 |
| RB | hold-to-run: 놓으면 `none`(속도 0, 래치 아님) |
ROS 모의 결과(estop 시나리오, 지면 주행 모델): 래치까지 +0.05 s(joy 20 Hz 주기), 래치~리셋 구간 mux 속도·조향 최대 0.000, LB+스틱 전진에서도 humandrive 0회, 이동 중 리셋 거부("latched only 0.3 s; wheels not still"), 조건 충족 리셋 4/4 성공, joy 끊김 +0.50 s 래치.
**실차 미검증**: F710 모드(X/D)에 따른 A 버튼 번호, 실제 joy_node 발행 주기, VESC 제동. 소프트웨어 정지이며 Jetson/mux/USB 장애 시 동작하지 않음 → **물리 전원 차단 담당 유지 필수**.

### 5. 시험 결과
- 오프라인 `tests_offline`: **29 passed** (기존 9 + 신규 20: 고착 재현·원인 분리·해소, 예산 초과 복구, 기존 대비 동일성, A 행렬 상수성, gradient 동일성, STOP 조향 복귀, STOP 후 정지 출발 복구, 100 Hz 계산 예산, 프로파일, SafetyGate 9건)
- ROS 모의 (domain 219, VESC 없음)
  - MPC + safety_mode estop 시나리오: 위 4절 전 항목 기대대로
  - MPC + 기존 mux(`safety_mode:=false`) safety 시나리오: RB 전 0, 0부터 램프(0.01 m/s/틱), LB 0, odom 유실 0, TTC estop, behavior 유실 0, TRAILING 0, controller 종료 후 0.329 s에 0 → 기존과 동일
  - FTG 시나리오: FTGONLY에서 기존 FTG 1.0 m/s 동작(무변경 확인만)
  - PP 회귀 1회(`controller:=pp`, mux 50 Hz, safety off): 주행, LB 0, behavior 유실·TRAILING 0, controller 종료 후 0 ✓ (요약 스크립트가 MPC 지표 없음으로 끝에 오류 → 수정만, 재실행 안 함)
  - 시험 종료 시 Traceback은 `pkill -INT`에 의한 rclpy 종료 메시지(기능 무관)

### 6. 미해결
| # | 내용 |
|---|---|
| U1 | 실차 Jetson 부하에서 100 Hz 여유 미측정 |
| U2 | E-stop 래치/리셋 실차 미검증 (패드 버튼 매핑, joy 주기, VESC 제동) |
| P6 | 정지 odom 잡음 < −0.02 m/s → `invalid_local_control_input` (변경 안 함) |
| P9 | 조향 0.15 rad로 코너 추종 불가 (ROS 모의 지면 주행에서도 `nonlinear_prediction_outside_envelope` 반복) → 실측 필요 |
| U3 | 단발 `solved inaccurate`/`deadline_exceeded` 1틱 STOP 시 속도 setpoint가 0으로 리셋되어 명령 단속 가능 |
| U4 | FTGONLY 구간 제어 루프 최대 ~100 ms (분석 범위 밖) |

## 2026-11-22 단계 10: 조향각 조사 · LB 수동 복구 · A 비상정지 (실차 모터 구동 없음)
JSND_INT `c8727bb → 701b49a` (`safety_gate.py`, `simple_mux_node.py`만 변경), 이 브랜치 `jsnd_int/0004-*.patch` + 사본. 재빌드: `controller`, `stack_master`만.

### 1. 조향각 비교 (조사만, 파라미터 변경 없음)
| 경로 | 최대 조향 | 변화율 | 위치 | 성격 |
|---|---|---|---|---|
| PP (L1) | ±0.53 rad | 틱당 ≤0.4 rad (50 Hz → 20 rad/s, 사실상 무제한) | `controller/combined/src/Controller.py` 355–359 (하드코딩) | 소프트웨어 |
| FTG | ±0.4 rad | 없음(`ftg_steer_ema` 0) | `config/controller.yaml` `ftg_max_steer` | 소프트웨어 |
| 수동(LB) | ±0.4 rad × 오른쪽 스틱 X | 없음 | `simple_mux` `joy_max_steer` 기본값 | 소프트웨어 |
| **MPC** | **±0.15 rad** | **0.3 rad/s** | `config/mpc_car.yaml` | 11-20 bench 보수값 (주석: "실측 후 상향") |
| 서보 변환 | servo = −0.99·δ + 0.487 | – | `config/CAR/vehicle_config.yaml` (주석상 캘리브 결과) | 캘리브값 |
| VESC 서보 명령 범위 | 0.0 ~ 1.0 | – | `CAR/vehicle_config.yaml` `servo_min/max` (low_level이 vesc 설정으로 사용) | 소프트웨어(사실상 무제한) |

- 서보 범위 0~1을 각도로 바꾸면 **왼쪽 +0.492 / 오른쪽 −0.518 rad**. PP의 0.53 rad는 이 범위를 넘으므로 실제로는 서보 명령이 0/1에서 잘림 → PP 실효 한계 ≈ 0.49 rad
- `sensor/vesc/vesc_driver/params/vesc_config.yaml`(0.15~0.85)은 race 실행 경로에서 사용되지 않음
- **하드웨어 한계 실측 기록 없음** (`odom_calib` 도구는 주행 데이터로 max_steer를 추정하지만 보고서 없음). 서보 기계 끝점·실제 바퀴 각도·변화율 모두 미측정
- 차이 원인: MPC 값은 11-20 첫 bench(4바퀴 공중)에서 서보 범위·FTG 0.4 안쪽으로 **의도적으로 낮게 잡은 값**이며 하드웨어 근거 없음. 변화율 0.3 rad/s는 PP 실효값의 약 1/67
- 결과: 단계 8에서 조향이 0.15에 계속 고착, 오프라인 폐루프는 0.4 rad에서만 11_18_6 코너 통과(0.15는 STOP)

### 2. LB 수동 복구 · A 비상정지 (safety_mode = MPC 모드 기본)
| 버튼 | 이전(0003) | 지금(0004) |
|---|---|---|
| RB | 자율(새로 누름 + 유지) | 동일 |
| LB | E-stop 래치 | **기존 수동조작**(유지하는 동안, 기존 매핑: 왼쪽 스틱 Y × `joy_max_speed`, 오른쪽 스틱 X × `joy_max_steer`), RB보다 우선(기존과 동일). 놓으면 none(속도 0), RB 자율 재진입은 새로 눌러야 함 |
| A | E-stop 래치 | 동일 (유일한 정지 버튼) |
| joy 끊김 | 자율 중 래치 | 자율 **또는 수동** 중 래치 |
- 래치 중: 자율·수동 모두 차단(출력 속도·조향 0 지속), 재시작 = 버튼 모두 놓음 + 바퀴 정지 1 s + 래치 2 s 경과 후 `/simple_mux/reset_estop` 승인 → RB/LB 새로 입력
- `safety_mode=false`(PP 기본) 경로는 기존 코드 그대로(수동 매핑만 함수로 묶음, 동작 동일)
- 수동 모드 출력에도 TTC estop(`use_estop`) 적용 — 기존과 동일

### 3. 18456 ERPM 급가속 분석 (단계 8 P7)
- 계산: `speed = axes[1] × joy_max_speed = 1.0 × 4.0 m/s` → ×4614 = **18456 ERPM**. 버그가 아니라 기존 수동 설계값(`low_level.launch.xml` `joy_max_speed` 기본 4.0, race.launch에서 바꾸는 인자 없음)
- 당시 LB를 정지 버튼으로 생각하고 누른 상태에서 왼쪽 스틱이 끝까지 전진 → 무부하(바퀴 공중)라 약 2 s 만에 실제 13781 ERPM
- 지금 구조에서도 LB 수동은 같은 속도(4.0 m/s)까지 낼 수 있음. 정지는 **A**
- **변경안 (사용자 결정 필요, 적용 안 함)**:
  | 안 | 내용 | 영향 |
  |---|---|---|
  | A | 유지 4.0 m/s | 기존과 동일. bench에서 스틱 실수 시 18456 ERPM |
  | B | MPC 모드만 `joy_max_speed` 1.0 (= MPC lowspeed) | 수동 최대 4614 ERPM. race.launch에 인자 전달 1줄 추가 필요 |
  | C | MPC 모드만 2.0 | 최대 9228 ERPM |

### 4. MPC 설정 확인 (변경 없음)
- `control_loop_rate_hz` 100, `mpc_horizon` 10, `mpc_max_iter` 4000, `mpc_solver_time_limit_s` 0.004, `mpc_profile` lowspeed 1.0 m/s
- GitHub `jsnd_int/` 사본 ↔ Jetson 소스: `controller_manager.py`, `Controller.py`, `integrated_mpc.py`, `race/low_level.launch.xml`, `mpc_car.yaml`, `mpc_profile_*.yaml` 동일. 차이는 이번 수정 2파일뿐
- JSND 원본 `~/JSND/src/racing_stack` `2091417` + 기존 미커밋 launch 2건 그대로

### 5. 검증 (실차 모터 구동 없음)
- 오프라인 `tests_offline/test_stabilization.py -k gate`: **9 passed** (A로 자율·수동 래치, LB 유지=수동·RB보다 우선·놓으면 0, 래치 중 LB 차단·LB 누른 채 재시작 거부, 수동 중 joy 끊김 래치, 버튼 중복 거부)
- ROS 모의 `tests_ros/run.sh mpc estop` (domain 219, VESC 없음, 요약 `docs/logs/ros_mock_estop_20261122_step10.txt`):
  | 이벤트 | 결과 |
  |---|---|
  | 시작 래치 / 래치 중 RB | 0.000 m/s |
  | RB 유지 / 놓음 | 자율 0.98 m/s / none |
  | LB + 스틱 0.25 | humandrive, mux **1.000 m/s** (0.25×4.0) |
  | LB 놓음 | none, 0 |
  | LB 수동 중 A | **0.052 s** 후 estop, 래치 동안 속도·조향 0.000 |
  | joy 끊김 / 소프트 estop | 0.50 s / 0.052 s 래치 |
  | 주기 | mux 100.1 Hz, controller 100.1 Hz, MPC p99 3.0 ms |
- 이미 통과한 MPC 안정화 시험(단계 9)은 반복하지 않음
- 종료 시 rclpy 이중 shutdown Traceback은 SIGINT 종료 시 기존 코드에서도 나는 것(동작 무관)

## 2026-11-22 단계 11: 실차 점검 준비 (코드·파라미터 변경 없음, 모터 구동 없음)
### 환경 확인
- GitHub `fd4bf56`, JSND_INT `701b49a`, install = src (`safety_gate.py`, `simple_mux_node.py`, `integrated_mpc.py`, `mpc_car.yaml` 비교), 미커밋 없음
- DOMAIN 173, `/dev/input/js0` 있음, **`/dev/ttyACM0` 없음, Hokuyo 192.168.0.10 무응답** → 실차 점검 보류
- ROS 모의 100 Hz·MPC 시간(단계 9·10)은 통과 항목이라 반복 안 함

### 조향 검토 보강 (단계 10 표에 추가)
- 서보 캘리브 이력: `steering_angle_to_servo_gain -0.99` = JSND 커밋 `7bb5f0e`(2026-08-20, "아직 yaw값 수정중"), `offset 0.4870` = `247b4ed`(2026-08-06, "Fixed Steering agnle"). 서보 끝점·바퀴 각도 실측 기록 없음
- 판단: 실차에서 사용 이력이 있는 0.4 rad(FTG·수동)가 1랩용 근거 있는 후보. 변화율은 PP가 사실상 무제한이라 기준 없음 → 서보만 움직여 실측 필요. **변경은 사용자 승인 후**

### LB 수동 최고속도
- 스틱 끝까지 = 4.0 m/s = 18456 ERPM. MPC 저속(1.0 m/s)의 4배, 스틱 실수 시 그대로 급가속. 변경 안 함 (안 A/B/C 결정 대기)

### 다음 실차 점검 (최소 명령어) — 전원 투입·스탠드 고정·4바퀴 공중·배터리 차단 담당 확인 후
```bash
source /opt/ros/jazzy/setup.bash && source ~/JSND_INT/install/setup.bash
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc use_estop:=true > ~/bench_logs/race_$(date +%H%M).log 2>&1 &
# 노트북 RViz 2D Pose Estimate
python3 ~/JSND_INT/tests_ros/bench_check.py 10      # 읽기 전용
# 사용자 승인 후: ros2 service call /simple_mux/reset_estop std_srvs/srv/Trigger → RB 유지 → A
```
| 항목 | 기준 |
|---|---|
| controller / mux 주기 | ≈100 Hz, p99 < 15 ms |
| MPC 계산 | p99 < 4 ms (시간 한도) |
| mpc_status | solved 대부분 |
| controller 속도 (RB 전) | 0.000 |
| estop_state | 시작 `latched: startup…`, A 후 `latched: button [0]` |


## 2026-11-22 단계 12: MPC 조향 0.4 rad · MPC 모드 LB 수동 1.0 m/s (모터 구동 없음)
JSND_INT `701b49a → 91d0e26` (`stack_master/config/mpc_car.yaml`, `stack_master/launch/race.launch.xml`만), 이 브랜치 `jsnd_int/0005-*.patch` + 사본. 재빌드: `stack_master`만. JSND 원본·PP·FTG 코드 변경 없음.

### 1. 조향
- `mpc_max_steer_rad` 0.15 → **0.4** (= FTG `ftg_max_steer`, 수동 `joy_max_steer`). servo = 0.487 ∓ 0.99×0.4 = **0.091 ~ 0.883** → VESC servo_min/max 0~1 안(0~1 ⇔ +0.492/−0.518 rad)
- simple_mux 조향 클립(`_clip`)은 기존부터 주석 처리 → mux는 조향을 자르지 않음(확인). 서보 클램프는 VESC 드라이버(0~1)
- `mpc_max_steer_rate_radps` **0.3 유지**. 오프라인 11_18_6 1랩(1.0 m/s, 이상적 운동학 모델): 최대 |조향| 0.336, 변화율이 0.3에서 포화, 최대 횡오차 0.052 m (3.2 rad/s와 동일 수준). 서보 지연·위치추정 튐은 모델에 없음 → **실차 검증 필요**
- 서보 기계 끝점·바퀴 실제 각도 **미측정**

### 2. LB 수동 최고속도
- race.launch `joy_max_speed` 인자: `controller:=mpc` → **1.0 m/s**(스틱 끝 4614 ERPM), 그 외 4.0(low_level 기본값과 동일 → PP 동작 불변)
- launch 해석(프로세스 미실행): mpc → mux `joy_max_speed 1.0`, safety_mode True, 100 Hz / controller 조향 0.4·0.3, 100 Hz, N=10, vmax 1.0. pp → `joy_max_speed 4.0`, 50 Hz
- ROS 모의 `tests_ros/run.sh mpc lbmax`(domain 219, VESC 없음, LB+스틱 1.0): mux 최대 **1.000 m/s**, A 래치 0.055 s, mux/controller 100.1 Hz, MPC p99 2.46 ms, solved 2394/2395(deadline 1건)

### 3. 시험
- 오프라인 `test_mpc_integration.py -k "step12 or bench_lifted"`: 4 passed
- 참고: `test_stabilization.py` 2건(`held_steering_worsens…`, `drive_windows_no_osqp…`)은 0.15 조건에서 기록된 bench 입력 재생 진단이라 CAR 설정 0.4를 읽으면 실패. lowspeed+0.4 재생은 OSQP 실패 0건, exp2mps(2.0, bench 전용)+0.4는 368틱 중 1틱 4 ms 시간한도(해당 틱 정지). 시험 고정은 사용자 지시로 중단(미수정)

## 2026-11-22 단계 13: Pitwall MPC 실시간 성능 모니터 (모터 구동 없음)
JSND_INT `91d0e26 → 8c35270`, 이 브랜치 `jsnd_int/0006-*.patch` + 사본. controller_manager·simple_mux·PP·FTG 변경 없음.
- `stack_master/scripts/mpc_monitor_node.py`: 별도 프로세스, controller가 이미 발행하는 토픽만 구독 → `/pitwall/mpc_monitor`(String, **2 Hz**). race.launch `controller:=mpc`에서만 기동(`target_hz` = `mpc_rate_hz`)
  | 항목 | 출처(실측) |
  |---|---|
  | 설정 주기/Hz | `mpc_rate_hz` (controller와 같은 값) |
  | 실제 주기/Hz | `/controller/mpc_loop_period_ms` 평균, p99, 최대 (최근 10 s) |
  | 제어 사이클 계산 | `/controller/mpc_cycle_ms` (MPC 포함 전체) |
  | MPC 계산 | `/controller/mpc_elapsed_ms` 최근값, p99, 최대 (10 s) |
  | OSQP 상태 | `/controller/mpc_status`: Solved / Failed(사유), 10 s·누적 개수 |
  | 주기 초과 | 누적: 계산 > 설정주기, 실제 주기 > 1.5×설정주기 |
  | 목표/실제 속도 | `/vesc/high_level/ackermann_cmd` speed / `/car_state/odom` |
  - 1 s 이상 데이터 없으면 해당 항목 N/A
- pitwall 패널(`race_utils/pitwall`): Telemetry 아래 "MPC monitor" 고정폭 텍스트. 수신 시에만 갱신(~2 Hz), 3 s 끊기면 N/A. 기존 기능 변경 없음
- 검증: `stack_master`·`pitwall` 빌드 성공(JSND_INT 오버레이). 모니터 노드 단독(domain 219): 데이터 없음 → 모두 N/A, 주입값 → 주기·p99·초과·실패 개수 정상
- 노트북 반영: 노트북 작업본에 이 사본(pitwall 2파일)을 넣고 READMEforlaptop 146행의 `colcon build --packages-select stack_master pitwall` 재빌드 필요. 노트북이 relay(domain 174) 경유면 `/pitwall/mpc_monitor`를 relay 화이트리스트에 추가해야 함(173 직접 접속이면 불필요)

## 2026-11-22 단계 14: 실차 무구동 점검 (래치 유지, 모터 0, E-stop 해제 없음)
- 장치: VESC `/dev/ttyACM0`, F710 `js0`, LiDAR ping OK. JSND_INT = install 확인
- 100 Hz 실측(전체 스택, load 13/6코어, MAXN_SUPER): 모니터 켬 controller 74~78 Hz·MPC p99 14.5 ms·deadline 12% / 모니터 끔 89 Hz·p99 11.3 ms·deadline 4% → **100 Hz 불가**. rclpy 모니터 CPU ~27%
- 사용자 결정 → JSND_INT `22ab8de`: race.launch MPC 기본 **50 Hz**, `mpc_deadline_s = 0.8/rate`, `mpc_solver_time_limit_s = 0.4/rate` (100 Hz면 기존 8/4 ms, 50 Hz 16/8 ms), `mpc_monitor:=false` 기본(켜려면 `mpc_monitor:=true`)
- 50 Hz 실측(래치): controller **50.2 Hz** (p50 19.9, p99 28.5 ms), mux 100.2 Hz, MPC p50 0.77 / **p99 5.64** / max 7.74 ms, load 7.5. mpc_status `outside_heading_envelope` 100% → PF 초기 자세 미지정(RViz 2D Pose Estimate 필요). 속도 0, `latched: startup`

## 2026-11-22 단계 15: MPC 최고속도 1.5 m/s 프로파일
- 사용자 요청(1.0 x 1.5). JSND_INT `e53b31a`: `config/mpc_profile_1p5.yaml` 추가(`mpc_max_speed_mps` 1.5만). 조향 0.4/0.3, 50 Hz, LB 수동 1.0 그대로
- MPC 파라미터는 실행 중 변경 불가 → launch 재기동 `mpc_profile:=1p5`. 재기동 확인: max_speed 1.5, 50 Hz, 래치, 모터 0. 2D Pose Estimate 재지정 필요
- 참고: 횡가속 한도 2.0 m/s² → 곡률 1.11 코너는 MPC가 약 1.34 m/s 이하로 감속
