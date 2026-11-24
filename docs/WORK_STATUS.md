# WORK_STATUS

> **현재 상태 요약 (2026-11-23 단계 18 — Step 17 실차 빌드 적용, 3/4/5 m/s 프로파일·실행/멈칫 로그 스크립트 준비, 모터 구동 없음)**
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

## 2026-11-22 단계 16: 2.25 m/s 프로파일, RB 한 번 누르기, 노트북 MPC 패널 설치, 코너 멈칫 진단
JSND_INT `e53b31a → 9e4cce3` (`jsnd_int/0008-*.patch` + 사본). controller·MPC 코드 변경 없음.
- `config/mpc_profile_2p25.yaml`: `mpc_max_speed_mps` 2.25 (1p5 × 1.5). 실행 `mpc_profile:=2p25`. 지면 검증 전
- race.launch `safety_engage_hold` (기본 **false**) → simple_mux `safety_require_engage_hold`: RB 한 번 누르면 자율 유지. 해제 = A(래치, `reset_estop` 필요) / LB(수동, 떼면 0) / joy 끊김. 이전 동작은 `safety_engage_hold:=true`
- Jetson 재부팅 후 복구: 남은 프로세스 없음. Wi-Fi iPhone 핫스팟 172.20.10.4, 노트북 172.20.10.3 ping OK, Jetson→노트북 SSH 불가(22 닫힘)
- 노트북 pitwall MPC 패널: `jsnd_int/laptop_pitwall_mpc/install_pitwall_mpc_panel.sh`(단계 13 pitwall 2파일 내장, thirdimpact alias로 작업공간 탐지, 원본 해시일 때만 교체+백업, `pitwall`만 빌드). 노트북에서
  `ssh jsnd@172.20.10.4 'cat JSND_INT/laptop_pitwall_mpc/install_pitwall_mpc_panel.sh' | bash` → `thirdimpact` → `pitwall`. Jetson 모의 작업공간에서 2회 실행 확인, 노트북 실제 실행은 미확인. 데이터 없으면 N/A(기존 동작)
- 모니터 CPU 절감 검토(미적용): 노트북에서 모니터 노드 실행(권장) / EventsExecutor / C++ 재작성
- 실차 주행(2p25, 약 145 s, 로그 `race_mpc_20261122_152621.log`): MPC STOP 303회(~2회/s) — deadline_exceeded 145, nonlinear_speed_profile_violation 79, OSQP inaccurate/time limit/infeasible 57, invalid_local_control_input 21. 한 주기 실패 → 속도 명령 0(전류 제동)+조향 0 복귀 → 코너 멈칫·조향 끊김의 원인. 최대 코너 곡률 1.11 → 필요 조향 ~0.35 rad < 0.4라 조향각 상한을 낮추면 악화. 조향 속도 0.3 rad/s가 느림(0→0.35 rad 1.2 s)
- 사용자 결정 대기: (1) 짧은 실패(≤0.1 s)는 직전 계획 이어쓰기 후 정지 (2) 조향 속도 상향(바퀴 들고 서보 측정 선행) (3) 횡가속 2.0→1.5

## 2026-11-22 단계 17: 코너 멈칫 수정 (진행 중, 실차 주행 검증 전)
JSND_INT `586ba13` (`jsnd_int/0009-*.patch` + 사본). 50 Hz·N=10·2.25 m/s 유지, PP/FTG/LB·JSND 원본 무변경, 안전 정지(경로 이탈·센서·장애물) 유지.
- 원인(로그 `race_mpc_20261122_152621.log`): 한 주기 실패 → 속도 0+조향 복귀가 멈칫의 직접 원인.
  - deadline/OSQP 시간초과·부정확: CPU 과부하(load 13~21/6코어)로 선점. 정지 상태에서도 elapsed p99 10 ms, 최대 17.7 ms. 오프라인 계산은 p50 2.8 / p99 5 ms
  - 속도 프로파일 위반: QP 상한은 기준 샘플 위치, 사후 검증은 실제 예측 위치 → 예측이 약간 앞서 낮은 구간에 들어가면 거부. 첫 단계 감속 허용 시점 오차(시작 시각)
  - primal infeasible: 코너 탈출 중 조향 속도 0.3 rad/s로 조향이 늦게 풀리는데 속도 상한은 직선 기준 → 조향 박스가 비어 버림
  - invalid_local_control_input: STOP 급제동 직후 발생(속도 역방향 추정). 이제 사유를 상태에 기록
- 수정(`integrated_mpc.py`):
  1. 프로파일 위반 시 해당 단계 상한만 실제 위치 기준으로 낮춰 최대 2회 재풀이. 사후 검증은 그대로 엄격
  2. 조향 속도로 도달 가능한 최소 조향각 기준 속도 상한(조이기만 함)
  3. 감속 도달 범위를 단계 종료 시각 기준으로(조이기)
  4. 계산 예산 실패(deadline·시간초과·max iter·inaccurate)에 한해 직전 검증 계획의 다음 단계를 최대 2틱(40 ms) 사용. 측정 상태가 계획 예측과 위치 0.10 m·방위 0.15 rad·속도 0.30 m/s 이내일 때만, 조향 속도·조향 상한·단계 속도 상한·횡가속 재검사. 상태 `plan_continuation:<사유>`로 표시. 경로·외란·입력·실현불가 실패는 그대로 STOP
- 검증(오프라인 최소): `tests_offline/test_corner_stall.py` 6개 통과. 실차 입력 주행구간(2p25·50 Hz): 프로파일 STOP 16→1(남은 1건은 실제 과속, 정상 정지). 코너 탈출 시나리오 primal infeasible 20/20 → solved(횡가속 한계 준수). 실차 로그 추정: 계산 실패 167건 중 137건(82%)이 2틱 이하 → 이어쓰기 대상
- 기존 테스트: 단독 실행 22/22·전체 37/40. 실패 3건은 부하 하 타이밍 테스트(스택 동시 실행, load 19~21)
- 미완료: 실차 재기동 확인 중 Jetson에 JSND 원본 race.launch(노트북 SSH pts/1)가 동시에 떠서 VESC 드라이버 2개 → 내 스택 종료. 새 코드 실차 정지 상태 확인·주행 검증 안 됨
- 남은 문제: CPU 과부하가 근본. controller 실시간 우선순위(limits.conf rtprio, root 필요) 또는 스택 부하 감축 필요. 2틱 넘는 계산 지연과 실제 과속은 계속 STOP

## 2026-11-23 단계 18: Step 17 실차 적용·빌드, 2.25→3→4→5 m/s 준비 (모터 구동 없음)
- Jetson JSND_INT 소스 = GitHub `ee57524` 코드 일치 확인. 백업 `~/JSND_backups/step17_20261123_122457` 후 `controller`·`stack_master` 재빌드 성공.
- JSND_INT `6545daa` (`jsnd_int/0010-*.patch` + 사본): `mpc_profile_3p0/4p0/5p0.yaml` 추가. MPC 최고속도만 변경. 조향 0.4 rad/0.3 rad/s, 가속 1.0, 횡가속 2.0 m/s²(코너 속도 제한 그대로), PP/FTG, A 래치, LB 1.0, RB 자율 무변경. 5 m/s = 23070 ERPM(VESC speed_max 23250).
- 실행 `bash ~/JSND_INT/run/race_mpc.sh <2p25|3p0|4p0|5p0>`: tmux `race`, 다른 race 스택·VESC 드라이버가 있으면 거부, 로그 `~/bench_logs/race_mpc<프로파일>_<시각>.log`(커밋·프로파일 기록).
- 멈칫 비교 `python3 ~/JSND_INT/run/stall_summary.py <로그...>`: STOP(속도 0) 분당 횟수·원인, 이어쓰기 건수. 수정 전 2.25 m/s 기준 STOP 43.2회/분(정지 시간 포함, 대부분 deadline_exceeded).
- 검증: 실행 스크립트 건식(VESC 없음) 환경·따옴표 확인. 프로파일 파라미터 로드는 실차 launch에서 확인 필요. 실차 주행은 사용자 직접.

## 2026-11-23 단계 19: 전원 차단 복구, 실행 스크립트 보강, 속도 단계 측정 도구 (Claude는 모터 구동 없음)
- 12:54 Jetson 전원 차단(정상 종료 기록 없음) 후 재부팅. 맵 `11_18_6`·소스·설치본 무손상, 재빌드 불필요. VESC 휴지 전압 11.8 V(셀 수 미확인).
- `race_mpc.sh`: `unknown profile`은 `$HOME` 불일치(sudo/원격 실행) 경우로 추정. 스크립트 위치 기준으로 설치 경로·맵을 찾고, root·Jetson 외 실행은 거부, 맵 존재 사전 검사.
- 2p25 스택 기동 확인: `/map` 225×142, `/scan` 40 Hz, PF 40 Hz, odom 50 Hz, MPC `max_speed=2.25` 로드. 사용자 보고: 2.25 m/s 멈칫 해결.
- 신규 `speed_rec.sh <프로파일>`(tmux `rec`, mcap bag `~/bench_logs/bags/`)와 `speed_report.py <bag> --vmax N`: 자율 구간 횡오차 |d|, 명령 대비 속도, 제동 이벤트(명령 0→실속 0.1 m/s 미만까지 거리·시간·감속, 3/4/5 m/s 예상 제동거리), 조향 최대·0.4 포화율·부호반전·yaw rate, MPC 계산시간 p99, 다음 단계 판정(잠정: |d| p95<0.25, max<0.5 m, 포화<5%, 반전<60/분, MPC p99<16 ms, 목표 90% 도달).
- 프로파일·가감속·횡가속·PP/FTG·안전 기능 무변경. 정지 상태 6초 기록으로 기록·파싱만 확인, 주행 지표는 실차 데이터로 미검증.

## 2026-11-23 단계 20: 3p0 출발 불가·수동 후 재출발 진단 (코드 변경 없음)
- 3p0 실행 로그에서 MPC `max_speed=3.0` 로드 확인. 5p0 설치본=소스(`mpc_max_speed_mps: 5.0`), 실행 `bash ~/JSND_INT/run/race_mpc.sh 5p0`.
- 기동 직후 속도 0: (1) 기동 래치 상태에서 `reset_estop`·RB 없음 → `active_host=estop` → controller_manager가 MPC 속도 0 고정. (2) 차량이 레이싱라인에서 0.90 m(한계 1.0)·방위 38.5°(한계 0.7 rad=40.1°, MPC 투영 기준 초과) → `outside_heading_envelope` STOP. 라인 위 정렬 + 2D Pose Estimate로 해소.
- LB 수동 → 해제(none) → RB 한 번 재선택은 설계대로 동작(로그 :114→:120→:131, 래치 없음). 재출발 안 된 원인은 상태머신 TRAILING(장애물 s≈9.27, d≈0.23, 0.15 m 감지) → MPC `unsupported_behavior:TRAILING` STOP. 주행 중 짧은 정지는 TTC EStop(0.35 s, 비래치).
- 사용자 결정 대기: MPC의 TRAILING 지원(감속 추종) 여부. 그 전까지 TRAILING=STOP 유지.

## 2026-11-23 단계 21: TRAILING 고착 해소 (MPC TRAILING 지원, 오래된 장애물 제거)
- 원인: 트랙 한 바퀴 13.9 m < `interest_horizon_m` 20 m. 차량 뒤 1.4 m(s≈9.27, d≈0.25, 0.21 m, 정적, is_visible) LiDAR 검출이 한 바퀴 앞 12.3 m 장애물로 계산 → GB 경로 막힘(static/geom free_dist 0.024) → TRAILING. 맵상 빈 공간(벽 아님), 좌측 경계에서 약 8 cm. 추적 데이터는 실시간(오래된 데이터 아님). 기존 MPC는 TRAILING을 `unsupported_behavior`로 STOP → PP는 같은 상황에서 정적 장애물 쪽으로 접근(회피 경로 대기)하지만 MPC는 영구 정지.
- 수정(JSND_INT `509f4ae`, `jsnd_int/0011-*.patch`):
  - `Controller.py`: TRAILING에서 MPC가 local path를 추종하되 모든 waypoint 속도 상한을 PP와 같은 `trailing_controller` 명령으로 제한(동적: 간격 PID, 정적: standoff 0.5 m 정지·크롤 0.2 m/s). 대상이 없으면 `trailing_without_target` STOP(PP의 0 명령과 동일). TRAILING 이탈 시 정적 정지 래치 초기화.
  - `state_machine`: `/tracking/obstacles`가 `obstacle_stale_timeout_s`(기본 1.0 s, 0 이하=비활성) 넘게 끊기면 장애물 목록 비움(경고 1회). 살아 있는 tracker 결과는 그대로 사용.
- 무변경: 장애물 검출·TTC EStop·A 래치·기동 잠금·PP/FTG 경로·5p0 프로파일·가감속·횡가속.
- 테스트: 신규 `tests_offline/test_mpc_trailing.py` 6개 + 기존 `test_mpc_integration.py` 통과(18/18), state_machine 기존 56/56. 오버레이에 `controller`·`state_machine` 빌드(state_machine 원본=JSND 일치 확인), 백업 `~/JSND_backups/step21_20261123_134113`.
- 실차 확인(5p0, 래치 유지, 모터 0): `max_speed=5.0` 로드, MPC가 TRAILING에서 거부 대신 계산 → 현재 `outside_tracking_envelope`(차량이 레이싱라인에서 1.08 m, 한계 1.0 m). 레이싱라인 위 정렬 + 2D Pose Estimate 필요. 검출 물체(차량 뒤 1.4 m, 좌측 벽 근처)는 실물 확인 필요.

## 2026-11-23 단계 22: 5 m/s 요청 검토 (코드 변경 없음, PP로 전환)
- 트랙 13.9 m, 곡률 max 1.11 / p90 0.86 1/m → 코너 속도 한계: 횡가속 2.0이면 1.3~1.5 m/s, 4.0이면 1.9~2.2, 8.0이어도 2.7~3.1 m/s. 웨이포인트 vx 최대 3.9(평균 2.86), Sector1·3 배율 0.6. 5 m/s는 이 트랙에서 불가.
- 오프라인 폐루프(11_18_6 raceline): 5p0·fast2(웨이포인트 속도 무시, 횡가속 4.0, 가속 3.0) 모두 약 2 m/s로 첫 급코너(s≈3 m, κ≈1) 진입 시 MPC 조향 증가가 늦어 바깥으로 밀림 → `nonlinear_prediction_outside_envelope` STOP. 조향속도 0.3→3.2 rad/s, envelope를 횡방향 거리 판정으로 바꿔도 동일 → 원인은 MPC 코너 진입(비용 가중치/참조 샘플링 추정, 미확인).
- fast2·옵션 코드 초안은 원복(`~/JSND_backups/step22_fast2_draft`), 빌드·배포 없음.
- 사용자 결정: 당일은 PP 주행. 권장 실행 `race.launch.xml map:=11_18_6 controller:=pp use_estop:=true safety_mode:=true joy_max_speed:=1.0`(A 래치 유지, LB 1.0). PP 속도 = 웨이포인트 vx × 구간 배율.
- 다음: MPC 코너 진입 조향 지연 수정(오프라인 랩 검증 필요).

## 2026-11-23 단계 23: 구조 재검토 — JSND 속도 계획 + 조향 전용 MPC (JSND_INT `a3396c2`, 패치 0012)
- 원인(코드·오프라인 로그로 확인): 기존 IntegratedMPC가 JSND 속도 계획(iqp raceline × 섹터 배율, ggv ay 4.5)을 hard 상한으로 쓰면서 속도를 재최적화 → 중복 제한(횡가속 2.0, 0.85 soft, 제동 preview 0.6 m/s², 조향속도 cap, 횡가속 연동 조향 bound, 가속 적분·lead 0.5). 섹터 경계 속도 계단(2.44→1.43, 2.79→1.64 m/s)에서 `nonlinear_speed_profile_violation` STOP. 조향속도 0.3 rad/s(벤치 값)가 조향 위상 지연(~460 ms)·레이싱라인 이탈(최대 0.9 m)의 원인. 파라미터 전부 완화해도 STOP 24회 → 구조 문제.
- TRAILING/OVERTAKE 이상: 실차 로그 `blocked_obs_id=25 gap=12.5m` — interest 20 m·gb 15 m horizon > 트랙 13.9 m라 차 뒤 1.4 m 물체가 12.5 m 앞으로 계산됨(PP도 동일 영향). OVERTAKE도 `enemy_in_front`가 항상 참.
- 변경:
  - `mpc_speed_source: jsnd`(기본): 속도 = 기존 PP 종방향 로직 그대로(lookahead, 횡오차/heading 보정, trailing, AEB, 전 상태). MPC(`mpc/steering_mpc.py`)는 조향만(15단계, 0.72 s preview, 조향각 0.4·조향속도 제약). `integrated`로 기존 MPC 선택 가능.
  - 유지 안전기능: 경로 1 m/heading 0.7 rad 이탈 즉시 STOP, odom stale·mux·웨이포인트 워치독·TTC EStop·A 래치, mpc_profile 속도 상한(운용 상한, 5p0에선 미발동), 계획 점별 물리 횡가속 4.5(ggv) 확인(실현 가능한 계획에선 no-op). 계산 실패는 최대 0.2 s PP 조향 대체 후 STOP(성공 전까지 STOP 유지).
  - `mpc_max_steer_rate_radps` 0.3→3.2(F1TENTH sv_max, **실측 아님**).
  - state_machine: interest/gb horizon을 `track_length − obstacle_rear_exclusion_m(2.0)`으로 제한.
- 오프라인 11_18_6(같은 속도 계획, 2랩, `tests_offline/lap_sim.py`; 서보·VESC 모델은 가정값):

| | 완주 | 플라잉랩 | 평균/최고 속도 | 최대 횡오차 | STOP |
|---|---|---|---|---|---|
| PP | 2/2 | 6.42 s | 2.05 / 3.70 | 0.187 m | 0 |
| 기존 MPC(5p0) | 2/2 | 11.84 s | 1.40 / 2.28 | 0.899 m | 16 |
| 신규 MPC | 2/2 | 6.84 s | 2.02 / 3.69 | 0.019 m | 0 |

  - 랩타임 차이는 속도가 아니라 경로: PP는 코너 안쪽 평균 0.11 m 컷(주행 13.11 m), MPC는 레이싱라인 추종(13.77 m).
  - 서보 지연 60 ms·VESC τ 0.25·위치잡음 2 cm·초기 오프셋 0.3 m/0.2 rad에서도 STOP 0, 완주.
- 테스트: tests_offline 55/55(신규 `test_jsnd_mpc.py` 9개 포함), state_machine 56/56. 기록용 테스트 3개 파일은 당시 조향속도 0.3을 명시 고정하도록 수정(재현 대상 유지), step12 단언은 3.2로 갱신. 오버레이 빌드 완료(controller, stack_master, state_machine). 백업 `~/JSND_backups/step23_20261123_140844`.
- 미해결:
  - ROS 모의 시험(`tests_ros/run.sh mpc safety`) 미실행(사용자 중단), 실차 미검증.
  - 서보 실제 조향속도·지연 미측정(3.2 rad/s는 가정). 실측 후 낮아야 하면 조정 필요.
  - 차량 0.12~0.16 m 앞 정적 검출(id 25, s≈1.06)이 실물인지 오검출인지 미판별.
  - 5 m/s는 속도 계획 최대 3.9 m/s라 불가 — 더 빠르게는 섹터 배율/레이싱라인 재생성 필요.
  - 실차 출발은 자동 실행하지 않음.

## 2026-11-23 단계 23 Jetson 배포
- GitHub `88960e1` = Jetson 소스 일치 확인, 백업 `~/JSND_backups/step23_deploy_20261123_142725`, 오버레이 빌드(controller, stack_master, state_machine), 설치본=소스 일치.
- ROS 모의 시험에서 발견: `controller_manager.global_wpnts_cb`가 `__init__` 지역변수 `mpc_values` 참조 → `/global_waypoints` 수신 시 NameError로 노드 종료(오프라인 시험은 노드를 거치지 않아 미검출). `self.mpc_values`로 수정(JSND_INT, 패치 0013). 튜닝·구조 변경 없음.
- 수정 후 모의 시험(domain 219, VESC 없음):
  - `run.sh mpc safety safety_mode:=false`: 전 구간 기대대로(lowspeed 1.0 m/s 주행, odom/웨이포인트 끊김·TRAILING 무대상 0, 컨트롤러 종료 후 mux 0), 100 Hz, MPC solve p99 1.8 ms, 2197/2197 solved. `scan_obstacle`은 use_estop 기본 off라 mux 0이 아님(PP 동일).
  - `run.sh mpc estop`(safety_mode on): 시작 래치·리셋·RB 출발·A/조이 끊김/soft estop 시 0 m/s 정상, 2398/2398 solved.
  - 참고: `safety` 시나리오는 safety_mode 이전 시나리오라 기본값(on)으로 돌리면 시작 래치로 속도 0(설계대로).
  - 시험 종료 시 SIGINT로 인한 `context is invalid` traceback은 종료 시점 아티팩트(PP·simple_mux 동일).
- 실차 미출발.

## 단계 24 (2026-11-23) — 실차 3p0 조향 이상 분석 (JSND_INT `f3326a1`)
- 실차 로그 `race_mpc3p0_20261123_143721.log`: RB 후 1.08 s, 1.76 m/s에서 `outside_heading_envelope` STOP → TTC EStop. 출발 전 정지 중에는 MPC solved(초기 자세·방향 정상).
- `steering angle clipped`는 PP 내부 0.4 변화량 클립(PP 희망 조향과 MPC 조향의 차이)이며 MPC ±0.4 포화가 아님. 로그에 조향값·자세 기록 없음.
- 코드 비교: 조향 부호(+좌), `/drive`→VESC 경로, rear axle(오프셋 0), 경로 투영·yaw 오차 계산 모두 PP와 일치. 부호 반전은 오프라인에서 PP도 망가뜨리므로 제외.
- 오프라인 재현(`tests_offline/start_repro.py`, s=12.45 출발): 자세 지연 120~200 ms에서 MPC만 1.2~1.8 s에 heading STOP(PP 정상). **가설이며 실차 지연은 미측정.**
- 수정: `mpc_latency_comp_s: 0.1`(rear axle 자세를 적용 조향으로 0.1 s 앞 예측, 0이면 기존 동작) + STOP 상세 로그(자세/속도/MPC·PP 조향). 안전 기능·한계값 변경 없음.
- 시험: 재현 시험 지연 0/120/200 ms 모두 STOP 0, 랩 3p0 MPC 7.04 s/STOP 0/횡오차 0.024 m(PP 6.42 s), `tests_offline` 55 passed, 오버레이 재빌드.
- 다음: bench(바퀴 띄움)에서 `/car_state/odom` 지연 측정 → 값 확정 후 저속 프로파일부터 실차 재시험.

## 단계 25 (2026-11-23 14:51~15:07) — 실차 5p0 주행, pit 화면 계산 시간 표시 (코드 변경 없음, JSND_INT `f3326a1`)
- 맵 미표시 원인: 14:37 스택(3p0)이 Ctrl-C로 종료돼 `map_server` 부재. 재기동 절차: `tmux kill-session -t race` 후 `bash ~/JSND_INT/run/race_mpc.sh <profile>`를 단독 명령으로(죽은 `race` 세션이 남아 있으면 ABORT). `/map`은 기동 시 1회 발행 → 나중에 켠 RViz는 Map Durability=Transient Local 필요, 재기동마다 2D Pose Estimate 재지정.
- 14:53 3p0 재기동(`/map` 225×142 수신 확인) → 15:00 사용자 요청으로 5p0 재기동(`mpc_max_speed_mps`=5.0 확인, VESC 드라이버 1개, 시작 시 estop 래치).
- 5p0 로그 `race_mpc5p0_20261123_150006.log`(402 s, `stall_summary.py`): STOP 7(모두 `invalid_local_control_input:invalid_or_reverse_speed`, v≈-0.02 정지 잡음), `outside_heading_envelope` 0회 → 단계 24 지연 보정 후 조향 발산 재발 없음. 계산 이어쓰기 31(`deadline_exceeded` 22, osqp 시간한도 2). 사용자 판단: 속도 튜닝 완료.
- pit 화면 계산 시간: 기존 `mpc_monitor_node.py`(단계 13)를 스택 재기동 없이 별도 tmux `mon`에서 `target_hz:=50`으로 실행 → `/pitwall/mpc_monitor`(2 Hz) → 노트북 pitwall 패널 "MPC monitor".
  - 정지 상태 측정(10 s): 설정 20 ms/50 Hz, 실제 주기 24.16 ms/41.4 Hz(p99 61.5, max 80.1 ms), 제어 주기 계산 9.85 ms(p99 42.9 ms), MPC 풀이 5.82 ms(p99 14.5 ms), solved 211/failed 1.
  - 모니터 CPU ≈ 32 %(1코어), Jetson load 15(6코어). 41 Hz 저하에 모니터 부하가 섞였을 수 있음(미분리). 끄기: `tmux kill-session -t mon`. 기동 시 함께 켜려면 launch 인자 `mpc_monitor:=true`(현재 `race_mpc.sh`는 미전달).
- 다음: 50 Hz 미달 원인 분리(모니터 on/off 비교) 또는 모니터 경량화(C++/저빈도 구독).

## 단계 27 (2026-11-24 18:20~18:40) — Step 26 실차 준비 점검·출발 불가 진단 (코드·파라미터 변경 없음, JSND_INT `7734321`)
- 코드: Jetson JSND_INT `7734321` = Step 26, 작업트리 clean, install = 소스. Hokuyo `192.168.0.10` ping OK. 배터리 3S 11.4 V(3.80 V/셀, 약 50%).
- `tests_offline`: 5 failed / 50 passed. 실차 영향 없음으로 판단 — `100hz_compute_budget`(100 Hz 예산, 실차 50 Hz), `bench_lifted`(오프라인 실측 시간 `deadline_exceeded`), `drive_windows`(11-22 기록·구 한계 재생, 기존 실패), `profile_refinement`(허용오차 2e-4 m/s 초과), `trailing_far_static`(60틱 후 0.276 < 0.3 m/s, TRAILING 출발 약간 느림).
- 4p0 integrated 래치 30 s(사용자 측정): loop mean 20.3/p99 33.1/max 82.8 ms, MPC mean 4.1/p99 13.8/max 29.9 ms, solved 1089, `trailing_without_target` 379, deadline 6. 주기 초과 원인 = CPU 경합(load 14.7/6코어, state_machine 82 %, controller 73 %, PF 45 %), MPC p99는 16 ms 마감 이내 → 주행 가능 판단.
- 출발 불가 원인 1 (직접 원인): 18:21:36 F710이 USB 재연결되며 **DirectInput 모드**(`046d:c219`)로 전환, `logitech: probe failed -1` → `/dev/input/js0` 없음 → `/joy` 미발행 → `simple_mux` `reset refused: no fresh /joy (pad connected?)`. 버튼 매핑(A=0/LB=4/RB=5)도 XInput 기준. 조치: 패드 뒷면 스위치 **X**, 수신기 재삽입(물리 조작, 사용자).
- 출발 불가 원인 2: `ObstacleTransition`(`state_transitions.py:133-170`)은 관심 장애물이 있고 |d| ≥ gb_ego_width 0.29 m이며 recovery/OT 경로가 없으면 GB가 비어도 TRAILING. 차 d=0.48 m 정지 + 오른쪽 벽 가장자리 검출(s≈9.96, d=-1.54, 폭 -1.66~-1.42, 트랙 d_right 1.70, 맵 xy≈(0.49, 1.76)) → TRAILING, `trailing_targets` 비어 MPC `trailing_without_target` STOP(PP도 0 출력, MPC 고유 문제 아님). 벽 검출 원인(PF 오프셋 vs 실제 벽 위치)은 미확정.
- 환경 비교: domain 173/cyclone/`cyclonedds.xml` 동일, 중복 노드·VESC 2중 없음. config·맵은 JSND와 동일(`mpc_car.yaml`만 추가). race_mpc.sh 차이는 의도된 MPC 기본값(시작 래치·`/joy` 필요 reset, 수동 1.0 m/s, `use_estop:=true`, 50 Hz).
- 재출발 절차: F710 X 모드 → `tmux kill-session -t rec`(스택 없이 남은 녹화) → `bash ~/JSND_INT/run/race_mpc.sh 4p0 11_18_6 integrated` → 차를 레이스라인 위(|d|<0.05 m, 20° 이내)에 두고 2D Pose Estimate → `/state_machine` GB_TRACK 확인 → reset_estop + RB(사용자).

## 단계 28 (2026-11-24 18:37) — 통합 MPC lowspeed 재기동, 초기 위치 지정 대기 (코드·파라미터 변경 없음, JSND_INT `7734321`)
- `race_mpc.sh` ABORT 원인: PID 8305 = tmux 서버(명령줄에 `race.launch.xml` 문자열 포함 → 중복 스택 pgrep에 걸림). `rec` 녹화 정상 종료(bag `4p0_20261124_182441` 보존) 후 `tmux kill-server`.
- 수동 `joy_node`는 정리 시점에 이미 종료 → 스택 내장 `joy_node` 1개만(`/joy` 발행자 중복 없음).
- 기동: `race_mpc.sh lowspeed 11_18_6 integrated`, 로그 `race_mpclowspeed_integrated_20261124_183702.log`.
- 상태: `/joy` 15.6 Hz, `/scan` 40 Hz(827빔, sensor QoS), VESC `/vesc/sensors/core` 50 Hz·드라이버 1개, `/state_machine` 50 Hz GB_TRACK, MPC 50 Hz·약 1 ms·`outside_heading_envelope`(초기 위치 지정 전, 정상), estop `latched: startup` 유지.
- 경고: VESC 입력 10.4 V(3S, 3.47 V/셀, 18:1x 11.4 V에서 하강) → state_machine `VOLTS TOO LOW`(volt_threshold 11.0, 로그·마커만, 정지 동작 없음). 주행 전 충전/교체 필요.
- 사용자 조작 순서: 배터리 충전·교체(교체 시 스택 재기동) → 차를 레이스라인 위(|d|<0.05 m, 20° 이내) → Pitwall 2D Pose Estimate(scan·맵 정합 확인) → GB_TRACK·`solved` 확인 → `reset_estop` → RB. 정지 A, 수동 LB.

## 단계 29 (2026-11-24) — 6p0/7p0 프로파일 준비 + 5p0 코너 반복 정지 원인 (실차 구동 없음, JSND_INT `6738aa7`)
- 추가: `stack_master/config/mpc_profile_6p0.yaml`, `mpc_profile_7p0.yaml` — 5p0 사본, `mpc_max_speed_mps`만 6.0/7.0. 가감속·횡가속·조향각/변화율·곡률 감속·비상정지·MPC 코드 무변경.
- 실행: `bash ~/JSND_INT/run/race_mpc.sh <6p0|7p0> 11_18_6 integrated`, 기록 `speed_rec.sh` → `speed_report.py <bag> --vmax N`(`/car_state/odom` 최고속도, 명령-실속 차이, `/vesc/sensors/core`).
- stack_master만 빌드, install에 6p0/7p0 존재·YAML 파싱·5p0 대비 차이(속도 상한 1줄) 확인.
- **주의 1**: VESC `speed_max` 23250 ERPM / 4614 = **5.04 m/s 하드웨어 상한**(무변경) → 6/7 명령은 모터에서 포화.
- **주의 2**: 11_18_6 iqp 참조 속도 최대 3.87 m/s(s≈0), 코너 2.07~2.25 → 이 맵에서는 5p0/6p0/7p0 실주행 최고속도 차이가 작을 것으로 예상.
- 5p0 코너 반복 정지(`race_mpc5p0_integrated_20261124_190832.log`, 352 s, STOP 216): ego_s 매칭 결과 **s≈9~11에 131회(61%)**(곡률 0.53~0.86, 두 번째 헤어핀 진입: outside_envelope·heading·osqp inaccurate), **s≈3에 26회**(곡률 0.9: deadline·braking_envelope). 원인 후보 = 고곡률 진입 시 QP 수렴 실패/계산 시간 초과 + 비선형 예측 envelope 이탈. 수정은 사용자 결정 대기.

## 단계 30 — 통합 MPC 코너 예측 개선 (JSND_INT `6530872`, 실차 미구동)
- 예측: 10×0.1 s → 12×0.05 s(0.02+11×0.05 = 0.57 s), 제어 50 Hz 유지. 로컬 창 80점≈7.9 m라 15×0.1 s·20×0.05 s는 경로 부족/예산 초과.
- OSQP 미수렴 해(solved inaccurate/max iter/time limit)는 기존 제약·비선형 envelope 검사를 모두 통과할 때만 채택(status `*_inexact_verified`), 실패 시 기존대로 계획 이어쓰기/STOP. 비상정지·프로파일 무변경.
- 오프라인 11_18_6 5p0(잡음 2 cm·0.05 m/s, 서보 60 ms, race.launch 예산): STOP 1→0, 횡오차 최대 0.162→0.089 m, MPC p99 11.4→8.5 ms, 랩 5.32→5.30 s. 6p0/7p0 STOP 0~1. r_rate 1/2/4/8 비교 → 2.0 유지.
- 비교 명령: `python3 jsnd_int/tests_offline/corner_sweep.py --real --noise "" mpc_horizon=10,mpc_preview_dt=0.1`.
- 알려진 차이: test_stabilization budget_miss(구형 max_iter 200)에서 정지 복구 7틱(실제 4000회 설정은 0~2틱).

## 단계 31 — 주행 중 불필요한 MPC STOP 수정 (JSND_INT `d3b20b9`, 실차 미구동)
- 원인: `IntegratedMPC.solve()`가 `/local_waypoints`가 직전 틱과 비트 단위로 다르면 이어쓰기 계획을 폐기. 주행 중 같은 레이스라인 창이 앞으로 밀리는 것도 "변경"으로 판정 → 계산 지연(deadline/OSQP 시간 한도·부정확)마다 이어쓰기 없이 STOP(속도 0). bench 입력 610틱 중 146틱이 창 슬라이드.
- 수정: `_same_path()` — 같은 점열이 밀린 경우만 계획 유지. 점이 하나라도 다르거나(회피·재계획) 속도 상한이 바뀌면 기존대로 폐기. 이어쓰기 검사(최대 2틱, 위치/heading/속도 일치, 횡가속·경로 재검사) 무변경.
- 근거: 5p0 실차 로그(1487 s) MPC STOP 2139회(86/min) 중 계산 지연 계열 ~1350회.
- 검증: `test_corner_stall.py::test_slid_local_window_keeps_continuation_changed_path_drops_it` (수정 전 실패/후 통과), 기존 이어쓰기 테스트 3건 통과. controller만 빌드.
- 무변경: A 버튼 래치, joy 단절 보호, simple_mux TTC EStop, MPC 튜닝값.
- reset_estop `no fresh wheel odometry` 거부: simple_mux 기동 52 ms 직후 요청(첫 odom 수신 전) → 코드 정상, 스택 기동 후 재호출.
- 미확인: 5p0 자율주행 중 TTC EStop 17회 이상(±70°, 0.35 s) — `/scan` 미기록으로 오판 여부 판단 불가.

## 단계 32 — 순간 버벅임 수정: 소프트 STOP + 이어쓰기 5틱 (JSND_INT `badf33a`, 실차 미구동)
- 진단(19:44 3p0 실차 로그, 자율 4구간): MPC STOP 1틱마다 속도 명령 0 → VESC `Current braking engaged: target=0.00 actual=2.57` 후 20 ms 뒤 해제 = 순간 버벅임. STOP 틱마다 조향도 0 쪽으로 0.064 rad 복귀(코너 꺾임). 이어쓰기 ~330회 적용됐지만 deadline STOP 165회 잔존 — 최대 2틱이고 2번째 틱이 단계 경계(0.07 s)에서 거부됨.
- 수정 1 (Controller.py): 주행 중 solver STOP → 속도 setpoint를 `max_decel`로 램프 감속 + 조향 유지, 최대 `mpc_fallback_max_s`(0.2 s), 이후 기존처럼 0·조향 복귀. TRAILING 대상 없음/미지원 상태/입력 오류 STOP은 즉시 0 유지.
- 수정 2 (integrated_mpc.py): `MAX_CONTINUATION_TICKS` 2→5(100 ms, 계획 0.57 s), 단계 경계 거부 조건 제거(상태 일치·속도 상한·횡가속·추종오차 재검사 유지).
- 무변경: A 래치, simple_mux TTC EStop(자율 중 79회, 거의 매 랩 — 별도 오판 확인 필요), 자율 미선택 0 강제, 최고속도 상한, OVERTAKE/상태머신.
- 검증: `test_mpc_integration.py::test_solver_stop_while_moving_ramps_down_then_zero`(신규), `test_corner_stall.py::test_deadline_miss_continues_verified_plan_at_most_max_ticks` 포함 7건 통과. 기존 실패 3건(test_profile_refinement…, test_budget_miss_recovers_over_ticks, test_recovery_after_stop_from_standstill)은 수정 전에도 실패. controller만 빌드, 백업 `~/JSND_backups/step32_20261124_195839`.
- 실차 확인: 로그 `Current braking engaged: target=0.00` 빈도. 튜닝: `mpc_fallback_max_s`(소프트 STOP 창), `mpc_r_rate`(코너 조향 부드러움).

## 단계 33 — Step 32 통합 MPC가 PP보다 느리고 불안정한 원인 분석 (코드 무변경, JSND_INT `badf33a`)
- 데이터 범위: 최신 실차는 텍스트 로그뿐(`race_mpc5p0_integrated_20261124_200556.log`, 자율 34 s). bag `4p0_20261124_182441`은 estop 래치 정지(최고 0.0007 m/s)라 주행 데이터 아님. **주행 중 조향각·조향속도·횡오차·헤딩오차·조향지연·MPC 계산시간은 실측 없음.** MPC 상태 로그는 상태 변화 시 1줄 → 횟수는 하한.
- 랩타임(11_18_6): PP(11-18 20:01 원본 JSND, 섹터 1구간 1.0) 5.45~6.01 s / 통합 d3b20b9 3p0 5.95~6.7 / 통합 badf33a 3p0 7.39~7.86 / 통합 badf33a 5p0 8.75, 7.67 후 3랩째 실패(수동 개입) / 조향 MPC f3326a1 5p0 7.87~8.00. Step 32는 5p0가 3p0보다 빠르지 않음 → 속도 상한은 비결정 요인.
- 현재 `speed_scaling.yaml`은 코너 2구간 0.6(5섹터) — PP·jsnd 소스에만 적용, 통합 MPC 미사용. PP 기록(11-18)은 전 구간 1.0이었음.
- 1. 코너 속도: 제동 이벤트 실제속도 최대 PP 3.88 / MPC 5p0 2.86 m/s. MPC 기하 상한 sqrt(0.85·4.5/|κ|)은 레이싱라인과 거의 같음(s≈3: 2.06 vs 2.07, s≈10: 2.15 vs 2.26, 상한<vx 14%, 평균 −0.09 m/s) → 곡률 제한은 주원인 아님. MPC가 로컬 경로 수치미분으로 구한 실제 κ는 미기록.
- 2. 예측 0.57 s(N=12): 2.0~2.8 m/s에서 1.1~1.6 m, 헤어핀 진입(κ 0.25→0.9) 약 2 m의 절반만 봄. 단계 22의 '진입 조향 지연'과 부합. 단 계산시간 초과가 잦아 N 증가는 위험. 실차 효과 측정 불가.
- 3. 조향 진동·포화: 측정 불가. 간접 증거 — 5p0 3랩째 468.5~470.5 s 계산 실패 집중 → outside_tracking(>1.0 m) → heading envelope STOP → 473.4 s 수동. d3b20b9 로그 tracking STOP 1593, heading 36.
- 4. 가중치: q_position 12, q_heading 8, r_steer 0.2, r_rate 2.0(단계 30), terminal 3, q_speed 8, r_accel 0.15. r_rate 2.0 + 짧은 예측 → 진입 조향 지연 추정(실측 미확인).
- 5. 모델: 휠베이스 0.324·조향속도 3.2 rad/s·지연보정 0.1 s 모두 실측 아님. 오프라인(단계 24)에서 자세지연 ≥120 ms면 MPC만 발산. 실제 PF/odom·서보 지연 미측정.
- 6. 빈도(5p0, 자율 34 s, 하한): MPC STOP 67(≈2/s — deadline 20, OSQP inaccurate 13, time limit 12, braking_envelope_infeasible 7, heading/tracking 9, 입력오류 5), plan_continuation 57, solved_inexact_verified 22. TTC EStop 6(≈3/랩), 3p0 badf33a 10/4랩, d3b20b9 103/62랩, PP 로그 0(PP 당시 TTC 활성 여부는 로그로 확인 불가). TTC 1회당 0.06~0.4 s 동안 실제속도 평균 약 −0.6 m/s.
- 가속 구조: MPC A=5.0이지만 setpoint는 측정속도+0.5 m/s(`mpc_cmd_max_lead_mps`) 이내, 이전 setpoint에 a·dt 누적. PP는 0.25 s 앞 레이싱라인 vx를 계단 명령. 실제 가속도는 미기록.
- 우선순위: ① TTC EStop(랩당 ~3회, PP와 설정 동일 여부 확인) ② 계산 실패 STOP + Step 32 소프트 STOP(틱당 0.1 m/s, 최대 1.0 m/s 하강 후 낮은 setpoint에서 재가속; 3p0 랩 Step 31→32 약 +1.3 s, 세션 차이 있어 '유력') ③ lead 0.5 m/s 누적 방식의 가속 제약 ④ 코너 불안정(예측 0.57 s, r_rate 2.0, 미측정 지연) ⑤ 곡률 제한(주원인 아님).
- 다음: 코드 수정 없이 `speed_rec.sh`로 주행 bag 1회(조향·속도 명령, odom, frenet, mpc_status, elapsed, IMU, /scan) → 3~5번 정량화 후 튜닝. OVERTAKE·상태머신 무변경(로그 상태 GB_TRACK만).

## Step 34 (JSND_INT `5e69a81`, controller만 빌드, 실차 미구동)
- 누적 감속 수정: Step 32 소프트 STOP 램프가 속도 설정값 기준(`mpc_speed_cmd`)을 낮춘 채 남겨, 해가 돌아와도 max_accel로만 회복 → 실패마다 속도 손실 누적(Step 31 5.95~6.7 s → Step 32 7.4~7.9 s). 기준을 `measured - mpc_cmd_max_lead_mps` 이상으로 클램프(대칭 anti-windup). MPC 자체 감속(a·dt)은 그대로.
- deadline: 검증 통과 해가 deadline(16 ms)만 넘고 제어주기(20 ms) 안이면 버리지 않고 사용(`solved_late`). 주기 초과는 기존대로 이어쓰기/STOP.
- 예측시간·조향변화 가중치·가속 제한: 주행 bag 실측이 없어 무변경. 차량 모델·지연 무변경.
- N=30 실험(오프라인 무부하, bench 입력 363틱, preview_dt 0.05 유지): N=12 p99 5.6 ms / N=30 p99 11.7 ms(2.1배), N=30에서 `outside_envelope` STOP 7건 신규. 실차 전체 스택은 오프라인 대비 약 2배(Step 13) → N=30 p99 약 23 ms로 50 Hz 미달 추정. 기본값 N=12 유지, N=30은 yaml만 바꿔 실차 실험 가능.
- test_corner_stall: 6 passed, 2 failed(test_profile_refinement, test_unwinding — 수정 전부터 실패). deadline 강제 테스트는 `ACCEPT_VERIFIED_LATE=False`로 주기 초과를 모사, `solved_late` 테스트 추가.

## Step 35 — lowspeed 1 m/s 실차 bag 시간축 분석 (JSND_INT `5f5c831`, controller만 빌드, 실차 미구동)
bag `3p0_20261124_203250`(97 s, 자율 4랩), 로그 `race_mpclowspeed_integrated_19700101_090100.log`.
- **버벅임 직접 원인**: 매 랩 s≈1.3에서 state_machine이 정적 장애물(obs_s≈4.22, RIGHT, ot lane 0.12 m, NO_STATIC_PATH)로 TRAILING 판정 → safety cap이 한 틱에 급락 → 감속 부족분 > 0.30 m/s → `braking_envelope_infeasible` STOP 5틱(자율 중 STOP 19틱 중 16틱) → 소프트 STOP 램프가 설정값 0 → VESC 전류제동 0.97→0.78 m/s. 4랩 모두 동일(990.0/1005.1/1020.3/1035.7 s).
  수정: 부족분이 safety cap(TRAILING) 때문이고 기하/끝점만으로는 허용치 이내면 STOP 대신 최대 가능 감속으로 추종(`braking_recovery`). 기하 원인 부족분은 기존대로 STOP. 상태머신·estop 무변경. 회귀테스트 `test_mpc_trailing.py::test_trailing_cap_drop_while_moving_brakes_instead_of_stop`(수정 전 실패/후 통과).
- **코너 불안정 직접 원인 = 위치추정**: 헤어핀(s≈2.9~3.4)에서 매 랩 PF 자세가 1.5~2.0 s 정체(휠 1.45~1.84 m 이동 중 자세 0.24~0.45 m, IMU 요 0.8~1.2 rad 중 자세 0.03~0.5 rad) 후 0.1~0.4 m 순간이동. MPC는 정체된 자세로 조향(+0.24→−0.12→+0.25 rad 진동). MPC 계산실패·조향 포화와 무관(이 구간 status solved). PF/EKF는 원본 JSND 설치본이고 bag에 /scan·PF 원자세 없음 → 무변경. `speed_rec.sh`에 `/state_machine /pf/pose/odom /vesc/sensors/imu_corrected` 추가.
- **TTC**: 로그에 TTC/estop 래치 0건(estop_state 전부 clear) — 원인 아님.
- **MPC 계산시간**(자율 이동 중만): p50 4.4 / p99 16.8 / max 33.0 ms(87.5 ms는 정지 중). `deadline_exceeded` STOP 3틱.
- **집계 정정(speed_report.py)**: non-tracking이 `startswith('tracking')`로 집계돼 항상 100%(4763/4763) → 실제 자율 이동 중 STOP 19/2943틱. 제동 이벤트 기준 v0>1.0 고정 → 0.5로, 재가속한 소프트 STOP 램프는 제외. [POSE] 항목 추가: 추종 샘플의 25%가 자세 불일치, 이를 뺀 |d| 평균 0.167(전체 0.215). 횡오차 기준선(/car_state/odom_frenet d = iqp 레이싱라인, GB_TRACK의 local_waypoints와 동일)은 맞지만 PF 자세 오류 구간에선 신뢰 불가.
- 기존 실패(수정 전에도 실패): test_profile_refinement…, test_unwinding…, test_step12…, test_stale_obstacles_dropped(state_machine API 이름 불일치).

## Step 36 — lowspeed bag 20:44 반복 감속 원인 확정 (JSND_INT `abc3c3e`, controller만 빌드, 실차 미구동)
bag `lowspeed_20261124_204417`(142 s, 자율 20.7~125.2 s, 7랩), 로그 `race_mpclowspeed_integrated_20261124_204404.log`. N=12·비상정지·Step 35 변경 유지.
- **급감속 구간**: 자율 중 휠속도 저하는 매 랩 s≈1.1~1.9에서만 발생(30.4/45.7/61.1/76.6/91.5/121.5 s). 명령 1.0 → 0.10~0.37, 휠속도 0.97 → 0.52~0.75(61.16 s는 0.06, 전류제동 잠김). 그 외 구간 명령은 1.0 유지.
- **직접 원인**: 매 랩 static_avoidance가 정적 장애물(obs_s≈4.2~4.4, RIGHT)을 2.0~3.3 m 앞에서 검출 → state_machine이 회피 경로 생성 전(path_exists=0, decision=TRAILING) 1~45틱 TRAILING → `_mpc_trailing_cap`이 PP 규칙대로 정적 대상이면 거리와 무관하게 crawl 0.2 m/s 반환 → 한 틱에 cap 1.0→0.2 → braking_recovery 최대 감속 → VESC 전류제동. 이후 OVERTAKE에서 설정값이 measured−0.5(≈0.47)부터 재가속(~0.6 s).
  osqp 시간한도·deadline_exceeded는 이 틱들에 동반됐을 뿐 원인 아님(그 외 구간은 solved, cmd 1.0).
- **수정**: MPC 모드 TRAILING + 정적 대상 + standoff 래치 전: cap = max(PP cap, min(√(crawl² + 2·a·(gap−standoff)), max_speed)), a=`mpc_static_approach_decel_mps2` 1.5(mpc_car.yaml, 0이면 기존 동작). 2.53 m 앞·0.97 m/s → cap 2.6 → 감속 없음. 접근 시 a로 감속, standoff 0.5 m 0 래치 유지. PP 모드·동적 대상 trailing·state_machine 무변경.
  회귀테스트 `test_mpc_trailing.py::test_static_trailing_far_obstacle_no_step_brake`, `::test_static_trailing_approach_still_stops_at_standoff`(수정 전 실패/후 통과), 파일 전체 9 passed.
- **위치추정(무변경, 원인 미확정)**: 매 랩 s≈2.83에서 1.4 s 동안 휠+IMU 추측항법 1.25 m·요 0.75 rad, PF 0.25 m·0.15 rad(7랩 동일, 횡가속 0.66~0.75 m/s²로 실제 주행 중). PF 입력 정상: 모션모델 속도 0.93 m/s(/vesc/odom), IMU 요레이트, 42 Hz·최대 간격 49 ms·weight collapse 0, 헤더 지연 1~7 ms(now() 스탬프, 의도된 설계). → 센서모델이 자세를 끌어당기는 스캔-맵 불일치로 추정(정적 장애물 1.4 m 앞 또는 맵 변경). /scan 미기록이라 확정 불가 → 코드 무변경. 마지막 랩은 정체 후 d=−0.7 이탈, 125.2 s 수동 개입. PF 정체 중 ego s≈2.9에서 s≈2.40 유령 장애물 검출.
  다음 확인: `speed_rec.sh`에 /scan 추가해 같은 구간 기록, 장애물 제거 상태 1랩 비교.
- 백업 ~/JSND_backups/step36_20261124_205217.

## Step 37 — 5p0 bag 20:56 영구 정지 수정 (JSND_INT `9fd48ca`, controller만 빌드, 실차 미구동)
bag `5p0_20261124_205612`(258 s). 최종 mux 출력이 0이 된 이벤트 분류: TTC EStop 13(simple_mux→estop.py, 0.04~0.48 s, 무변경), TRAILING 중 OSQP 시간한도 STOP 2, **heading envelope 영구 정지 1**, 사람 개입 6.
- **영구 정지**: t=81.2 s 헤어핀 출구(s≈11.5, d −0.95→복귀 중) heading 오차 > 0.7 rad → `integrated_mpc._reference` `outside_heading_envelope` → 소프트 STOP 0.2 s 후 `Controller._mpc_speed_command` 0. 정지 후에도 매 틱 현재 자세로 같은 판정 → 자기 고착(사람 개입까지 0).
- **수정**: TRAILING 아님 + 속도 ≤ 0.6 m/s + heading envelope STOP이면 PP(전방 1 m) 조향으로 0.5 m/s 크리프 복귀, MPC 해 성공 시 MPC 복귀. `mpc_envelope_recovery_speed_mps: 0.5`(0=기존). N·비용·구조·비상정지 무변경. 테스트 `tests_offline/test_envelope_recovery.py` 2 passed.
- **PF 정체**: 81 s에서는 결과(정지 전 PF·car_state 차 ≤ 0.07 m). 62.3~63.3 s(휠 2 m/s, s≈9 고정)는 정지에 선행 — PF 고착인지 실제 접촉인지 /scan·IMU 없어 미확정.
- `run/speed_rec.sh`에 /scan /imu/data 추가.

## Step 38 — '코너 늦은 조향' 점검 (JSND_INT `d837d32`, 비교 프로파일만 추가, 기본값 무변경)
- 예측 0.02+11×0.05 = 0.57 s(2.2 m/s에서 1.24 m), 코너 진입→정점 2.2~2.6 m → 진입부만 포함.
- 오프라인 lap_sim(통합 MPC, `--set mpc_speed_source=integrated`): 조향이 레이싱라인 곡률보다 +0.04 s 선행(PP 0.00). dt 0.10: 위상 동일, 횡오차 0.031→0.110 m, STOP 1→6. r_rate 0.5: 변화 없음 → reference·r_rate가 지연 원인 아님.
- 실차 bag: 조향-곡률 상관 0.30(시뮬 0.98). 명령→PF yaw rate 지연 110 ms(gain 1.11, latency_comp 0.1과 일치). PF/EKF 위치 지터 p95 0.08~0.10 m, EKF–PF heading 차 p95 0.17 rad. `/car_state/odom` angular.z는 조향·PF와 무상관. bag에 imu_corrected 0건(라이브 100 Hz, 원인 미상) → 자세 잡음이 주원인으로 추정, 미확정.
- 비교 실행: `bash run/race_mpc.sh 5p0_pdt10 11_18_6 integrated` (`mpc_profile_5p0_pdt10.yaml` = 5p0 + `mpc_preview_dt: 0.10`).


## Step 40 — 횡가속도 제한 오프라인 비교 (기준 `660b6c8`, 실차 기본값 무변경)
- 스크립트: `jsnd_int/tests_offline/compare_step40.py`; 완료된 원본 결과: `jsnd_int/tests_offline/step40_results.jsonl`.
- 조건: 저장소 test0, integrated MPC 8.88 m/s, N=12, preview_dt=0.05, 50 Hz, 각 2랩. 기존 lap_sim의 차량·서보·속도루프 가정 및 실제 deadline 사용. 횡가속도 한계만 4.5/6.0/8.0 m/s²로 실행 시 override; 종가감속 한계 5 m/s², 결합 제약, E-stop 및 실차 YAML은 무변경.
- 랩타임은 두 번째 랩; 횡오차·STOP·최고속도는 전체 주행 기준. STOP은 연속 실패 구간 수이며 실제 정차 횟수가 아님. 연산 시간 기반 실패는 실행 환경에 따라 변동 가능.

| 횡한계 (m/s²) | 랩타임 (s) | 평균 / 최대 횡오차 (m) | STOP 구간 / 틱 | 실제 최고속도 (m/s) |
|---|---:|---:|---:|---:|
| 4.5 | 11.30 | 0.040 / 0.222 | 2 / 2 | 6.385 |
| 6.0 | 10.12 | 0.051 / 0.220 | 3 / 4 | 6.508 |
| 8.0 | 9.16 | 0.062 / 0.285 | 2 / 2 | 6.616 |

- **결론**: 4.5→8.0에서 2.14 s(18.9%) 단축. 최고속도 상승은 0.231 m/s에 그쳐, 횡한계가 주로 코너 통과 속도를 통해 랩타임을 제한한다는 근거. 기존 Step 39 PP(8.56 s)보다 여전히 0.60 s 느림. 평균 횡오차 증가, STOP 개선은 확인되지 않음. 실차 접지 한계 검증 결과가 아님.
- **종·횡 결합**: `(ax/5)²+(ay/Y)²≤1`; QP는 속도 상한·조향 선형 상계와 내접 16각형으로 보수적으로 적용하고 비선형 예측을 재검증. 역방향 제동 계획도 `D*sqrt(1-(v²κ/Y)²)`로 남은 제동 여유를 반영. 횡가속도 사용률 85%에서는 종가감속 여유가 약 2.63 m/s²로 줄어듦. 계획 속도 자체도 `sqrt(0.85*Y/|κ|)`에 제한되므로 Y 상향은 곡률 속도·조향 여유·결합 가감속 여유를 함께 변경함.
- 성공한 해의 첫 단계에서 실제 비선형 예측 기반 타원 사용률 >90% 비율: 28.9/23.2/20.8%. 같은 단계의 타원 기준 종가감속 가용량 중앙값: 3.66/3.83/4.21 m/s². 이는 QP 제약 활성률이나 실제 구동력 측정이 아님. 결합 제약 단독의 랩타임 기여도는 별도 분리 실험 없이 확정할 수 없음.
- 기존 실행 스크립트의 import 경로만 저장소 상대 경로로 변경하여 보관. 결과 JSONL은 기존 파일 그대로 복사. 저장 과정에서 테스트·시뮬레이션 재실행 없음. 실차·SSH 접속 없음.
- 이후 작업 완료 시 실험 스크립트·결과·핵심 분석과 코드 변경을 GitHub에 함께 저장한다는 사용자 요청을 기록함.

## Step 41 — 종방향 가감속도 제한 오프라인 비교 (기준 `730187b`, 실차 기본값 무변경)
- 스크립트: `jsnd_int/tests_offline/compare_step41.py`; 원본 결과: `jsnd_int/tests_offline/step41_results.jsonl`. 기존 `compare_step39.py`의 저장소 환경과 `lap_sim.simulate` 재사용.
- 고정: test0, integrated MPC 최고속도 상한 8.88 m/s, 횡한계 8.0 m/s², N=12, preview_dt=0.05, 50 Hz, 실제 deadline, 각 2랩. MPC 가속·감속 한계를 함께 5/6/8 m/s²로 실행 시 override. 그 외 파라미터·안전 기능·실차 기본 YAML 무변경.
- 랩타임은 두 번째 랩; 횡오차·최고속도·STOP은 전체 주행 기준. STOP은 연속 solver 실패 구간 수이며 물리적 정차 횟수가 아님.

| MPC 가속 / 감속 한계 (m/s²) | 랩타임 (s) | 평균 / 최대 횡오차 (m) | 실제 최고속도 (m/s) | STOP 구간 / 틱 |
|---|---:|---:|---:|---:|
| 5 / 5 | 9.14 | 0.061 / 0.248 | 6.619 | 0 / 0 |
| 6 / 6 | 8.96 | 0.066 / 0.254 | 6.989 | 0 / 0 |
| 8 / 8 | 8.86 | 0.099 / 0.500 | 7.553 | 2 / 3 |

- **결론**: 5→6은 0.18 s(2.0%) 단축, 횡오차 소폭 증가, STOP 없음. 6→8은 추가 0.10 s 단축에 그치며 최대 횡오차가 약 2배, OSQP 실패 3틱(2구간) 발생. 5→8 전체 개선은 0.28 s(3.1%)로 Step 40 횡한계 상향의 2.14 s(18.9%)보다 작다. 이번 범위에서는 종방향 한계보다 횡한계의 랩타임 영향이 컸고, 6/6이 8/8보다 추종오차·STOP 측면에서 양호했다. 기존 PP 8.56 s보다 8/8도 0.30 s 느림.
- **해석 한계**: 기존 시뮬레이션 차량의 실제 가속/제동 포화는 +4/-6 m/s², 속도루프 시정수 0.12 s로 그대로 유지. MPC 한계 상향은 플랜의 결합 타원 및 역방향 제동 여유를 늘리지만 실제 플랜트 능력은 증가하지 않는다. 특히 8/8은 플랜트 제동 한계보다 낙관적인 모델이므로 이 결과를 실차 한계 상향 근거로 사용할 수 없다. 가속·감속을 동시에 변경해 둘의 개별 기여도는 분리하지 않았으며, 높은 오차의 원인을 하나로 확정하지 않았다.
- 5/5는 Step 40의 9.16 s·STOP 2와 달리 9.14 s·STOP 0. 실제 wall-clock deadline을 사용하는 단일 실행 결과여서 계산 실패와 경로는 실행 환경에 따라 변동 가능하며, STOP 차이를 알고리즘 개선으로 해석하지 않는다.
- 요청한 3조건만 1회씩 실행. 별도 테스트·파라미터 탐색·실차/SSH 접속 없음. 실차 설정·안전 기능 변경 없음.

## Step 42 — 남은 속도 제한 계측 및 곡률 계획 여유 계수 비교 (기준 `2ff1c64`)
- 스크립트: `jsnd_int/tests_offline/compare_step42.py`; 원본 측정값과 조건 메타데이터: `jsnd_int/tests_offline/step42_results.jsonl`.
- 기준은 Step 41에서 균형이 좋았던 MPC 종방향 가속/감속 6/6 m/s², 횡한계 8 m/s², 최고속도 상한 8.88 m/s. test0, N=12, preview_dt=0.05, 50 Hz, 2랩, 실제 deadline, 기존 플랜트/잡음/서보 조건 유지. PP는 Step 39의 동일 test0 결과를 참조하며 재실행하지 않음.
- **계측 및 선정**: 기준 주행에서 `mpc_cmd_max_lead_mps=0.5` 상한 활성 0%; 예측 속도 최댓값에 의한 명령 상한 활성 약 7.6%; 누적 명령이 MPC 가감속과 반대 방향인 구간 약 3.5%. 곡률 단독 상한 근접 구간 약 24.0%, 역방향 제동 상한 근접 구간 약 29.9%. 제동 계획은 약 32.7% 구간에서 곡률 단독 상한보다 0.15 m/s 이상 낮았음. 이 비율은 인과적 랩타임 기여율이 아니며, 근접 판정은 `v >= cap-0.15`로 초과 구간도 포함. 명령 지표는 성공한 해만 집계.
- **0.5 lead 및 누적 명령**: 기존 플랜트는 `a=clip((v_cmd-v)/0.12,-6,+4)`이므로 0.5 m/s lead가 허용하는 가속은 4.17 m/s²로 플랜트 +4 포화보다 큼. 실제로 상한 활성 0%여서 lead를 올릴 근거 없음. 누적 방식은 가감속 전환에서 짧은 지연을 만들 수 있지만 반대 방향 비율 3.5%만으로 지배적 병목이라 할 수 없음. 둘 다 무변경.
- **곡률 및 제동 계획**: 곡률 상한 `sqrt(0.85*Y/|κ|)`는 물리 횡한계 대비 속도를 약 7.8% 낮추고 조향 보정 여유를 남김. 전체 로컬 경로를 쓰는 역방향 제동은 같은 종·횡 결합 타원과 경로 끝 정지 여유를 반영하므로 무조건 불필요한 감속으로 볼 수 없음. 안전 제동은 유지하고, 계측상 영향이 큰 변경 가능한 후보인 곡률 계획 여유 계수 하나만 0.85→1.0으로 실험. 모든 후보의 개별 ablation을 한 것은 아니므로 유일하거나 절대 최대 병목이라고 단정하지 않음.

| 곡률 계획 계수 | 두 번째 랩 (s) | 전체 평균 / 최대 횡오차 (m) | STOP 구간 / 틱 |
|---|---:|---:|---:|
| 0.85 (기준) | 8.98 | 0.066 / 0.268 | 0 / 0 |
| 1.00 (실험) | 8.58 | 0.079 / 0.303 | 0 / 0 |

- **결과**: 0.40 s(4.5%) 단축, 평균 횡오차 +0.013 m·최대 +0.035 m, STOP 증가 없음. 기준과 PP 8.56 s의 차이 0.42 s 중 대부분을 줄였지만 잔여 0.02 s는 제어 한 틱과 같아 PP보다 빠르다고 할 수 없음. 곡률 계획 여유가 이 조건의 남은 성능 차이에 크게 기여한다는 근거. 계수 변경은 역방향 제동 envelope에도 전파되므로 직접적인 코너 속도 효과와 그에 따른 제동 계획 효과는 분리하지 않았음. 실험에서 실제 최대 횡가속도는 7.79 m/s²로 고정 물리 한계 8.0 이내.
- **모델 불일치**: MPC +6/-6 대 플랜트 +4/-6 m/s² 및 0.12 s 속도루프. 가속 능력은 MPC가 낙관적이고 제동 포화 수치는 같지만 속도루프와 누적 명령의 과도응답은 MPC의 직접 가속 입력 모델과 다름. 이 불일치는 그대로 유지했으며, 횡한계·종한계 추가 상향이나 모델 변경은 하지 않음. 기구학 플랜트에는 실제 타이어 접지/슬립 검증이 없어 1.0을 실차 권장값으로 해석할 수 없음.
- **범위 및 보존**: 0.85/1.0 각 1회만 실행. script 내 프로세스 한정 class 상수 override로만 비교했으며 production `LATERAL_PLAN_FRACTION=0.85`, 실차 YAML, 제약 검증, E-stop, PP/FTG/회피는 무변경. 별도 테스트·추가 탐색·실차/SSH 접속 없음. 결과 보관 시 실행 경로를 상대 경로로 정리하고 조건 메타데이터만 추가했으며 재실행하지 않음. wall-clock deadline과 단일 실행 변동 때문에 개선 수치는 이 오프라인 조건에 한정됨.

## Step 43 — Step 38 잡음·응답 지연 가정에서 PP / Step 42 MPC 비교 (기준 `7e230fe`)
- 스크립트: `jsnd_int/tests_offline/compare_step43.py`; 결과: `jsnd_int/tests_offline/step43_results.jsonl`. test0, 50 Hz, seed=0, 조건별 최대 2랩/60초, 실제 deadline. PP는 기존 설정 그대로. MPC는 Step 42의 8.88 m/s, 종방향 6/6 m/s², 횡방향 8 m/s², 계획 계수 1.0을 프로세스 내에서만 적용. 추가적인 물리 한계 상향 없음.
- **기존 구현 확인**: 조향 큐 기본 1틱(20 ms), 서보 속도 제한 6 rad/s, 속도루프 tau=0.12 s 및 +4/-6 m/s² 플랜트 유지. 위치 잡음은 x/y 독립 Gaussian 표준편차 `noise_pos`, 기존 heading 잡음은 `0.5*noise_pos`. 초기 `offset_yaw`는 측정 잡음이 아니라 차량 초기 자세 오차이므로 사용하지 않음. MPC의 기존 `mpc_latency_comp_s=0.1`은 제어기 예측 보정이므로 유지하며 추가 센서 지연 큐는 넣지 않음.
- **Step 38 근거와 가정**: PF/EKF 위치 지터 p95 0.08~0.10 m → 중간값 0.09 m를 2D 방사 오차 p95로 해석해 x/y sigma=0.03677 m. EKF–PF heading 차 p95 0.17 rad → 절댓값 p95의 영평균 Gaussian으로 해석해 sigma=0.08674 rad. 두 값은 실제 절대 위치/자세 오차나 백색잡음 분포가 확정된 측정이 아니므로 명시적 민감도 가정임.
- 명령→PF yaw rate 지연 110 ms는 액추에이터/차량/위치추정 지연이 섞인 관측. 이를 조향 큐 **총** 5.5틱(110 ms)으로 대표하고 기본 20 ms에 더하지 않음. 연속 5/6틱 지연 명령을 선형 보간. 기존 rate limiter와 이산 플랜트 응답은 유지되므로 전체 명령→yaw 응답이 정확히 110 ms로 식별·보정된 모델은 아님. 별도 pose delay도 적용하지 않음.
- `lap_sim.simulate`에 선택형 `noise_yaw`를 추가: 지정하면 기존 위치 연동 heading 잡음을 대체하며 더하지 않음. 기본 None은 기존 동작 그대로. fractional servo delay 지원도 정수 delay에서는 기존 동작을 보존. 정상 조건은 기존 20 ms·모든 측정잡음 0, 오차 조건만 위 값을 사용. 두 제어기에 동일 seed·분포·큐 지연 적용, 속도 잡음은 0.

| 조건 | 제어기 | 완료 랩 | 두 번째 랩 (s) | 평균 / 최대 횡오차 (m) | STOP 구간 / 틱 |
|---|---|---:|---:|---:|---:|
| 정상 | PP | 2 | 8.56 | 0.164 / 0.327 | 0 / 0 |
| 정상 | Step 42 MPC | 2 | 8.58 | 0.079 / 0.300 | 1 / 1 |
| Step 38 오차 모사 | PP | 2 | 8.66 | 0.151 / 0.380 | 0 / 0 |
| Step 38 오차 모사 | Step 42 MPC | 0 | 미완주(60초) | 1.176 / 1.357 | 6 / 2595 |

- **결과**: 정상에서 PP와 MPC 랩타임이 근접했으나 합성 오차 조건에서 PP는 +0.10 s로 완주, MPC는 크게 이탈하며 60초 내 한 랩도 완료하지 못함. 이 모사 조건에서 MPC의 강한 성능 저하가 재현됨. MPC 오차 조건 실패는 기존 집계 기준 ValueError 2565틱, nonlinear_prediction_outside_envelope 27틱, OSQP 3틱. 최대 오차가 tracking 허용 1 m보다 큰 점은 추종 범위 이탈과 부합하지만 ValueError의 세부 이유를 JSONL이 분리하지 않으므로 전부 동일 원인이라고 단정하지 않음.
- **해석 범위**: 잡음·heading·지연을 함께 변경한 1개 조건/seed이므로 각각의 인과 기여나 Step 38 실차 원인을 확정할 수 없음. PF 정체·점프, 시간 상관 잡음, 타이어/슬립, 맵·장애물은 미모사. 기존 시뮬레이터는 local path 선택과 Frenet s/d에 ground truth를 쓰며 noisy pose만 Controller에 전달하고 yaw rate도 ground truth임. 따라서 완전한 센서/위치추정 재현이 아님. E-stop 전체 ROS 체인은 구동하지 않았으며 관련 코드는 무변경.
- 횡오차는 잡음이 섞인 측정값이 아닌 실제 시뮬레이션 상태 기준. 미완주 MPC의 평균은 정체 구간을 포함한 60초 전체이고, 완주 사례는 2랩 구간이므로 동일 시간 창의 평균 비교가 아님. STOP은 PP에서는 speed<=0 구간, MPC에서는 solver 실패 구간으로 집계되는 기존 정의이며 물리적 정차 횟수/비상정지 횟수가 아님. 정상 MPC 1틱 OSQP STOP은 실제 deadline의 실행 환경 변동 가능성이 있어 Step 42 대비 코드 성능 퇴행으로 단정하지 않음.
- 정상/오차 × PP/MPC의 4회만 실행. 별도 테스트·대규모 탐색·실차/SSH 접속 없음. 실차 기본 설정, production MPC 계수, 안전 기능 변경 없음.

## Step 44 — 위치·방향 잡음 및 조향 지연 단독 비교 (기준 `7099aa3`)
- 재현 스크립트: `jsnd_int/tests_offline/compare_step44.py`; 결과: `jsnd_int/tests_offline/step44_results.jsonl`. 정상·복합 오차 4개 결과는 `step43_results.jsonl`을 그대로 재사용했고 재실행하지 않았다. 새 실험은 세 조건 × PP/MPC 각 1회, 총 6회 완료. 제어기·기존 시뮬레이터·YAML 수정 없음.
- Step 43과 동일한 test0/글로벌 경로, 50 Hz, seed=0, 최대 2랩/60초, N=12, preview_dt=0.05, 실제 deadline 16 ms/OSQP 8 ms. MPC 최고속도 8.88 m/s, 종방향 6/6 m/s², 횡방향 8 m/s², 프로세스 내 계획 계수 1.0, 기존 지연 보상 0.1 s 유지. production 계획 계수는 0.85 그대로다.
- 위치만: x/y 독립 Gaussian sigma=0.0367685084 m(방사 p95=0.09 m), yaw sigma=0을 명시해 기존 위치 연동 yaw 잡음을 차단. 방향만: yaw Gaussian sigma=0.0867362877 rad(절댓값 p95=0.17 rad), 위치 잡음 0. 두 조건의 조향 큐는 정상 1틱(20 ms). 조향 지연만: 총 5.5틱(110 ms), 위치·방향 잡음 0. 속도 잡음은 모두 0, 추가 pose delay 없음. 플랜트 +4/-6 m/s², 속도루프 tau=0.12 s, 서보 rate=6 rad/s 유지.
- 실행: `OPENBLAS_NUM_THREADS=1 /tmp/jsnd-step39-venv/bin/python jsnd_int/tests_offline/compare_step44.py`. 최초 시스템 Python 실행은 위치-only PP 완료 후 MPC 생성 시 OSQP 미설치로 중단됐다(해당 MPC 주행 시작 전). 완료 결과를 보존하고 기존 Step 39 venv에서 `--resume jsnd_int/tests_offline/step44_results.jsonl` 및 `>>`로 나머지 5회를 실행했다. 양쪽 Python 3.10.12, NumPy 1.26.4, SciPy 1.15.3, PyYAML 5.4.1로 동일하며 venv OSQP는 1.1.3. 추가 설치·재주행·별도 테스트는 하지 않았다.

| 조건 | 제어기 | 완료 랩 | 1랩 / 2랩 (s) | 평균 / 최대 횡오차 (m) | STOP 구간 / 틱 |
|---|---|---:|---:|---:|---:|
| 정상 (Step 43 재사용) | PP | 2 | 9.10 / 8.56 | 0.164 / 0.327 | 0 / 0 |
| 정상 (Step 43 재사용) | MPC | 2 | 9.56 / 8.58 | 0.079 / 0.300 | 1 / 1 |
| 위치 잡음만 | PP | 2 | 9.10 / 8.56 | 0.162 / 0.326 | 0 / 0 |
| 위치 잡음만 | MPC | 2 | 9.54 / 8.52 | 0.066 / 0.273 | 3 / 3 |
| 방향 잡음만 | PP | 2 | 9.16 / 8.58 | 0.157 / 0.350 | 0 / 0 |
| 방향 잡음만 | MPC | 2 | 9.54 / 8.60 | 0.093 / 0.544 | 4 / 7 |
| 조향 지연만 | PP | 2 | 9.18 / 8.64 | 0.164 / 0.353 | 0 / 0 |
| 조향 지연만 | MPC | 2 | 9.72 / 8.74 | 0.127 / 0.520 | 1 / 1 |
| 복합 오차 (Step 43 재사용) | PP | 2 | 9.24 / 8.66 | 0.151 / 0.380 | 0 / 0 |
| 복합 오차 (Step 43 재사용) | MPC | 0 | 미완주 (60초) | 1.176 / 1.357 | 6 / 2595 |

- **영향 순위**: MPC 랩타임과 평균 횡오차에는 **조향 지연**이 가장 컸다. 정상 대비 2랩 +0.16 s(+1.9%), 평균 +0.048 m(+60.8%), 최대 +0.220 m. **방향 잡음**은 최대 횡오차 0.544 m(정상 대비 +0.244 m)와 STOP 7틱으로 가장 나빴다. 평균 +0.014 m, 랩타임 +0.02 s. 위치 잡음은 완주/횡오차 악화가 관찰되지 않았으며 -0.06 s를 잡음의 성능 개선 효과로 해석하지 않는다. PP는 모두 완주·STOP 0이며 2랩 증가도 지연 +0.08 s가 최대였다.
- **핵심 결론**: 우선 조사할 원인은 조향 지연과 방향 입력의 결합이다. 단독 세 조건 모두 완주했으므로 복합 오차의 미완주를 어느 한 원인만으로 설명할 근거는 없다. 단독 결과상 위치 잡음의 영향은 작지만 복합 조건에서의 기여까지 배제할 수 없다. 지연×방향 등 2요인 조합이나 보상 on/off는 요청 범위 밖이므로 실행하지 않았으며, 상호작용의 크기나 보상 구조의 인과적 기여는 확정하지 않는다.

### STOP 세부 원인
- 원본 결과에 `stop_causes_full`과 `stop_episodes`를 추가했다. 기존 결과의 축약 `status`와 달리 콜론 뒤 세부 이유를 보존하며, 각 연속 실패 구간의 시작 tick/시각/실제 횡오차/첫 사유/길이를 기록한다. tick은 0부터, 시작 시각은 제어 호출 직전 기준이다. 후처리만 추가하여 solver 실행 중 계측 부하는 없다.
- 위치만: `osqp:run time limit reached` 2틱, `osqp:solved inaccurate` 1틱. t=8.36/16.94/16.98 s에서 각각 1틱. 지속적인 추종 envelope 이탈은 없었다.
- 방향만: `nonlinear_braking_envelope_violation` 1틱(t=12.72 s), `osqp:solved inaccurate` 1틱(t=15.98 s), `nonlinear_prediction_outside_envelope:tracking` 5틱(t=16.38 s 1틱, 16.46 s부터 4틱). 예측 경로 tracking 위반이며 현재 차량이 1 m 밖이라는 뜻은 아니다. 해당 두 구간 시작 실제 횡오차는 -0.034/-0.074 m이고 전체 최대도 0.544 m다. 방향 잡음이 단순 heading 허용치 초과가 아니라 미래 횡위치 예측과 제약 검증을 통해 실패를 만들 수 있음을 보여준다.
- 지연만: `osqp:primal infeasible` 1틱(t=16.92 s), 시작 실제 횡오차 +0.00985 m. 한 틱의 infeasible만으로 어떤 QP 제약 조합이 지배했는지 단정할 수 없다.
- 기존 정상 MPC는 축약 OSQP 1틱, 복합 MPC는 ValueError 2565틱·nonlinear_prediction_outside_envelope 27틱·OSQP 3틱. 기존 파일에는 세부 사유와 시계열이 없어 복합 오차의 최초 실패나 ValueError 전체를 tracking 원인으로 확정할 수 없다. 총 STOP 시간은 복합 51.90 s, 위치 0.06 s, 방향 0.14 s, 지연 0.02 s다.
- STOP은 MPC의 `not result.success` 구간/틱이며 실제 완전 정차·E-stop 횟수가 아니다. 기존 soft-stop은 이동 중 제한된 실패 구간에서 속도 명령을 감속하고 조향을 유지한다. 이후에는 0속도/조향 복귀 경로를 따른다. 저속 heading envelope 복귀 분기도 있지만 tracking envelope 복귀에는 적용되지 않는다. 이번 단독 실행에는 해당 recovery 상태가 없었다.

### 지연 보상과 자세 입력 구조 확인
- **실제 실행 소스**는 `jsnd_int/racing_stack/controller/controller/combined/src/Controller.py`와 그 아래 `mpc/integrated_mpc.py`다. `compare_step39.py`가 해당 경로를 우선 import한다. 저장소 루트 `controller/controller/mpc/integrated_mpc.py`의 구형 구현과 혼동하지 않는다.
- `integrated_loop`는 `[x,y,yaw]`를 복사하고 rear-axle offset(현재 0 m)을 적용한 뒤, `round(0.1/0.02)=5`단계 동안 측정 속도와 `curr_applied_steering`을 고정한 기구학 모델로 x/y/yaw를 전진시킨다. 이 예측 자세와 현재 측정 속도를 `[x,y,yaw,v]`로 solver에 전달한다. PP 자세/속도 전처리와 IMU yaw-rate 융합은 이 integrated 경로에서 사용하지 않는다.
- **`curr_applied_steering`은 실제 바퀴각이 아니다.** `record_applied_steering`은 마지막 발행 명령을 저장하며, 시뮬레이터 역시 지연 큐와 rate limiter를 통과하기 전 명령을 전달한다. 실제 플랜트 `delta`나 대기 중인 5/6틱 명령 이력을 보상기에 주지 않는다. MPC 상태도 `[x,y,yaw,v]`뿐이므로 액추에이터 지연 상태나 명령 큐가 없다. 따라서 100 ms pose 전진 예측을 110 ms 조향 지연의 정확한 보상으로 해석할 수 없다. 조향 전환 때 아직 적용되지 않은 명령을 이미 적용된 조향으로 예측할 수 있다.
- 이번 환경의 pose에는 측정 지연 큐가 없는데 고정 100 ms 전진 보상은 정상·단독·복합 모두 켜져 있다. 실제 pose timestamp/age에 맞춰 보상 시간을 조정하지 않는다. 이는 Step 43 조건을 유지한 구조적 모델 불일치이며, 보상값을 바꿔 검증한 결과는 아니다.
- 위치·방향 잡음은 필터/공분산 가중 없이 매 틱 입력에 직접 들어간다. 위치 잡음은 예측 시작 위치를 이동시키고 방향 잡음은 시작 yaw뿐 아니라 전진 예측의 cos/sin 방향도 바꾼다. 작은 각도 근사로 보상 이동의 횡오차 표준편차는 `v*0.1*0.08674`이며 v=6 m/s에서 약 0.052 m다(계산 예시, 측정 기여량 아님). 이어지는 MPC 미래 예측에도 yaw 오차가 전파된다. solver의 yaw unwrap/각도 정렬은 ±π 표현 처리를 위한 것이며 잡음 제거 필터가 아니다. 입력 tracking 허용 1 m, heading 허용 0.7 rad 및 비선형 예측 재검증은 유지됐다.
- ROS manager `odom_cb`는 `/car_state/odom`에서 x/y와 quaternion→yaw, twist 속도를 읽어 전달한다. 이 callback 자체에는 자세 평활화가 없다. 실제 upstream PF/EKF의 융합 품질은 이번 실험으로 검증하지 않았다. offline harness는 manager callback을 통하지 않고 noisy pose를 직접 공급하며 local path 선택과 Frenet s/d, yaw rate는 ground truth를 사용한다.
- 단일 seed·단일 실행 및 wall-clock deadline 변동 때문에 작은 랩타임/solver STOP 차이는 통계적 효과로 해석하지 않는다. 횡오차는 실제 상태 기준 전체 주행 평균/최대이며 복합 미완주의 60초 정체 구간과 완주의 약 18초 주행은 동일 시간 창이 아니다. 실차 지연 식별·타이어 슬립·시간 상관 잡음은 포함하지 않는다. 요청한 실험 결과·구조 분석만 저장하며 제어기 변경과 추가 테스트는 수행하지 않았다.
