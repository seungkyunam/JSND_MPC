# WORK_STATUS

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
