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
