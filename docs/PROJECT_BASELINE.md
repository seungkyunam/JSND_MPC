# PROJECT_BASELINE — JSND 기반 Integrated MPC 실차 통합

기준 시점: 2026-11-22, Jetson(`jsnd`, Jazzy, `ROS_DOMAIN_ID=173`, CycloneDDS)에서 직접 확인.

## 저장소/작업본

| 경로 | 역할 | Git 상태 |
|---|---|---|
| `~/JSND/src/racing_stack` | 기존 정상 JSND (실차 실행 기준, `~/JSND/install`) | HEAD `2091417`, `race.launch.xml`(joy_node 추가)·`mapping.launch.xml` 미커밋, 맵 `11_16_*`, `11_18_*` 미추적 |
| `~/JSND_MPC` | 최신 Integrated MPC 작업 (Jetson이 최신) | 브랜치 `jsnd-integrated-mpc`(main `bc696d5`에서 분기, origin/main보다 1커밋 앞섬) + **미커밋 15개 파일**(`mpc_vehicle` global_waypoints 구독 등), 미추적 `pitwall/`, `mpc_report.txt` |
| `~/JSND_MPC/mpc-build.WFANmz` | mpc_vehicle Jazzy 빌드 + `venv`(osqp 1.1.3, system-site-packages) | — |
| `~/JSND_backups/JSND_MPC_20261122_jetson/` | 오늘 진단 전 전체 스냅샷(tgz) + 미커밋 patch + HEAD | — |

## 기존 JSND 제어 경로 (변경 대상 아님)

`/behavior_strategy` → `controller_manager`(PP, 50 Hz) → `/vesc/high_level/ackermann_cmd`
→ `simple_mux`(RB=autodrive 래치, LB=사람, 선택적 TTC estop) → `/vesc/ackermann_cmd`
→ `ackermann_to_vesc`(speed→ERPM gain 4614, 정지 시 brake 전류 제어) → VESC.

## 최신 Integrated MPC 구성

- `controller/controller/mpc/integrated_mpc.py`: 상태 `[x,y,yaw,v]`, 입력 `[accel, steer]` LTV-QP(OSQP). 속도·조향 동시 최적화. 실패/시간초과 시 STOP(0속도).
- `controller/controller/combined/src/Controller.py`: `lateral_controller=MPC`이면 `integrated_loop()`로 PP 속도·조향을 모두 우회.
- `controller_manager.py`: 파라미터로 PP/MPC 선택. **단, `sim=false`에서 MPC를 거부**(실차는 mpc_vehicle 사용 설계).
- `mpc_vehicle`(Shadow/Bench/Drive Gate), `pitwall/`(173↔174 relay): 과거 검증용 별도 스택. 오늘 최종 아키텍처에서는 사용하지 않음.
- `JSND_MPC/stack_master`는 기존 JSND보다 **오래된 사본**(맵·joy launch·base_system 차이). 통합 시 기준으로 쓰지 않음.
