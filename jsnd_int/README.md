# jsnd_int — JSND racing_stack에 통합한 Integrated MPC (실차용)

기존 JSND(`~/JSND/src/racing_stack`, HEAD `2091417`)에 Integrated MPC(속도+조향)를 연결한 **수정 파일만** 담았다.
저장소 루트의 `controller/`, `stack_master/`(sim 전용 MPC 버전)는 그대로 두며 덮어쓰지 않는다.

| 경로 | 내용 |
|---|---|
| `racing_stack/...` | JSND_INT에서 수정/추가한 파일 (JSND racing_stack 경로 그대로). 2026-11-22 `0003`부터 `mpc/integrated_mpc.py`, `safety_gate.py`, `mpc_profile_*.yaml` 포함 |
| `0000-jetson-uncommitted-launch-edits.patch` | Jetson JSND의 미커밋 launch 수정(joy_node 등) — 통합의 기준 상태 |
| `0001-*.patch`, `0002-*.patch`, `0003-*.patch` | 통합 커밋 3개 (`git am` 순서대로). 0003 = OSQP 안정화·100 Hz·E-stop 래치·속도 프로파일 |
| `tests_offline/` | pytest: 속도 setpoint, 제한, STOP 처리 + `test_stabilization.py`(실차 bag 입력 `bench_20261122_mpc_inputs.npz` 재현) |
| `tests_ros/` | 격리 domain 219, VESC 없이 controller_manager + simple_mux 시나리오 |

MPC 모듈 중 `integrated_mpc.py`는 0003에서 수정되어 여기 사본을 둔다(루트 `controller/controller/mpc/`의 원본은 그대로). 나머지 4개는 루트와 동일.

## Jetson 적용
```bash
cd ~/JSND_INT/src/racing_stack     # JSND clone (원본 ~/JSND 미수정)
git am /path/to/jsnd_int/0001-*.patch /path/to/jsnd_int/0002-*.patch /path/to/jsnd_int/0003-*.patch
cd ~/JSND_INT && source ~/JSND/install/setup.bash
colcon build --base-paths src --packages-select controller stack_master
# osqp 1.1.3만 ~/JSND_INT/pydeps 에 둔다 (시스템 numpy/scipy 유지)
ros2 launch stack_master race.launch.xml map:=<map> controller:=mpc   # 기본 mpc_profile:=lowspeed(1.0 m/s), mpc_rate_hz:=100, safety_mode on
# controller:=pp 가 기본(기존 그대로: mux 50 Hz, safety_mode off)
```
