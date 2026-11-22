# jsnd_int — JSND racing_stack에 통합한 Integrated MPC (실차용)

기존 JSND(`~/JSND/src/racing_stack`, HEAD `2091417`)에 Integrated MPC(속도+조향)를 연결한 **수정 파일만** 담았다.
저장소 루트의 `controller/`, `stack_master/`(sim 전용 MPC 버전)는 그대로 두며 덮어쓰지 않는다.

| 경로 | 내용 |
|---|---|
| `racing_stack/...` | JSND_INT에서 수정/추가한 파일 6개 (JSND racing_stack 경로 그대로) |
| `0000-jetson-uncommitted-launch-edits.patch` | Jetson JSND의 미커밋 launch 수정(joy_node 등) — 통합의 기준 상태 |
| `0001-*.patch`, `0002-*.patch` | 통합 커밋 2개 (`git am` 순서대로) |
| `tests_offline/` | pytest: 속도 setpoint, 제한, STOP 처리 |
| `tests_ros/` | 격리 domain 219, VESC 없이 controller_manager + simple_mux 시나리오 |

MPC 모듈(`controller/controller/mpc/*.py` 5개)은 저장소 루트의 `controller/controller/mpc/`와 **바이트 단위로 동일**하여 복제하지 않았다(패치에는 포함).

## Jetson 적용
```bash
cd ~/JSND_INT/src/racing_stack     # JSND clone (원본 ~/JSND 미수정)
git am /path/to/jsnd_int/0001-*.patch /path/to/jsnd_int/0002-*.patch
cd ~/JSND_INT && source ~/JSND/install/setup.bash
colcon build --base-paths src --packages-select controller stack_master
# osqp 1.1.3만 ~/JSND_INT/pydeps 에 둔다 (시스템 numpy/scipy 유지)
ros2 launch stack_master race.launch.xml map:=<map> controller:=mpc   # controller:=pp 가 기본(기존 그대로)
```
