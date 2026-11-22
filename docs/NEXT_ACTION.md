# NEXT_ACTION

## 목표
기존 JSND(`race.launch.xml`)에 `controller:=mpc` 선택만 추가해 Integrated MPC로 실차 저속(≤1.0 m/s) 주행.

## 통합 작업본
`~/JSND/src/racing_stack`에 `git worktree add ~/JSND_INT -b integrated-mpc` (원본 미수정).
미추적 맵 폴더(`11_18_6` 등)는 작업본으로 복사. 별도 colcon 빌드 출력(`~/JSND_INT_ws`).

## 최소 변경 파일 (승인 필요)
| 파일 | 변경 |
|---|---|
| `controller/controller/mpc/` (신규) | `JSND_MPC`에서 `integrated_mpc.py`, `configuration.py`, `bicycle.py`, `__init__.py` 복사 (알고리즘 수정 없음) |
| `controller/controller/combined/src/Controller.py` | MPC 분기 이식 + **B1 수정**: 가속도→목표속도 변환을 내부 명령속도 적분 `v_cmd ← clip(v_cmd + a·dt)`로 하고, 출발 시 최소 구동속도(예 0.3 m/s ≈ 1400 ERPM) 적용. STOP/실패 시 `v_cmd=0` 리셋 |
| `controller/controller/controller_manager.py` | PP/MPC 선택 파라미터, 실차 거부 제거, `FTGONLY`는 기존 FTG 그대로 실행 |
| `stack_master/launch/race.launch.xml` | `controller` 인자(default `pp`) + `mpc_car.yaml` 로드 |
| `stack_master/config/mpc_car.yaml` (신규) | max_speed 1.0, 실측 wheelbase/조향한계, 40 Hz |
| (설정만) simple_mux | 시험 중 `joy_freshness_threshold` 0.25 s, `use_estop:=true` 전달 |
| 실행 환경 | `PYTHONPATH`에 venv site-packages 추가 (osqp) — 시스템 재설치 없음 |

## 시험 순서
1. 빌드: `colcon build --packages-select controller stack_master` → 성공
2. 오프라인: 기존 MPC 단위 테스트 + 정지 상태(v=0) 입력 시 `speed ≥ 0.3` 확인
3. 실차 무구동(VESC 미연결 또는 RB 미선택): `ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc`
   → `ros2 topic hz /vesc/high_level/ackermann_cmd`(≈40 Hz), `/controller/mpc_status`=solved, speed·steer 둘 다 변화
4. **바퀴 띄움**: RB → ERPM·servo 확인, LB/명령 차단(controller kill)→0 확인, 회전 중 STOP 확인
5. 통제 트랙 저속 주행 (물리 정지 수단·관찰자 확보 후)

## ChatGPT 검토 필요
- B1 수정 방식(명령속도 적분 + 최소 출발속도 0.3 m/s) 적절성
- VESC 속도 PID 최소 ERPM 실제값 (VESC Tool에서 확인 필요)
- Gate의 hold-to-run 미이식 상태로 시험해도 되는지 (LB 수동 우선 + 0.25 s freshness로 대체 제안)
