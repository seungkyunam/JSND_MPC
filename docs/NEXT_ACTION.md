# NEXT_ACTION

## 목표
JSND_INT `controller:=mpc`로 바퀴를 띄운 상태(bench)에서 모터·조향 구동과 정지 동작 확인 → 통제 트랙 저속 주행.

## 실행 전 확인 (사용자 결정 필요)
1. 조향 한계: `mpc_max_steer_rad` 0.4 / `mpc_max_steer_rate_radps` 3.2 확정 또는 실측값으로 수정
2. VESC Tool에서 속도 PID 최소 ERPM 확인
3. simple_mux `joy_freshness_threshold` 1.0 → 0.25 s 적용 여부 (설정만), `use_estop:=true` 사용 여부
4. 물리 비상정지 수단: LB(수동, 스틱 중립=0) + VESC 전원 차단. 시험자 1명은 항상 LB 대기

## 실행 명령 (Jetson, domain 173)
```bash
source /opt/ros/jazzy/setup.bash && source ~/JSND_INT/install/setup.bash
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc
# 다른 터미널
ros2 topic echo /controller/mpc_status
ros2 topic hz /vesc/high_level/ackermann_cmd     # ≈40 Hz
ros2 topic echo /vesc/commands/motor/speed       # ERPM
```
롤백: `controller:=pp` 또는 `~/JSND/install` 사용.

## Bench 순서 (바퀴 띄움)
1. 초기 자세 설정 → `mpc_status` = solved 확인 (RB 누르기 전)
2. RB → ERPM 상승·servo 변화 확인 / LB → 0 확인
3. RB 상태에서 controller_manager 종료 → 1 s 내 0
4. 맵 위치를 흔들어 STOP 요청 → 0 확인

## ChatGPT 검토 필요
- setpoint 적분 방식과 lead 0.5 m/s 적절성
- FTGONLY 이후 경로 이탈 시 MPC가 STOP(재합류 안 함) → 허용 여부
