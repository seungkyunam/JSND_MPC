# 실차 속도 프로파일과 적용 제한

## 5p0까지만 선택됐던 이유

`c16bced` 기반 최종 실차 릴리스를 만들 때 실행 스크립트와 launch 검증기의 허용 목록을
검증된 단계 상승 범위인 `lowspeed`~`5p0`로 닫았다. 이는 MPC의 구조적 한계가 아니다.
`6p0`, `7p0` 속도 전용 YAML은 이미 있었지만 최종 stage 대상과 허용 목록에서 제외되어
선택할 수 없었고, `8p0`~`10p0` 파일은 없었다.

이번 정리로 `6p0`, `7p0`, `8p0`, `9p0`, `10p0`을 stage하고 두 실행 스크립트 및
launch 검증기에서 선택할 수 있다. 각 파일은 `mpc_max_speed_mps` 한 값만 덮어쓴다.
50 Hz, N=10, 1초 시간격자, 목적함수 가중치, 가감속·횡가속·조향 물리 제약,
VESC 응답 보상 0.12초 및 E-stop에는 변경이 없다.

## 계층별 제한

| 계층 | 설정/동작 | 실제 의미 |
|---|---|---|
| MPC | 선택 프로파일의 `mpc_max_speed_mps` = 1.0~10.0 m/s | 최적화 예측과 controller가 내보내는 속도 명령의 상한. 곡률 기반 횡가속·제동 포락선, local waypoint 속도 및 장애물 안전 cap이 더 낮으면 그 값이 우선한다. |
| ROS2 | 자율주행 명령은 `controller_manager → /vesc/high_level/ackermann_cmd → simple_mux → /vesc/ackermann_cmd`로 전달 | simple_mux는 자율주행 속도를 추가 clamp하지 않는다. `joy_max_speed=1.0`은 MPC 운용 중 LB 수동 조작에만, `ftg_max_speed=1.0`은 FTGONLY fallback에만 적용된다. 11_18_6 IQP raceline 자체 최고속도는 3.901877 m/s이고 sector scaling은 0.6 또는 1.0이므로 이 맵에서는 경로/포락선이 6p0 이상 프로파일보다 먼저 제한한다. |
| VESC ROS driver | `speed_max=23250 ERPM`, `speed_min=-23250 ERPM`, `speed_to_erpm_gain=4614 ERPM/(m/s)`, offset 0 | ackermann 변환기가 m/s 명령을 ERPM으로 바꾸고 driver가 `CommandLimit::clip`으로 ±23250 ERPM에 clip한다. 양의 속도 환산 상한은 `23250/4614 = 5.039012 m/s`이다. 이 값과 VESC 펌웨어/기계적 한계는 변경하지 않았다. |

따라서 직선에서 다른 동적 제한이 없다고 가정해도 액추에이터에 실제 적용되는 구성상 상한은
약 5.039 m/s다. `6p0`~`10p0`은 MPC 상한을 높여 선택할 수 있게 한 것이지 VESC 한계를
높이는 프로파일이 아니다. 11_18_6의 정상 IQP 경로에서는 이보다 낮은 약 3.902 m/s의
경로 속도와 동적 제동/곡률 포락선이 먼저 적용된다. 실제 측정 속도는 타이어 슬립, 배터리,
부하와 보정 오차 때문에 구성상 상한과 다를 수 있으므로 `/vesc/odom` 또는
`/car_state/odom`으로 판정한다.

`/vesc/commands/motor/speed`는 ackermann 변환기 출력이라 driver clip 직전 값이다.
새 recorder는 이 토픽을 함께 저장한다. 23250 ERPM 초과 입력 여부는 확인할 수 있지만,
driver가 VESC에 보낸 clip 이후 패킷을 다시 publish하는 토픽은 아니므로 구성·소스 검증과
VESC telemetry를 함께 사용해야 한다.
