# 조향 전용 MPC 복구 자료

이 디렉터리는 설치되는 Python 패키지가 아니며 정상 실행 경로에서 사용하지 않는다.
`Controller.py`, `controller_manager.py`, `configuration.py`, `mpc_sim.yaml`,
`vehicle_nodes.py`는 통합 전 adapter/config 스냅샷이다.
`original_*` 파일의 Git 기준점과 SHA256은 `manifest.json`에 기록했다.
현행 `controller/mpc/lateral_mpc.py`, `bicycle.py`도 그대로 유지한다.

복구는 별도 작업 사본에서 먼저 수행한다. 위 스냅샷을 각 원래 위치에 복원하고,
원래 launch/config/runner와 함께 새 isolated overlay를 빌드한다.
`vehicle_nodes.py`의 원래 위치는 `mpc_vehicle/mpc_vehicle/nodes.py`다.
기존 승인 해시는 코드 변경으로 무효가 된다. 실차 승인 파일이나 잠금 상태를
복원·위조해 주행을 허용하지 않는다. 복구 빌드는 시뮬레이션부터 다시 검증한다.

PP 함수는 현행 combined/src/Controller.py에도 보존되어 있으며 golden 회귀
검사에서만 명시적으로 선택한다. 정상 controller_manager는 PP를 거부한다.
