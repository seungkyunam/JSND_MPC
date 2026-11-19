# 속도·조향 통합 MPC 시뮬레이션

통합 MPC 실행은 `controller:=mpc sim:=true`로 선택한다. 기존 `controller:=pp` 경로와 PP/조향 전용 MPC 소스도 복구용으로 보존한다.
실행 명령·의존성·11_16_3 자료 준비 및 검증 상태는 [최신 보고서](../INTEGRATED_MPC_STATUS.md)를 따른다.

기존 steering-only 검증의 랩타임은 통합 MPC 성능 근거가 아니다.
실제 11_16_3 맵이 없는 상태에서 다른 맵 결과를 해당 맵의 완주로 간주하지 않는다.
실차는 `race.launch.xml`이 아니라 별도 `mpc_vehicle` 안전 게이트 경로만 사용한다.
