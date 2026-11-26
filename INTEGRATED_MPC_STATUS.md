# 속도·조향 통합 MPC 구현 및 검증 상태

2026-11-19. 사용자 승인에 따라 시뮬레이션 대상은 기존 **test0** 맵과 글로벌 경로다.
`11_16_3` 자료는 이 PC에 없으며 해당 맵 검증이나 실차 검증으로 간주하지 않는다.
실차 접속·주행과 승인 잠금 해제는 수행하지 않았다.

## 구현

- `controller/controller/mpc/integrated_mpc.py`: ROS 비의존 공통 제어기.
  상태 `[x, y, yaw, v]`, 입력 `[acceleration, steering]`를 LTV QP로 동시 최적화한다(기본 20단계, test0 시뮬레이션 15단계).
  첫 단계는 제어 주기(20ms), 이후는 100ms preview다. QP의 고정 행렬 구조를 재사용한다.
  test0 설정은 속도 상한 2 m/s, 감속 preview는 최대 감속의 30%로 보수적으로 계획한다.
  실제 가감속·조향·횡가속도 제약과 15 ms 계산 기한은 유지한다.
- 속도 프로파일은 목표값 및 상한이다. 곡률의 횡가속도 상한과 역방향 감속 가능성 계산으로
  상한을 더 낮춘다. 가속·감속·조향각·조향 변화율·속도 및 보수적 횡가속도 제약을 적용한다.
  비선형 예측의 횡가속도·추종 오차·실제 경로 투영 위치의 속도 상한을 다시 검사한다.
- 실패/부정확 해/시간 초과/잘못된 입력/제약 충족 불가능 시 0속도 STOP 요청.
  이 요청은 실제 차량의 제동 완료를 의미하지 않는다.
- 시뮬레이션 manager와 `mpc_vehicle` Shadow/Bench/Drive 모두 같은 `IntegratedMPC`를 호출한다.
  PP 속도 보정·조향 계산 및 FTG 대체 주행을 통합 MPC 경로에서 호출하지 않는다.
  TRAILING/FTGONLY 등 미지원 상태는 정지한다. 동적 추월·상대차 추종 성능은 검증하지 않았다.
- `race.launch.xml`/`run_mpc.sh`는 기존 `controller:=pp` 복구 경로와
  `controller:=mpc` 시뮬레이션 경로를 모두 보존한다. MPC는 sim=true에서만 선택되며,
  실차는 `mpc_vehicle` Shadow/Bench/Drive 안전 게이트를 사용한다.
  기존 quali/final launch의 실차 실행은 이 manager에서 거부되므로 실차는 반드시 `mpc_vehicle`를 사용한다.
- 원본 PP 코드와 조향 전용 solver는 보존했다. 복구 스냅샷·Git 기준점·해시는
  `controller/legacy_lateral/README.md`, `manifest.json` 참조.
- 실차 수동 우선, hold-to-run, E-stop latch, 입력/명령 freshness, graph 소유권 검사,
  중립 후 명시 재무장, 독립 watchdog 검증, bench 시간 제한 및 drive 승인 바인딩을 유지한다.
  예시 프로파일의 모든 실차 승인 플래그는 false다. 코드 변경으로 기존 drive 승인 해시는 무효다.
- `/home/sk/JSND` 소스는 수정하지 않았다. 정지 ROS 점검 중 기존 lap_analyser가 underlay에 만든 29바이트 로그 1개는 제거하여 되돌렸다. 재발 방지를 위해 sim에서 legacy lap logger를 비활성화했다. SSH, 차량 접속·주행은 하지 않았다.

## 검증 결과와 한계

- 현재 소스 회귀 검사: **163 passed, 1 skipped**.
  `controller/docs/integrated_validation/test0_final_tests.xml`.
  조향 전용 MPC/PP 보존, 통합 MPC 제약·실패 정지, manager 연결,
  실차 gate 수동 우선·비상정지·watchdog/승인 정책과 ROS adapter/launch selector 포함.
  이 실행에서 건너뛴 항목은 명시적 opt-in이 필요한 Shadow DDS 시험이다.
- 이전 localhost 합성 입력 Shadow DDS 시험과 Humble adapter 시험은
  `ros_adapter_tests.xml`에 기록되어 있다. 실차 시험이 아니다.
- 최신 Humble 빌드: `/home/sk/JSND_MPC/mpc-build.F39akS`,
  `batched-log`에 controller/stack_master 빌드 기록. 이전 mpc_vehicle Humble 빌드 기록은
  `mpc-build.Ap0XnT/vehicle-log`에 있다. Jazzy 실차 완료 마커는 생성하지 않았다.
- QP 재사용, 감속 목표와 안전 상한 분리, 행렬 계산 일괄 처리 적용.
  일괄 처리 전후 11개 상태의 최대 제어 입력 차이는 `8.9e-14`:
  `controller/docs/integrated_validation/batched_equivalence.json`.
- 중간 ROS test0 실험들은 `ros_test0_drive3`, `ros_test0_cached`,
  `ros_test0_conservative`, `ros_test0_final`에 CSV/JSON으로 보존한다.
  충돌 없이 3랩 순환한 시도도 있으나 solver 정지 요청이 있었으므로 연속 성공이 아니다.
  마지막 실행 결과는 아래 추가 기록을 따른다. 실시간 deadline을 늘리거나 실패를 무시하지 않았다.
- 최신 이상적 kinematic 보조 시험도 3랩에 도달했지만 1회의
  `deadline_exceeded`가 발생해 `continuous_success=false`로 기록됐다
  (`controller/docs/integrated_validation/final_test0_supplemental/kinematic.json`).
  따라서 현재 검증 상태에서 ROS 3랩 연속 성공을 주장하지 않는다.
- GUI RViz는 이 headless 검사 환경에서 GLX 오류로 종료됐다. 센서·물리 시뮬레이터의
  주행 검사와 GUI 표시 성공을 구분한다.
- 로컬 Conda Jazzy/Python 3.12 구문·메시지 API import 점검 기록은 기존 검증 자료에 있다.
  `/opt/ros/jazzy`가 없어 지원 스크립트를 통한 Jazzy 전체 빌드는 미수행이다.
- **실차 미검증**: 실제 geometry/servo/ERPM, 맵·초기 자세 정합, 물리 조이스틱/분리,
  비상정지, 독립 watchdog·제동거리, 하드웨어 CPU 지연, 실제 Shadow/Bench/Drive 전부.
  실차 주행 준비 완료 또는 주행 승인이 아니다.

## Solver deadline 수정 및 최종 test0 결과

- 원인: 코너 진입 시 현재 속도가 곡률 기반 속도 상한보다 높을 수 있는데 이를 즉시
  hard upper bound로 넣어 가감속 한계와 충돌했다. 또한 이전 조향각이 새 횡가속도
  조향 envelope 밖에 있을 때 조향 rate 행과 box 행의 교집합이 비었다.
- 수정: 도달 가능한 감속 envelope를 QP 상한으로 사용하고, 첫 조향 단계는 현재
  조향각에서 rate 제한으로 복귀 가능한 범위를 허용했다. 통합 속도·조향 최적화와
  STOP fail-safe는 유지했다.
- 계산량: test0 시뮬레이션 horizon 15→10, `max_iter` 1000→400, 제어 주기 50→40 Hz,
  Solver 제한 12 ms, MPC deadline 20 ms로 조정했다.
- 최종 Humble ROS test0 결과
  (`controller/docs/integrated_validation/ros_test0_tuned40_metrics/result.json`):
  3랩 성공, infeasible/deadline 실패 0회, 충돌 0회, 랩타임 27.412/20.788/20.744 s,
  횡방향 RMSE 0.0463 m, 최대 오차 0.1173 m, Solver 최대 14.736 ms,
  Solver p99 7.378 ms. 제어 주기 p99 39.32 ms, 최대 89.65 ms로 nominal 25 ms보다
  느린 샘플이 있어 CPU 여유가 충분하다고 판단하지 않는다.
- 이 결과는 격리된 Humble 시뮬레이터에서만 얻었다. Jazzy와 실차 Shadow/Bench/Drive는
  검증하지 않았고, 실차 주행 가능 판정으로 확장하지 않는다.

## 시뮬레이션 실행 명령 — test0

2026-11-19 사용자 승인에 따라 로컬에 있는 `test0` 맵과 글로벌 경로를 사용한다.
`11_16_3` 자료는 여전히 없으며 실차용 맵 검증으로 대체하지 않는다.

이 PC에서 첫 번째 터미널:

```bash
cd /home/sk/JSND_MPC
SIM_ONLY_ISOLATED=YES bash run_test0_sim.sh
```

스크립트는 Humble underlay와 격리 MPC 빌드를 읽고, localhost 전용
`ROS_DOMAIN_ID=219`의 빈 도메인 검사 후 `test0`, `sim:=true`, `controller:=mpc`로 실행한다.
먼저 실행한 시뮬레이션이나 observer가 있으면 거부하므로 종료 후 다시 실행한다.
실차 드라이버는 시작하지 않는다. `SIM_ONLY_ISOLATED=YES`는 실제 차량과 분리된 환경에서만 지정한다.

두 번째 터미널(시뮬레이션이 시작된 후):

```bash
cd /home/sk/JSND_MPC
source controller/test/sim_environment.bash
ros2 run joy joy_node
```

패드 RB로 MPC AUTO, LB로 수동 우선. 이 기존 sim mux는 실차 안전 게이트와 다른 구조다.
검증 자동화는 raceline 첫 점으로 시뮬레이터를 reset한 뒤 주행한다.
GUI에서 다른 초기 자세를 쓰면 같은 검증 조건이 아니다.

랩 기록 전용(명령 발행 없음):

```bash
cd /home/sk/JSND_MPC
source controller/test/sim_environment.bash
export SIM_ONLY_ISOLATED=YES
python3 mpc_deployment/record_laps.py --map-dir stack_master/maps/test0 \
  --output log/test0-three-laps --timeout 300
```

출력 폴더는 매 실행마다 새 경로를 사용한다. 실차 실행/승인 조건은 아래와 같이 유지한다.

## 실차 실행 명령 — 여기서는 실행하지 않음

실제 Jazzy 환경에서 실측 프로파일이 준비된 경우의 명령이다. 맵은 프로파일에 넣지 않는다:
shadow는 실행 중인 기존 스택의 `/global_waypoints`를 따르고, bench/drive는 MAP 인자(맵 이름 또는 맵 폴더)로 받는다.
예시 프로파일을 그대로 사용하면 필수 파일/검증 누락으로 차단되는 것이 정상이다.

```bash
cd /home/sk/JSND_MPC
bash build_mpc_vehicle.sh "$MPC_JAZZY_UNDERLAY"
# Vehicle build completed 경로, 실측 프로파일, 검증된 로컬 ROS domain을 지정
export MPC_VEHICLE_BUILD=/출력된/새/mpc-build.XXXXXX
export MPC_PROFILE=/검증된/vehicle.json
export ROS_DOMAIN_ID=차량의_검증된_도메인
bash run_mpc_vehicle.sh "$MPC_VEHICLE_BUILD" "$MPC_PROFILE" shadow
# 물리 E-stop/watchdog 검증 + 바퀴 공중 조건 + 현장 확인 후에만:
bash run_mpc_vehicle.sh "$MPC_VEHICLE_BUILD" "$MPC_PROFILE" bench MAP_NAME
# 현재 코드·설정·맵에 바인딩된 실제 bench 승인 + 통제된 저속 환경 후에만:
bash run_mpc_vehicle.sh "$MPC_VEHICLE_BUILD" "$MPC_PROFILE" drive MAP_NAME   # bench 승인과 같은 맵
```

구체적 joy/안전 bridge/arm/stop/승인 절차는 `VEHICLE_RUN_GUIDE.md`를 따른다.
이 문서와 자동 테스트는 해당 물리 시험이나 승인 기록을 생성·대체하지 않는다.
