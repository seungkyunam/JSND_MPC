# Integrated MPC v2 설계·검증·배포 요약

- 기준: Step 25 `b1e130f6`; 구현: `07729a7d38f57626651a89ec0fe2aa5444364575`.
- 브랜치: `integrated-mpc-v2`. 기존 로컬 미커밋 변경은 별도 작업 트리로 보존; Step 25는 `preserve/integrated-step25`로도 보존.
- **실차 주행 준비 기준 미충족.** 실차 자동 주행 및 Jetson 배포는 수행하지 않았다. 이 문서 추가에서는 코드 수정 없음.

## 1. 모델·목적함수

상태 $z=[x,y,\psi,v]$, 입력 $u=[a,\delta]$, 축거 $L$인 기존 Kinematic Bicycle Model을 유지한다.

$$z_{k+1}=z_k+h_k[v_k\cos\psi_k,\ v_k\sin\psi_k,\ v_k\tan\delta_k/L,\ a_k]^T.$$

첫 단계 $h_0=0.02$초(50Hz), 이후 설정된 preview 간격을 사용한다. 경로 XY에서 접선·방위·곡률을 계산하며 속도/섹터 열은 사용하지 않는다. 기존 LTV 선형화와 상태 소거 $z_k=M_kU+b_k$를 재사용한다.

$$J=\sum_k\left(\|p_{k+1}-\bar p_{k+1}\|_{Q_p}^2+q_\psi(\psi_{k+1}-\bar\psi_{k+1})^2+r_a a_k^2+r_\delta(\delta_k-\bar\delta_k)^2+r_\Delta(\delta_k-\delta_{k-1})^2\right)-q_s\sum_k t_k^T(p_{k+1}-p_k).$$

$t_k$는 고정된 기준 경로 접선이다. 마지막 상태 추종 비용에는 기존 terminal_scale을 적용한다. 속도 추종 제곱 비용은 제거했고 기존 `q_speed` 설정 키를 진행량 보상 $q_s$로 사용한다. 진행량은 경로 접선에 대한 선형 근사이며 정확한 비선형 arc-length 최적화는 아니다.

## 2. 제약조건·QP 변환

기존 한계 $A,D,Y$는 각각 가속·감속·횡가속 한계다. 실측 없이 상향하지 않는다.

- $-D/\sqrt2\le a_k\le A/\sqrt2$, $|\delta_k|\le\delta_{max}$.
- $|\delta_k-\delta_{k-1}|\le\dot\delta_{max}h_k$.
- $0\le v_{k+1}\le c_k$.
- $V_k=\max(c_{k-1},c_k)$; 첫 단계에는 측정 속도를 포함한다. $|\delta_k|\le\arctan((Y/\sqrt2)L/V_k^2)$를 추가한다.
- 따라서 단계 내 속도 상한 아래에서 $(a_k/a_{long})^2+(a_{y,k}/Y)^2\le1$인 보수적 내부 사각형을 보장한다. $a_{long}=A$ 또는 $D$. 마찰 타원 자체는 실측 식별 모델이 아니다.

좌표 기반 원시 속도 상한은

$$\hat c(s)=\min\{v_{operator},v_{obstacle},\sqrt{0.85(Y/\sqrt2)/\max(|\kappa(s)|,10^{-6})}\}.$$

경로 마지막 점 상한은 0으로 두고, $b=\beta D/\sqrt2$로 후방 계산한다.

$$c_i^2=\min_{j\ge i}\{\hat c_j^2+2b(s_j-s_i)\}.$$

샘플 사이 최소 상한 및 조향 복귀에 필요한 속도 상한도 반영한다. 종단 상태에도 남은 경로 내 제동 가능 상한을 적용한다. 이는 유한 경로 끝 정지 여유이지 강건 불변 종단 집합 증명은 아니다.

소거 후 OSQP 입력은 $\min_U\frac12U^TPU+q^TU$, $l\le A_{QP}U\le u$이다. 속도 행은 $v_{k+1}=v_0+\sum_{j\le k}h_ja_j$이므로 고정된다. 양의 입력 비용과 준양정 상태/변화율 비용으로 $P\succ0$이며 진행량 항은 선형이므로 볼록성이 유지된다.

해를 비선형 모델로 재적분해 연계 가속도·속도·경로·방위·실제 투영 위치의 제동 상한을 검사한다. 허용 오차는 QP/속도 $10^{-4}$, 연계 및 횡가속 검사 $10^{-3}$이다. 불가능한 제동 상한을 측정 속도에 맞춰 완화하지 않는다. 계산 실패 시 기존 최대 2틱 계획 이어쓰기는 현재 경로·장애물 상한이 같고 상태/물리/투영 위치 검증을 통과할 때만 허용한다.

## 3. 변경 사항

모두 `jsnd_int/` 아래에 있다.

| 파일 | 핵심 변경 |
|---|---|
| `racing_stack/controller/controller/mpc/integrated_mpc.py` | 속도 열 의존 제거, 진행량 비용, 연계·제동·종단 상한, 비선형 및 이어쓰기 검사 |
| `racing_stack/controller/controller/combined/src/Controller.py` | TRAILING 안전 상한 별도 전달 및 출력 제한, 기존 자세 지연 보정 적용 |
| `racing_stack/stack_master/launch/race.launch.xml` | `mpc_speed_source` 인자 추가; 기본 `jsnd` 유지 |
| `tests_offline/test_integrated_v2.py` | 독립성·물리 제약·제동·이어쓰기·기존 코드 보존 검사 |
| `tests_offline/compare_v2.py` | 동일 합성 원형 경로 비교 |
| `tests_offline/v2_results.json` | 기존 검증 수치 |

PP, 조향 전용 MPC, FTG, 장애물 회피 경로, TTC, 비상정지 및 하드웨어 보호 코드는 변경하지 않았다. 통합 모드만 속도 열을 무시하며 TRAILING 안전 상한과 운용 최고속도는 유지한다. 저장소 루트의 별도 시뮬레이션용 통합 MPC는 변경하지 않았다.

## 4. 검증 결과

- 117개 통과: v2/조향 MPC 40개 + 기존 루트 제어기 회귀 77개. 후자는 별도 시뮬레이션 코드의 회귀 검사이며 실차 ROS 통합 검증을 대신하지 않는다.
- 속도 열을 0, 0.01, 100, 음수, NaN으로 바꿔도 통합 최적해 동일. 이어쓰기도 속도 열과 독립적이다.
- ROS2 Humble 격리 빌드: `controller`, `stack_master` 성공. 최종 XML 파싱 및 diff 검사 통과.
- 원본 `11_18_6` 맵 없음. 아래는 반경 3m, 운용 상한 1m/s 합성 원형 경로와 동일 운동학 차량/속도 지연 모델의 x86 호스트 결과다.

| 제어기 | 완주 | 플라잉랩(s) | 최대 횡오차(m) | STOP 이벤트 | 계산 p99(ms) |
|---|---:|---:|---:|---:|---:|
| PP | 2랩 | 18.62 | 0.03721 | 0 | 0.47 |
| 조향 MPC | 2랩 | 18.84 | 0.00088 | 0 | 1.61 |
| 통합 v2 | 2랩 | 18.80 | 0.00675 | 0 | 1.66 |

모두 20ms 초과 0회. 비교 시험 solver/deadline은 15/20ms, 실차 launch는 8/16ms다. 이 결과로 Jetson 50Hz 또는 실차 우위를 주장하지 않는다.

### 기존 벤치 재생의 `braking_envelope_infeasible` 18틱

입력은 `tests_offline/bench_20261122_mpc_inputs.npz`의 611틱이다. 기존 기록은 과거 제어기 주행 상태이며 v2 폐루프 결과가 아니다. 재생 설정은 `test_integrated_v2.config(max_speed=2.)`: $A=1,D=2,Y=2$, $\beta=0.3$, 첫 단계 20ms다. 기존 결과는 solved 593틱, 거부 18틱, 계산 p99 2.44ms였다.

문서 작성 시 동일 입력의 거부 조건을 재확인했다.

- `drive_1243` 17틱, `drive_1246` 1틱. **18번의 별도 정지 이벤트가 아니라 18개의 거부된 샘플**이다.
- 모든 거부 샘플에서 첫 단계부터 $v_0-(2/\sqrt2)\times0.02>c_0+10^{-4}$였다. 최대 허용 감속을 적용해도 20ms 안에 새 상한에 도달할 수 없다.
- 측정 속도 범위 1.2183~1.9725m/s; 첫 단계 부족분 0.00482~0.17356m/s.
- 해당 첫 단계 상한은 곡률 기반 후방 제동 상한이 결정했다. 조향 복귀 상한은 이 값들을 추가로 낮추지 않았고, 경로 끝 정지 상한도 지배 원인이 아니었다.
- 예: 데이터 인덱스 452에서 $v_0=1.47464$, 도달 가능한 최소 속도 1.44636, 상한 1.27280m/s → 0.17356m/s 부족.
- 이는 OSQP 시간초과/반복 실패나 웨이포인트 속도 충돌이 아니라 **새 보수적 제동 허용 영역 밖의 기록 상태에 대한 QP 전 사전 거부**다. 기존 로그의 물리적 주행 불가능성을 증명하지 않는다. 이전처럼 상한을 완화하면 검증 의미가 사라지므로 그대로 STOP 처리했다.

## 5. 미해결 문제·실차 판정

- 벤치 거부 18틱, 원본 맵 비교, Jetson/Jazzy 전체 ROS 안전 재생 및 부하 상태 50Hz 검증이 남았다.
- 조향속도 3.2rad/s, 자세 지연 보정 0.1초, 실제 가감속·횡가속·타이어 연계·VESC 속도 추종은 실측 필요. 기존 수치를 상향하지 않았다.
- 선형화 오차, 보수적 사각형/제동 상한, 유한 경로 끝 가정으로 STOP 또는 성능 저하 가능. 강건 재귀 실현 가능성은 증명하지 않았다.
- 속도 setpoint 적분과 실제 가속도의 일치도 실측 필요. 비선형 계획 검증은 실제 차량 안전 보장이 아니다.
- **현재는 래치 유지 벤치 준비 단계이며 지상 주행·속도 증가는 보류한다.**

## 6. Jetson 배포 및 단계별 확인

기존 `~/JSND_INT` Step 25 환경과 `~/JSND` underlay가 있다는 전제다. 실행 중인 race/VESC 스택을 먼저 종료한다. 아래 명령은 제공용이며 실제 Jetson에서 실행하지 않았다.

```bash
# 최초 1회: 기존 저장소/작업 트리와 분리
 git clone -b integrated-mpc-v2 https://github.com/seungkyunam/JSND_MPC.git ~/JSND_MPC_v2
# 이미 clone했다면 위 줄 대신 아래 업데이트만 실행
 git -C ~/JSND_MPC_v2 pull --ff-only
 git -C ~/JSND_MPC_v2 rev-parse HEAD

mkdir -p ~/JSND_backups
backup=~/JSND_backups/pre_v2_$(date +%Y%m%d_%H%M%S).tgz
files=(
  controller/controller/mpc/integrated_mpc.py
  controller/controller/combined/src/Controller.py
  stack_master/launch/race.launch.xml
)
tar -C ~/JSND_INT/src/racing_stack -czf "$backup" "${files[@]}"
printf '%s\n' "$backup"  # 복구에 쓸 실제 경로를 기록
for f in "${files[@]}"; do
  cp ~/JSND_MPC_v2/jsnd_int/racing_stack/"$f" ~/JSND_INT/src/racing_stack/"$f"
done
source /opt/ros/jazzy/setup.bash
source ~/JSND/install/setup.bash
cd ~/JSND_INT
colcon build --base-paths src --packages-select controller stack_master
source install/setup.bash
```

빌드 성공 후 바퀴를 띄우고 전원 차단 담당을 배치한다. 시작 래치를 해제하거나 RB로 구동하지 않고 확인한다.

```bash
ros2 launch stack_master race.launch.xml map:=11_18_6 \
  controller:=mpc mpc_speed_source:=integrated \
  mpc_profile:=lowspeed mpc_rate_hz:=50 \
  use_estop:=true safety_mode:=true safety_engage_hold:=true
```

별도 터미널에서 같은 ROS/overlay 환경을 source한 뒤 각각 확인한다.

```bash
ros2 param get /controller_manager mpc_speed_source
ros2 param get /controller_manager mpc_max_speed_mps
ros2 topic echo /controller/mpc_status
ros2 topic hz /vesc/high_level/ackermann_cmd
ros2 topic echo /controller/mpc_elapsed_ms
```

구동 없는 확인 → 원본 맵/ROS 안전 재생 → 실측 및 Jetson 주기 검증 → 저속 지상 시험 순서다. 현재 미해결 항목 때문에 래치 해제·지상 주행 명령은 제공하지 않는다. 기본 `mpc_speed_source:=jsnd`는 기존 조향 MPC다.

## 7. 복구

스택을 종료하고 위에서 기록한 백업 경로를 지정한다. 이 백업은 교체한 3개 파일의 배포 직전 상태이며 다른 파일을 되돌리지는 않는다.

```bash
backup=~/JSND_backups/pre_v2_실제타임스탬프.tgz
tar -C ~/JSND_INT/src/racing_stack -xzf "$backup"
source /opt/ros/jazzy/setup.bash
source ~/JSND/install/setup.bash
cd ~/JSND_INT
colcon build --base-paths src --packages-select controller stack_master
source install/setup.bash
```

Git 기준 성공 버전은 `b1e130f6`이다. 원본 미커밋 변경을 지우는 `reset --hard`는 사용하지 않는다.

## 8. 실차 준비 후속 확인 (2026-11-24, 기준 e6dffde)

- 원격 fetch 결과 기준 문서 이후 변경 없음. `race.launch.xml` → manager → Controller 분기를 확인했다: `controller:=pp`는 기존 PP, `controller:=mpc mpc_speed_source:=jsnd`는 기존 속도+조향 MPC, `mpc_speed_source:=integrated`는 v2다. 기본값은 계속 PP / MPC 선택 시 jsnd다. 운용 제어기·물리 한계·안전 코드 수정 없음.
- 기존 18틱 원인은 위 분석을 재사용했다. v2 폐루프 확인을 위해 실제 `mpc_car.yaml` 가중치/물리 설정, 50Hz, OSQP 8ms/전체 16ms로 직선·S자·회피 경로를 각각 1/2m/s 운용 상한에서 500틱씩 실행했다. 정지 출발, 매 틱 로컬 경로 갱신, 실제 Controller 속도 setpoint 적분을 사용했다.
- 총 3,000틱 모두 solved, `braking_envelope_infeasible` 0틱. 기존 로그 18틱이 이 조건의 신규 폐루프에서 재발하지 않았다. 모든 경로/외란에서 재발하지 않는다는 증명은 아니다.
- 정확한 pose, 즉시 조향, 가정된 속도 응답 지연 0.12초의 합성 시험이다. 지연 보정은 정확한 pose에 맞춰 0초로 설정했다. 실차 0.1초 보정·센서 지연/잡음·서보 지연 검증은 별도로 남는다. 오차는 이산 경로점까지 거리이며 순수 횡오차와 다르다.
- 재현: `PYTHONPATH=controller:controller/test:jsnd_int/tests_offline OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/validate_v2_readiness.py`. 저장 결과: `jsnd_int/tests_offline/v2_readiness_results.jsonl`. ROS 그래프나 하드웨어에 연결하지 않는다.
- 로컬에 11_18_6 없음. 기록된 Jetson `jsnd@172.20.10.4`에 읽기 전용 SSH를 시도했으나 연결 시간초과. 따라서 원본 맵 확보, Jetson 배포/Jazzy 빌드/50Hz 실측은 **미수행**이다. 연결 가능 시 `scp jsnd@172.20.10.4:~/JSND_INT/src/racing_stack/stack_master/maps/11_18_6/global_waypoints.json /tmp/11_18_6_global_waypoints.json`으로 경로 원본부터 확보한다.

### 세 제어기 실행 명령 (동시에 실행하지 않음)

6절의 백업·3파일 배포·Jazzy 빌드를 먼저 수행한다. 각 명령 실행 전 이전 스택이 종료되어 VESC 드라이버가 중복 실행되지 않는지 확인한다. **아래 명령은 래치 유지 확인용이며 주행 허가가 아니다.** PP는 `mpc_profile` 속도 제한 대상이 아니므로 PP를 1m/s 시험이라고 간주하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/JSND/install/setup.bash
source ~/JSND_INT/install/setup.bash

# 1. 기존 PP
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=pp \
  use_estop:=true safety_mode:=true safety_engage_hold:=true joy_max_speed:=1.0

# 2. 기존 조향 전용 MPC
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc \
  mpc_speed_source:=jsnd mpc_profile:=lowspeed mpc_rate_hz:=50 \
  use_estop:=true safety_mode:=true safety_engage_hold:=true joy_max_speed:=1.0

# 3. 신규 통합 MPC v2
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=mpc \
  mpc_speed_source:=integrated mpc_profile:=lowspeed mpc_rate_hz:=50 \
  use_estop:=true safety_mode:=true safety_engage_hold:=true joy_max_speed:=1.0
```

### Jetson 읽기 전용 확인 절차 및 판정

같은 환경을 source한 별도 터미널에서 실행한다. 모니터는 기존 기본값 off를 유지한다.

```bash
ros2 param get /controller_manager lateral_controller
ros2 param get /controller_manager mpc_speed_source
ros2 param get /controller_manager control_loop_rate_hz
ros2 param get /controller_manager mpc_max_speed_mps
python3 ~/JSND_MPC_v2/jsnd_int/tests_ros/bench_check.py 30
ros2 topic echo /vesc/ackermann_cmd
ros2 topic echo /simple_mux/estop_state
ros2 topic echo /simple_mux/active_host
ros2 topic echo /controller/mpc_cycle_ms
ros2 topic echo /controller/mpc_loop_period_ms
```

- 선택 모드/50Hz/lowspeed 1.0m/s 로드를 확인한다(MPC에 해당). `active_host=estop`, 시작 래치 유지, 최종 `/vesc/ackermann_cmd` 속도 0을 확인한다. PP의 high-level 속도는 래치 중에도 0이 아닐 수 있으므로 최종 mux 출력을 확인해야 한다.
- MPC 계산 p99 <16ms, 전체 cycle p99 <20ms, 실제 주기 약 20ms/50Hz인지 30초 이상 확인하고 deadline/STOP 사유를 보존한다. 주기 tail과 누락이 있으면 50Hz 달성으로 판정하지 않는다. `bench_check.py`만으로 전체 cycle 시간은 검증되지 않아 추가 토픽을 함께 확인한다.
- 래치 상태에서는 안전 출력 0과 설정/데이터 신선도는 확인 가능하지만 TTC·조이스틱 유실·odom/waypoint 유실·FTG/TRAILING 전환의 전체 동작을 증명하지 못한다. 해당 통합 안전 시나리오는 미검증으로 남긴다.
- `tests_ros/fake_world.py` 및 기존 `tests_ros/run.sh`에는 reset_estop/조이스틱 모의 입력이 포함되어 있어 이번 금지 조건에서는 실행하지 않는다. 실제 스택으로 시험 입력을 발행하지 않는다.
- 복구는 7절의 백업 tar 복원 후 동일 Jazzy 재빌드. 단순히 `mpc_speed_source:=jsnd`로 선택하는 것은 제어기 전환이며 소스 복구와 구분한다.

**최종 판정: 래치 유지 벤치 절차 준비 완료. 원본 맵 폐루프·Jetson/Jazzy 빌드·부하 상태 50Hz·안전 통합 및 차량 파라미터 실측이 남아 지상 자율 주행 테스트 준비 완료로 판정하지 않는다.**
