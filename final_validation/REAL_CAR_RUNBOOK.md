# 다음 실차 비교 실험 실행서

이 문서는 명령과 순서만 준비한다. 이번 작업에서는 배포와 주행을 수행하지 않았다. 모든 MPC
프로파일은 기존 A 정지 래치, LB 수동, RB 자율 선택, stale odom/waypoint watchdog, TTC E-stop,
조향·가감속·횡가속·결합 한계를 그대로 사용한다.

## 빌드와 실행 명령

Jetson에서 새 브랜치를 배포한 뒤 `stack_master controller`만 재빌드하고, 설치 공간에
`mpc_profile_final_1s*.yaml`이 존재하는지 먼저 확인한다.

```bash
cd ~/JSND_INT
colcon build --packages-select controller stack_master --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/JSND/install/local_setup.bash
source ~/JSND_INT/install/local_setup.bash
```

PP 비교 실행(비상정지와 안전 mux를 명시적으로 켠다):

```bash
ros2 launch stack_master race.launch.xml map:=11_18_6 controller:=pp use_estop:=true safety_mode:=true virtual:=false gp_predictor:=false
```

기존 0.57초 Integrated MPC:

```bash
bash ~/JSND_INT/run/race_mpc.sh 5p0 11_18_6 integrated
```

개선된 1초 Integrated MPC:

```bash
bash ~/JSND_INT/run/race_mpc.sh final_1s 11_18_6 integrated
```

## 단계별 속도 프로파일

같은 최종 N=10/1초/가중치 설정에서 속도 ceiling만 올린다. 각 단계의 분석 gate가 통과하기
전에는 다음 단계로 가지 않는다.

```bash
bash ~/JSND_INT/run/race_mpc.sh final_1s_1p0 11_18_6 integrated
bash ~/JSND_INT/run/race_mpc.sh final_1s_1p5 11_18_6 integrated
bash ~/JSND_INT/run/race_mpc.sh final_1s_2p25 11_18_6 integrated
bash ~/JSND_INT/run/race_mpc.sh final_1s_3p0 11_18_6 integrated
bash ~/JSND_INT/run/race_mpc.sh final_1s_4p0 11_18_6 integrated
bash ~/JSND_INT/run/race_mpc.sh final_1s 11_18_6 integrated
```

## 기록과 분석

레이스 stack을 시작한 뒤 별도 terminal에서 프로파일 이름으로 기록한다.

```bash
bash ~/JSND_INT/run/speed_rec.sh final_1s_1p0
# 주행 종료 후
tmux send-keys -t rec C-c
sleep 2
tmux kill-session -t rec
python3 ~/JSND_INT/run/speed_report.py ~/bench_logs/bags/final_1s_1p0_<timestamp> --vmax 1.0
```

직접 rosbag을 시작해야 할 때는 `speed_rec.sh`가 기록하는 topic을 그대로 사용한다. 이 스크립트는
odom/frenet, 명령, IMU/scan, mux/E-stop, MPC status/diagnostics, OSQP elapsed, 전체 loop period와
cycle을 MCAP에 저장한다. 분석기는 첫 랩 제외 랩타임, 추종 오차, 속도, 조향 안정성, 제동,
OSQP/전체 주기 P50/P95/P99/MAX와 20 ms 초과율, 계산 실패 원인을 출력한다.

## 비교 절차

1. 타이어·조향 중립·배터리 상태를 맞추고 차를 같은 시작점/방향에 둔다. E-stop, A 정지 래치,
   LB 수동, RB 자율 전환을 정지 상태에서 확인한다.
2. `final_1s_1p0`에서 11랩을 기록한다. 첫 랩을 버리고 10랩 통계를 확인한다. 새로운 STOP,
   조향 포화 지속, 경로 RMSE 증가, 20 ms 주기 초과 burst, 위치추정 jump가 있으면 중단한다.
3. 1.5→2.25→3.0→4.0→5.0 m/s 순서로 반복한다. 각 단계에서 새 bag과 로그를 보존한다.
4. 같은 배터리 구간에 기존 0.57초 MPC 11랩, 개선 MPC 11랩, 안전 mux를 켠 PP 11랩을
   순서 교차해 실행한다. 첫 랩 제외 통계를 비교한다.
5. 최종 판정은 랩타임만으로 하지 않는다. RMSE, 물리 한계 추정, OSQP/주기 P99/MAX, 실패율,
   정지, TTC/E-stop 개입과 localization consistency가 모두 허용 범위여야 한다.

시뮬레이션의 PP 랩은 횡/결합 물리 한계를 위반했으므로 해당 속도를 실차 목표로 사용하지 않는다.
