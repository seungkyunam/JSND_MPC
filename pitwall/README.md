# JSND_MPC 노트북 pitwall (X11 포워딩 없음)

```
[Jetson] 차량 스택 (domain 173, 127.0.0.1 전용, 변경 없음)
            │  mpc_pitwall_relay (화이트리스트 중계)
            │   173→174: /map /tf /tf_static /scan(10Hz) /pf/viz/inferred_pose
            │            /pf/viz/particles(5Hz) /car_state/odom(10Hz)
            │            /global_waypoints → /mpc_pitwall/global_path (nav_msgs/Path)
            │            /mpc_vehicle/gate_status /mpc_vehicle/controller_status
            │   174→173: /initialpose 만 (frame=map, 유한값, 1회/초, gate armed 시 거부)
[노트북] RViz (domain 174, WiFi) — mpc_pitwall
```

- 노트북은 domain 174에만 참여하므로 `/joy`, `/vesc/*`, `/mpc_vehicle/arm|stop`, 파라미터 변경이
  173(Gate/Estop/VESC)에 도달할 경로가 없다. 중계 노드는 173에 `/initialpose` 외에 아무것도 발행하지 않으므로
  gate의 graph ownership 검사에도 영향이 없다.
- 기존 `run_mpc_vehicle.sh`, `vehicle_mpc.launch.py`, MPC/속도/회피 코드, 기존 JSND pitwall은 수정하지 않았다.
- MPC 예측 horizon은 현재 controller가 토픽으로 발행하지 않으므로 표시되지 않는다(표시되는 것은 MPC 기준 경로).

## Jetson
```bash
mpc_pitwall_relay            # = ~/JSND_MPC/pitwall/mpc_pitwall_relay.sh
```

## 노트북 설치 (최초 1회, 노트북 터미널)
```bash
JETSON=jsnd@172.20.10.4
mkdir -p ~/.local/bin ~/.config/msnd
scp $JETSON:JSND_MPC/pitwall/laptop/mpc_pitwall ~/.local/bin/
scp $JETSON:JSND_MPC/pitwall/laptop/mpc_pitwall.rviz ~/.config/msnd/
chmod +x ~/.local/bin/mpc_pitwall
```
`~/.config/msnd/cyclonedds-laptop.xml`은 기존 JSND pitwall 설정(READMEforlaptop.md 6번)을 그대로 쓴다.
네트워크가 바뀌었으면 6번을 Jetson 새 IP로 다시 실행한다.

## 노트북 실행
```bash
mpc_pitwall                      # Conda msnd_pitwall(Jazzy) RViz
MPC_PITWALL_HUMBLE=1 mpc_pitwall # 대체: 시스템 Humble RViz
mpc_pitwall shell                # 같은 환경의 CLI (ros2 topic list/hz 확인용)
```

## 최소 입력 절차 (2026-11-20 추가)
노트북 설치(진단+설치, 재실행 안전):
```bash
ssh jsnd@172.20.10.4 'cat JSND_MPC/pitwall/laptop/install_mpc_pitwall.sh' | bash
```
`laptop/mpc_pitwall` 또는 rviz를 고치면 `laptop/make_installer.sh`로 설치 스크립트를 다시 생성한다.
노트북 DDS는 기존 JSND 파일과 분리된 `~/.config/msnd/cyclonedds-mpc-pitwall.xml`을 쓴다.

Jetson bench+relay(tmux 'mpc'), 노트북 터미널에서:
```bash
ssh -t jsnd@172.20.10.4 JSND_MPC/pitwall/mpc_bench_session.sh        # 기본 맵 11_18_6
ssh -t jsnd@172.20.10.4 JSND_MPC/pitwall/mpc_bench_session.sh stop   # 전체 종료
```
