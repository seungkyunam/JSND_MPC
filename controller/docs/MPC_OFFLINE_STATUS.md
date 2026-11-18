# JSND MPC 구현·오프라인 검증 현황

작성일: 2026-11-17

## 현재 상태

원본 `/home/sk/JSND`에 조향 전용 Kinematic Bicycle LTV-MPC 구현과 PP/MPC
선택 연결을 추가했다. **OSQP 실제 실행 테스트와 계산시간 측정은 아직 미완료**다.
확인한 시스템 Python 및 thirdimpact/jsnd-humble 환경에는 OSQP가 없으며,
`/tmp` 격리 환경 설치 허용 질문에 대한 답변을 기다리고 있다.
패키지 설치, 전체 빌드, ROS launch, 실제 제어 명령 발행은 수행하지 않았다.

기본값은 PP이며, MPC는 명시적인 휠베이스·확인된 차량 좌표계·양수 조향
한계와 변화율 없이는 활성화되지 않는다. 테스트용 수치를 실차 설정에
입력하지 않았다. Docker 독립 복사본, F1 프로젝트, 실제 지도와 웨이포인트,
LiDAR 및 회피/복귀 플래너는 수정하지 않았다.

## 변경 파일

| 파일 | 변경 내용 |
|---|---|
| `controller/controller/mpc/bicycle.py` | rear-axle bicycle 모델 및 선형화 |
| `controller/controller/mpc/lateral_mpc.py` | 조향만의 QP, OSQP 연결, 조향/변화율 제약, 실패 결과와 시간 측정 |
| `controller/controller/mpc/configuration.py` | 차량 파라미터 확인 및 활성화 조건 |
| `controller/controller/mpc/__init__.py` | 독립 모듈 패키지 |
| `controller/controller/combined/src/Controller.py` | PP/MPC 분기, PP 보정 우회, 실패 정지 요청 처리 |
| `controller/controller/controller_manager.py` | 시작 시 선택, 최종 조향 피드백, 선택적 MPC 계산시간 토픽 |
| `stack_master/config/controller.yaml` | PP 기본값 및 비활성 상태의 MPC 설정 |
| `controller/setup.py` | 선택적 `mpc` 의존성 선언 |
| `controller/test/` | 가상 경로, 회귀 기준값, 단위/연결 테스트, 벤치마크 도구 |
| `controller/docs/MPC_IMPLEMENTATION.md` | 수식, 구조, 설정, 실패 정책, 실행 전 확인 문서 |

기존 Controller 함수 중 변경된 함수는 `__init__`, `main_loop`뿐임을 AST로
확인했다. `calc_speed_command`, 기존 PP 조향 계산 및 보정, 종방향 관련 함수,
AEB 함수의 본문은 그대로다. 추가된 정지는 MPC 실패에 한정된 안전 override이다.

## 완료한 검증

- OSQP 없이 실행 가능한 테스트 **37개 통과**, OSQP 필요 테스트 **27개 미실행**.
- 수정 전 Git commit `2091417c8772d87f76eafcb73f402474481c16a7`에서 추출한
  PP 기준 결과와 네 경로 × 다섯 상태 × 여덟 주기, 총 **160주기 결과 일치**.
- 동일 입력/내부 상태에서 기존 속도 결과 보존, MPC의 PP 전용 보정 우회 확인.
- MPC 실패 정지가 기존 AEB의 2m/s 반환으로 취소되지 않음 확인.
- FTG 우선 선택, watchdog 정지, 최종 발행 조향 피드백 확인.
- OSQP/SciPy import를 차단해도 PP 제어가 가능한지 확인.
- 모델 Jacobian 수치 미분 검증, 필수 파라미터 gate 검증 통과.
- Python 구문 검사 및 `git diff --check` 통과.

테스트는 ROS 메시지 값 객체 및 발행기를 mock으로 대체했다. 실제 ROS
초기화, DDS 통신, 액추에이터 및 전체 timer 주기의 검증은 아니다.
호스트 pytest의 외부 플러그인 버전 충돌이 있어 자동 플러그인 로딩을 끄고
실행했다. 기존 패키지는 수정하지 않았다.

## 수학 모델 교차 검산 — OSQP 검증과 구분

설치를 기다리는 동안 기존 SciPy의 SLSQP로 동일 QP를 임시 교차 검산했다.
이 임시 검산기는 제품 코드에 추가하지 않았고, OSQP를 대체하는 fallback도 아니다.

합성 차량: 휠베이스 0.32m, 최대 조향 0.53rad, 변화율 2rad/s, 주기 0.02s.
이 값은 테스트 fixture이며 실제 차량 파라미터로 확정하지 않았다.

| 가상 경로 | 주기 수 | 속도 | 횡오차 RMS | 최대 횡오차 |
|---|---:|---:|---:|---:|
| 직선 | 200 | 2.0m/s | 0.01988m | 0.08000m |
| 급커브: 반경 1.2m | 100 | 1.5m/s | 0.02783m | 0.07962m |
| S자 | 250 | 2.0m/s | 0.01763m | 0.07468m |
| 정적 장애물 우회·복귀 | 350 | 2.0m/s | 0.01652m | 0.07999m |

우회 경로에서는 합성 장애물 중심 `(5,0)`과 점 차량 사이의 최소 거리가
약 0.9779m였으며 최종 y는 약 -0.000001m로 기준선에 복귀했다.
차체 footprint나 실제 LiDAR·플래너 실행을 포함하지 않으므로 실차 충돌
안전성 또는 실제 장애물 회피 성능의 증거로 해석해서는 안 된다.

## 남은 검증과 계산시간

실제 OSQP의 수렴, timeout 처리, warm-start 반복 동작, 전체 64개 테스트,
그리고 real-clock 벤치마크가 남아 있다. **OSQP 계산시간은 아직 측정하지
않았으므로 50Hz 충족 여부는 확인 불가**다.

설정상 solver 한도는 8ms, MPC 전체 계산 deadline은 15ms이며 제어 주기는
20ms다. 이 수치는 측정 결과가 아니다. `benchmark_mpc.py`는 cold/warm,
P50/P95/P99/max, deadline 실패 수 및 mock 환경의 Controller 전체 호출
시간을 JSON으로 기록하도록 준비되어 있다.

## 내일 실차 적용 전

1. 실제 사용 소스·overlay·Python 및 OSQP 설치 여부 확인. Docker 사본과
   기존 install 결과에는 이번 수정이 자동 반영되지 않는다.
2. 휠베이스, pose 기준점/후륜축 offset, odometry 속도 좌표계, 조향 부호·단위,
   물리 한계·변화율·지연 확인. 기존 PP 소프트웨어 clamp를 실측값으로 간주하지 않는다.
3. 로컬 경로의 순서, 길이, 점 간격, 갱신 주기 및 회피·복귀 연결을 녹화 데이터로 확인.
4. 실제 활성화된 EStop·mux·watchdog 확인. 소스 보존과 안전 노드 활성화는 별개다.
5. 제어 명령을 발행하지 않는 차량 CPU shadow/replay에서 전체 처리시간 검증.
6. 속도 고정 상태에서 추종 가능한 경로인지, planner의 차체 여유거리와 실패
   정지 정책이 충분한지 확인. 이 MPC 자체에는 장애물 충돌 제약이 없다.

수식 및 상세 동작은 [MPC_IMPLEMENTATION.md](MPC_IMPLEMENTATION.md)에 정리했다.
