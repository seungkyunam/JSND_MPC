# 통합 속도·조향 MPC 제어 메커니즘

이 문서는 현재 `JSND_MPC` 소스의 구현을 수학식으로 설명한다. 설명의 기준은
`controller/controller/mpc/integrated_mpc.py`의 `IntegratedMPC.solve()`와
`_reference()`이며, 시뮬레이션 설정은 `stack_master/config/mpc_sim.yaml`이다.
아래 수식의 좌표는 차량 뒤 차축(rear axle)을 기준으로 한다.

## 1. 먼저 보는 전체 구조

제어기는 매 제어 주기마다 다음 순서로 동작한다.

1. ROS2의 차량 위치, yaw, 현재 속도와 글로벌 경로에서 잘라낸 순서 있는 local waypoint를 받는다.
2. `Controller.integrated_loop()`가 차체 기준점을 rear axle 기준으로 보정하고,
   측정 속도와 waypoint의 `[x, y, target_speed]`를 `IntegratedMPC.solve()`에 전달한다.
3. `IntegratedMPC._reference()`가 차량에 가장 가까운 경로 위치를 찾고, 경로의 arc length를
   따라 N개의 미래 기준 상태와 곡률 기반 속도 상한을 만든다.
4. 현재 기준 궤적 주변에서 차량 모델을 선형화하고, 모든 미래 상태를 가속도와 조향 입력의
   선형 함수로 압축한다.
5. OSQP가 가속도와 조향을 함께 포함한 QP를 풀고, 첫 입력 `a_0`, `delta_0`만 차량 명령으로 사용한다.
6. `IntegratedMPC.solve()`는 QP 해를 비선형 차량식으로 다시 전파해 횡가속도, 추종오차,
   속도 프로파일 위반을 검사한다. 실패하면 속도 0과 STOP 상태를 반환한다.

ROS 연결은 `controller/controller/controller_manager.py`의 `control_loop()`와
`Controller.integrated_loop()`에 있다. MPC 모드에서는 `Controller.main_loop()`가
PP의 속도·조향 계산으로 내려가지 않으며, `controller_manager.py`도 `FTGONLY`에서
FTG 대체 명령을 사용하지 않고 정지 명령을 만든다. 실차용
`mpc_vehicle/mpc_vehicle/nodes.py`의 `VehicleController.tick()`도 같은
`controller.mpc.integrated_mpc.IntegratedMPC`를 호출한다.

Pure Pursuit에서는 조향각을 lookahead 기하로 계산하고 속도는 별도 규칙/프로파일에서
결정한다. 이 MPC에서는 결정변수 하나의 벡터 안에 가속도와 조향각이 동시에 들어가며,
속도 오차·위치 오차·조향 변화·물리 한계를 같은 최적화 문제에서 다룬다. PP 코드는
삭제되지 않았고 `controller:=pp` 복구 경로로 남아 있지만, `controller:=mpc`에서는
그 경로가 호출되지 않는다.

## 2. 차량 모델

### 상태와 입력

`IntegratedMPC.solve()`의 입력 상태는 다음 4차원 벡터다.

\[
  z = \begin{bmatrix}x & y & \\psi & v\end{bmatrix}^{T}
\]

여기서 `x,y`는 rear axle 위치, `psi`는 yaw, `v`는 전진 속도(m/s)다. 최적화 입력은

\[
  u = \begin{bmatrix}a & \\delta\end{bmatrix}^{T}
\]

이며 `a`는 종방향 가속도, `delta`는 조향각(rad)이다. 소스에서 이 순서는
`controls[:, 0] = acceleration`, `controls[:, 1] = steering`으로 고정되어 있다.

### 연속시간 모델

`integrated_mpc.py`의 비선형 예측식(`predicted`를 만드는 부분)은 다음 rear-axle
kinematic bicycle 모델에 해당한다.

\[
\begin{aligned}
\dot{x} &= v\cos\psi,\\
\dot{y} &= v\sin\psi,\\
\dot{\psi} &= \frac{v}{L}\tan\delta,\\
\dot{v} &= a.
\end{aligned}
\]

여기서 \(L\)은 `IntegratedConfig.wheelbase`다. 시뮬레이션에서는
`wheelbase: 0.3302`가 사용된다. 이 식은 새로운 속도 제어식과 조향 제어식을
분리하지 않고 하나의 상태방정식으로 연결한다.

### 이산시간 모델

`solve()`는 첫 단계에 제어 주기 `c.dt`, 이후 단계에 `c.preview_dt`를 사용한다.
각 단계의 \(\Delta t_k\)에 대해 실제 예측 검사는 다음 forward Euler 형태다.

\[
\begin{aligned}
x_{k+1} &= x_k + \Delta t_k v_k\cos\psi_k,\\
y_{k+1} &= y_k + \Delta t_k v_k\sin\psi_k,\\
\psi_{k+1} &= \psi_k + \Delta t_k\frac{v_k}{L}\tan\delta_k,\\
v_{k+1} &= v_k + \Delta t_k a_k.
\end{aligned}
\]

구현 위치는 `IntegratedMPC.solve()`의 `predicted[:, 3]`, `predicted[:, 2]`,
`predicted[1:, 0]`, `predicted[1:, 1]` 계산이다.

### 선형 MPC인가, 비선형 MPC인가?

최종적으로 OSQP에 전달되는 문제는 **선형 시변(LTV) MPC를 압축한 convex QP**다.
매 주기 기준 상태 \(z^r_k\), 기준 입력 \(\delta^r_k\) 주변에서 차량식을
선형화해 다음 형태를 만든다.

\[
z_{k+1}\approx A_k z_k+B_k u_k+c_k.
\]

`aa`가 \(A_k\), `bb`가 \(B_k\), `affine`이 \(c_k\)에 해당한다. 이후 `all_m`과
`all_b`로

\[
z_k = M_k U+b_k
\]

형태로 모든 상태를 입력 벡터 하나에 대해 표현한다. 따라서 OSQP가 푸는 것은
비선형 제약을 직접 포함하는 NMPC가 아니다. 다만 해를 얻은 뒤 원래의 비선형
bicycle 식으로 다시 예측하고(`predicted`), 횡가속도·오차·속도 상한을 검사한다.
그러므로 정확한 표현은 **비선형 모델을 매 주기 선형화해 푸는 LTV-QP MPC + 비선형
안전 검증**이다.

## 3. 미래 예측

### 예측 길이와 시간 간격

시뮬레이션 `stack_master/config/mpc_sim.yaml`은 다음과 같다.

| 항목 | 값 | 코드 의미 |
|---|---:|---|
| `mpc_horizon` | 10 | 현재 입력부터 미래 10개 제어 입력 |
| `control_loop_rate_hz` | 40 Hz | `c.dt = 1/40 = 0.025 s` |
| `mpc_preview_dt` | 0.1 s | 두 번째 이후 예측 단계 간격 |

따라서 첫 입력은 25 ms 동안 적용되는 현재 제어 입력이고, 나머지 예측점은 100 ms
간격으로 배치된다. 총 horizon은 \(0.025+9\times0.1=0.925\)초에 걸친 입력
계획이다. 기본 `IntegratedConfig`는 N=20이지만 시뮬레이션 설정이 N=10으로
덮어쓴다. 실차 `VehicleController`는 profile에서 별도 horizon을 구성하며,
알고리즘 자체는 동일하다.

### 기준 경로와 기준 속도

`_reference()`는 local waypoint를 먼저 중복점 제거하고, 차량 위치를 각 선분에
투영해 가장 가까운 arc-length 위치를 찾는다. 경로 접선으로 yaw를 계산하고 `np.unwrap`
으로 yaw wrap을 처리한다. 이후 arc length를 따라 미래 샘플을 만든다.

경로의 원래 속도 프로파일을 \(v^{profile}(s)\)라고 하면 코드의 기본 상한은

\[
v^{hard}(s)=\min\left(v^{profile}(s), v_{max},
\sqrt{\frac{a_{y,max}}{|\kappa(s)|+\epsilon}}\right).
\]

여기서 \(\kappa\)는 `_reference()`의 경로 곡률, \(a_{y,max})는
`max_lateral_accel`이다. 코드에는 보수적인 preview cap도 있고,
`braking_preview_ratio`를 사용한 뒤방향 감속 pass가 있다. 이 pass는 미래의 낮은
속도 구간을 미리 보고 현재 목표를 낮춘다.

기준 상태는

\[
z^r_k=[x^r_k,y^r_k,\psi^r_k,v^r_k]^T
\]

이고, `reference`의 마지막 열에 현재 속도와 미래 목표 속도가 들어간다. 현재 측정
속도가 이미 sharp한 상한보다 높을 수 있으므로 `solve()`는

\[
v^{reachable}_k=v_0-a_{max,dec}\sum_{i<k}\Delta t_i
\]

를 계산해 해당 순간 실제로 도달 가능한 상한을 만든다. 이것은 즉시 감속할 수 없는
상태에서 QP를 불가능하게 만들지 않기 위한 envelope이며, 원래 profile cap을 목표로
계속 감속한다.

## 4. 최적화 문제

### 결정변수

N=10일 때 OSQP의 결정변수는 20개다.

\[
U=[a_0,\delta_0,a_1,\delta_1,\ldots,a_{N-1},\delta_{N-1}]^T.
\]

상태 자체를 별도 decision variable로 두지 않고 `all_m`, `all_b`로 압축한다.

### 목적함수

코드의 `h`, `g` 구성은 OSQP 표준형

\[
\min_U \frac12 U^T P U+q^T U
\]

에 대응한다. 의미상 원래 목적함수는 다음과 같다.

\[
\begin{aligned}
J=\sum_{k=1}^{N} &\left[
q_p((x_k-x^r_k)^2+(y_k-y^r_k)^2)
 +q_\psi(\psi_k-\psi^r_k)^2
 +q_v(v_k-v^r_k)^2\right]\\
&+\sum_{k=0}^{N-1}\left[r_a a_k^2+r_\delta(\delta_k-\delta^r_k)^2\right]\\
&+r_{\Delta\delta}\sum_{k=0}^{N-1}(\delta_k-\delta_{k-1})^2.
\end{aligned}
\]

첫 조향 변화의 \(\delta_{-1}\)은 measured `previous_steering`이다. 코드의
`weights[-1] *= terminal_scale` 때문에 마지막 상태 오차에는 terminal scale이
곱해진다.

시뮬레이션의 실제 값은 다음과 같다.

| 기호 | 설정값 | 코드 필드 |
|---|---:|---|
| \(q_p\) | 12.0 | `q_position` (기본값) |
| \(q_\psi\) | 8.0 | `mpc_q_heading` |
| \(q_v\) | 8.0 | `mpc_q_speed` |
| \(r_a\) | 0.15 | `mpc_r_accel` |
| \(r_\delta\) | 0.2 | `mpc_r_steer` (기본값) |
| \(r_{\Delta\delta}\) | 2.0 | `mpc_r_rate` |
| terminal scale | 3.0 | `mpc_terminal_scale` (기본값) |

`stacked.T @ (weights * (all_b-ref[1:]))`가 상태 추종의 선형항을 만들고,
`d.T @ d`가 조향 변화율 비용을 만든다. `g[1::2] -= r_steer*delta`는
조향 기준값 \(\delta^r_k\)를 조향 비용에 반영한다.

### 제약조건

#### 가속도와 속도

시뮬레이션 값은

\[
-2.5\le a_k\le1.5\quad [m/s^2].
\]

선형화된 속도 행 `rows = all_m[:,3,:]`와 `all_b[:,3]`를 사용해

\[
0\le v_k=M^v_kU+b^v_k\le v^{cap}_k
\]

를 QP에 넣는다. `v^{cap}`은 profile, `max_speed`, 곡률 횡가속도 상한, backward
braking, 현재 상태에서 도달 가능한 감속 envelope를 반영한다.

#### 조향각

기본 최대 조향각은 `max_steer_rad=0.4189`다. 속도에 따른 횡가속도 제한을
조향 box로 바꾸어

\[
|\delta_k|\le
\min\left(\delta_{max},
\arctan\frac{L a_{y,max}}{v_{max,k}^2}\right)
\]

를 적용한다. 시뮬레이션의 \(a_{y,max}=3.0\,m/s^2\)다.

#### 조향 변화율

`d`는 현재와 다음 조향 입력의 차분 행렬이다. 따라서

\[
|\delta_0-\delta_{prev}|\le \dot\delta_{max}\Delta t_0,
\quad
|\delta_k-\delta_{k-1}|\le \dot\delta_{max}\Delta t_k
\]

가 된다. 시뮬레이션의 `max_steer_rate_radps=3.2`다. 첫 단계 조향이 새 횡가속도
box 밖에 있더라도 rate 제한으로 복귀 가능한 첫 범위는 허용한다.

#### 횡가속도와 사후 검증

QP에서는 속도에 따른 조향 box로 횡가속도를 보수적으로 제한한다. 해를 얻은 뒤에는
비선형 예측에 대해 직접

\[
a_y=\max(v_k^2,v_{k+1}^2)\frac{|\tan\delta_k|}{L}
\le a_{y,max}
\]

를 검사한다. 동시에 `max_tracking_error`(기본 1.0 m),
`max_heading_error`(기본 0.7 rad), 실제 예측 위치에서의 speed profile 상한도
검사한다. 이 검사는 QP에 넣은 선형 근사가 실제 bicycle 식에서 여전히 안전한지
확인하는 단계다.

### OSQP가 계산하는 것

`IntegratedMPC.__init__()`가 OSQP와 SciPy sparse matrix를 준비하고,
`solve()`가 `P`, `q`, `A`, `l`, `u`를 만든다. OSQP는 다음 문제를 푼다.

\[
\min_U \frac12 U^TPU+q^TU
\quad\text{subject to}\quad l\le AU\le u.
\]

`A`는 조향/가속도 box, 조향 차분, 선형화된 미래 속도 행을 쌓은 행렬이다.
첫 solve 뒤에는 OSQP factorization을 재사용하고 `Px`, `Ax`, `q`, `l`, `u`만
업데이트한다. `Ax`도 매번 갱신하는 이유는 속도 행이 현재 선형화에 따라 변하기
때문이다. warm start에는 이전 `U`가 사용된다.

OSQP 상태가 정확히 `solved`가 아니거나, 해가 유한하지 않거나, 행렬 제약을 벗어나면
해를 사용하지 않는다. `solver_s`와 iteration은 `IntegratedResult`에 보관되며,
전체 deadline은 `perf_counter()` 기준 `elapsed`로 검사한다.

## 5. 실제 주행에서의 동작

### 직선 주행

직선에서는 곡률 \(\kappa\)가 작으므로 curvature speed cap이 `max_speed`나
원래 speed profile보다 낮아지지 않는다. 위치·yaw·속도 오차 비용이 작아지는 방향으로
가속도 \(a_k>0\)를 선택하고, 조향은 경로 yaw에 맞는 작은 값과 조향 변화율 비용을
선택한다. 결과적으로 첫 명령은 목표 속도에 가까워지는 speed와 거의 일정한 steering이다.

### 코너 진입

미래 local path의 곡률이 커지면

\[
v_{curve}(s)=\sqrt{a_{y,max}/|\kappa(s)|}
\]

가 작아진다. `_reference()`의 backward braking pass가 그 낮은 미래 속도를 현재
목표에 미리 반영한다. 최적화는 속도 오차를 줄이기 위해 가속하지 않고 음의
`a_k`를 선택하며, 동시에 \(v^2|\tan\delta|/L\)가 한계를 넘지 않도록 조향각을
선택한다. 즉 코너 진입 감속과 조향이 별도 순차 로직이 아니라 같은 U에서 결정된다.

### 코너 탈출

곡률이 다시 작아지면 speed cap이 올라가고, 속도 오차 항이 양의 가속도를 허용한다.
조향은 경로 접선 yaw로 돌아가며 `r_rate` 때문에 급격한 조향 변화를 피한다.
첫 입력만 적용한 뒤 다음 40 Hz 주기에서 새 위치·속도·경로로 다시 풀기 때문에,
고정된 한 번의 계획을 끝까지 실행하지 않는 receding-horizon 방식이다.

### 계산 실패와 정지 처리

다음 경우 `IntegratedMPC.reject()`가 호출된다.

- 상태, waypoint, 속도, 이전 조향각이 유효하지 않음
- 경로가 퇴화했거나 차량이 tracking/heading envelope 밖임
- horizon을 만들 만큼 경로가 길지 않음
- OSQP가 solved 상태가 아니거나 QP가 infeasible
- 선형 제약 또는 해가 유한성 검사를 통과하지 못함
- 비선형 횡가속도·추종오차·heading·실제 speed profile 검증 실패
- 전체 계산 시간이 `deadline`을 넘음

reject 결과는 이전 조향각을 안전한 범위로 clip한 값, `speed=0`, `acceleration=0`,
`SafetyRequest.STOP`이다. `Controller.integrated_loop()`와
`controller_manager.py`는 실패 결과에서 speed 0을 출력하고, `mpc_vehicle`는
`health=false`와 정지 상태를 gate에 전달한다. Gate는 watchdog, E-stop, 수동 조이스틱
우선권 및 명시적 재무장을 유지하며 자동으로 주행을 재개하지 않는다.

## 구현 범위의 한계

이 설명은 구현된 MPC에만 해당한다. 장애물 회피, 상대차 추월 최적화, full dynamic
vehicle model, 타이어 slip 추정, 가속도 변화율(jerk) 제약은 이 통합 MPC의 목적함수나
상태에 구현되어 있지 않다. `FTGONLY` 같은 지원하지 않는 behavior state는 MPC가
대체 제어기로 전환하지 않고 정지한다.

주요 소스:

- `controller/controller/mpc/integrated_mpc.py`: 모델 선형화, 기준 경로, QP, 제약, 사후검증
- `controller/controller/combined/src/Controller.py`: ROS controller와 MPC 연결
- `controller/controller/controller_manager.py`: 제어 주기, 상태 입력, 명령 publish, STOP 처리
- `controller/controller/mpc/configuration.py`: ROS parameter에서 `IntegratedConfig` 생성
- `stack_master/config/mpc_sim.yaml`: test0 시뮬레이션 N=10, 40 Hz, 제한값
- `mpc_vehicle/mpc_vehicle/nodes.py`: 실차 Shadow/Bench/Drive에서 같은 solver 호출
