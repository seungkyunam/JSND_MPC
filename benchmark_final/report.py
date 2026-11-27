"""Generate Korean presentation notes from saved simulation measurements only."""
import csv,json
from pathlib import Path
O=Path(__file__).parent/'results'
rows=list(csv.DictReader((O/'summary.csv').open()))
base={r['controller']:r for r in rows if r['pose_delay_ms']=='0'}
p,m=base['pp'],base['mpc']
def f(r,key):return f'{float(r[key]):.4f}' if r[key] else '미측정'
table='| 지표 | PP | Integrated MPC |\n|---|---:|---:|\n'
for label,key in [('통계 랩 수','stat_laps'),('최고(최단) 랩 [s]','best_lap_s'),('평균 랩 [s]','mean_lap_s'),
 ('랩 표본 표준편차 [s]','lap_std_sample_s'),('최고 속도 [m/s]','max_speed_mps'),('시간 평균 속도 [m/s]','mean_speed_mps'),
 ('횡오차 MAE [m]','lateral_mae_m'),('횡오차 RMSE [m]','lateral_rmse_m'),('횡오차 P95 [m]','lateral_p95_m'),
 ('최대 절대 횡오차 [m]','lateral_max_m'),('실제 조향각 RMS [rad]','actual_steer_rms_rad'),
 ('최대 절대 실제 조향각 [rad]','actual_steer_max_rad'),('실제 조향 변화율 RMS [rad/s]','actual_steer_rate_rms_radps'),
 ('최대 절대 실제 조향 변화율 [rad/s]','actual_steer_rate_max_radps'),('주행 중 정지 [회]','actual_stops'),
 ('차체 충돌 [회]','collision_count'),('후륜축 기준점 점유 셀 진입 [회]','rear_axle_occupied_events')]:
    table+=f'| {label} | {f(p,key)} | {f(m,key)} |\n'
corners=list(csv.DictReader((O/'corner_thirds.csv').open()))
ct='| 코너 | 구간 | PP 속도 [m/s] | MPC 속도 [m/s] |\n|---|---|---:|---:|\n'
for c in ['1','2']:
 for ph,ko in [('entry','진입'),('middle','중간'),('exit','탈출')]:
    values=[next(r['mean_speed_mps'] for r in corners if r['run']==base[ctrl]['run'] and r['corner']==c and r['phase']==ph) for ctrl in ['pp','mpc']]
    ct+=f'| {c} | {ko} | {float(values[0]):.3f} | {float(values[1]):.3f} |\n'
st='| 공통 위치 지연 | PP 평균 랩 [s] | MPC 평균 랩 [s] | PP/MPC 횡오차 RMSE [m] | PP/MPC 횡가속도 >4.5 비율 |\n|---|---:|---:|---|---|\n'
for delay in ['0','120','140']:
    a,b=[next(r for r in rows if r['controller']==ctrl and r['pose_delay_ms']==delay) for ctrl in ['pp','mpc']]
    st+=f"| {delay} ms | {f(a,'mean_lap_s')} | {f(b,'mean_lap_s')} | {f(a,'lateral_rmse_m')} / {f(b,'lateral_rmse_m')} | {f(a,'actual_ay_over_4p5_pct')}% / {f(b,'actual_ay_over_4p5_pct')}% |\n"
text=f'''# final-mpc: PP vs Integrated LTV-MPC 오프라인 비교

작성일: 2026-11-26. 기준 커밋: `54aa264` (`origin/final-mpc`). 실험과 산출물은 `benchmark_final/`에만 추가했다. 기존 제어 코드·설정·맵은 수정하지 않았다.

## 발표에 사용할 결론과 적용 범위

**동일한 기존 오프라인 자전거 모델에서, PP 기존 속도 프로파일과 최종 MPC 가중치를 유지했을 때 평균 랩은 6.420 → 5.128초(20.1% 단축)였다. 최고속도는 거의 같고, 시간 차이는 주로 코너에서 발생했다.**

이 수치는 **실차 기록이 아니며, 타이어 한계·차체 충돌까지 검증한 성능 결과도 아니다.** 기존 모델은 종·횡 결합 마찰을 강제하지 않는다. 기준 조건에서도 결합 마찰 타원 초과가 PP {f(p,'combined_ellipse_over_pct')}%, MPC {f(m,'combined_ellipse_over_pct')}%다. 따라서 실제 물리적으로 가능한 랩타임 및 무충돌 10랩의 입증은 현재 도구만으로 완료할 수 없다. 아래 값은 동일한 단순 모델의 관측값으로만 사용할 수 있다.

PP는 원래 코너 속도 배율 0.6을 그대로 사용하고, Integrated MPC는 waypoint 속도열을 읽지 않고 자체 속도를 결정한다. **이 비교는 기존 PP 운용 설정 대 최종 MPC 운용 설정 비교이며, 동일 속도 계획에서 제어기 자체만 비교한 실험이 아니다.**

## 조건과 재현 방법

- 맵: 저장소 `jsnd_int/racing_stack/stack_master/maps/11_18_6`, 경로 길이 13.9134 m. 맵·raceline·speed_scaling이 모두 있고 양쪽 완주가 가능하여 Spielberg로 대체하지 않았다.
- 각 조건에서 각 제어기 연속 11랩, 출발 1랩 제외, 10랩 통계. 공통 위치 지연 0/120/140 ms의 3조건, 총 66랩. **기준 결과는 0 ms**이며 가장 좋은 결과를 골라 대표값으로 사용하지 않았다.
- 동일 초기 위치: raceline 첫 점의 x/y/yaw, 정지 출발. 동일 rear-axle kinematic bicycle, wheelbase 0.324 m, 제어·물리 dt=0.02 s.
- 공통 플랜트 한계: 속도 23250/4614=5.0390 m/s, 가속 +4/감속 −6 m/s², 조향 ±0.49 rad, 조향 변화율 ±6 rad/s. VESC 1차 응답 tau=0.12 s, servo 순수 지연 60 ms. 기존 lap_sim/Step51 가정 재사용이며 실차 식별값이 아니다.
- 장애물 입력 `None`, GB_TRACK, 잡음 없음. 정적 맵 벽은 기준점 점유 검사만 수행. 타이어 slip, 접촉 반력, 차체 footprint, ROS 센서/스케줄링, TTC, 상태머신 전환을 시뮬레이션하지 않는다.
- PP: 원래 `controller.yaml` + `dynamic_controller.yaml`, waypoint speed + 5개 sector scaling 유지. Sector1(23–47), Sector3(87–119)의 0.6 배율 유지.
- MPC: `mpc_car.yaml` + 기존 `mpc_profile_5p0.yaml`, `step45_run.sh mpc`에 맞춰 integrated, history_prediction=true, heading_robust=false. 기존 성능비교의 **5p0**를 사용했다. 이것이 사용자가 확인한 마지막 실차 운용 프로파일이라는 뜻은 아니다. 최근 장애물 실차 시험 lowspeed=1.0과 혼동하지 말 것.
- 가중치 q_position=12, q_heading=16, q_speed=8, r_accel=0.05, r_steer=0.2, r_rate=3, terminal_scale=3. time_weighted_accel_cost=false. horizon=12, 첫 구간 0.02 s, 이후 0.05 s. 실차 launch와 같은 16 ms deadline / 8 ms solver budget. 노트북 측정이며 Jetson 시간 보장이 아니다.
- MPC 내부 제약(5 m/s, ±5 m/s², ay=4.5, steer=0.4, rate=3.2)은 원래 그대로다. **공통 플랜트의 물리 한계와 제어기 내부 보수적 제약은 구분했다.** PP에 MPC 내부 제약이나 새 속도 계획을 넣지 않았다.
- 기존 `lap_sim`의 track loader / converter / Controller builder / plant 공식을 재사용했다. 오래된 builder가 누락한 `mpc_cmd_accel_lead_mps=1.2`만 실제 YAML 값으로 전달했다. 기존의 낡은 `mpc_layers` 진단은 사용하지 않고 최종 Controller의 실제 진단을 기록했다.

```bash
cd /home/sk/JSND_MPC_final_comparison
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python benchmark_final/run.py
OPENBLAS_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python benchmark_final/analyze.py
/home/sk/JSND_MPC/.venv/bin/python benchmark_final/report.py
```

실행기는 이미 저장된 run.json을 덮어쓰지 않고 건너뛴다. 새로운 재실험은 별도 결과 디렉터리/복사본에서 실행해야 한다. 실행 중 초기 진단 로그 저장 오류 1건은 제어 출력과 무관한 dict 키 중복이었고, 실행기만 고쳐 MPC를 재시작했다. 먼저 완료한 PP 결과는 재실행하지 않았다. 원본 실행 로그에 이 이력을 보존했다.

## 기준 조건의 정량 결과

{table}
랩 경계는 Frenet 누적 진행거리가 경로 길이의 정수배를 넘은 첫 tick이다. 시간 분해능은 20 ms. 표준편차는 ddof=1이며, 잡음 없는 결정론적 주행에서 PP의 0은 실제 반복 재현성이나 통계적 우월성을 의미하지 않는다. 10랩은 독립 실험 10회가 아니다. 속도는 시간 평균, 횡오차는 raceline에 대한 부호 있는 d의 절댓값/RMS다. 실제 조향각과 명령 조향각을 CSV에서 구분했다.

정지는 출발 랩 제외 v<0.1 m/s가 0.2 s 이상 지속되는 사건. solver 실패는 정지와 별개이며 이 실행에서 0건이었다. 기준점 점유 0은 차체 충돌 0의 증거가 아니다. 차체 충돌은 **미측정(null/빈 CSV)**으로 남겼다.

## 구간별 속도와 시간 차이

코너 구간은 경로 |κ|≥0.35/m의 연속 영역을 길이 기준 3등분했다. 코너1 s=1.491–4.969 m, 코너2 s=8.050–12.323 m. 제어기 결과와 무관하게 같은 경계로 집계했다.

{ct}
1 m 고정 구간 분석에서 s=2–4 m는 MPC가 0.428 s/랩, s=8–11 m는 0.564 s/랩 짧다. 두 영역 합계 0.992 s로 전체 1.292 s 차이의 약 76.8%다. 이 구간은 PP 0.6 속도 배율 영역과 크게 겹친다. s=5–8 m 차이는 합계 0.006 s로 작다. 마지막 s=13–13.913 m에서는 MPC의 체류시간이 0.006 s 더 길었지만 20 ms 샘플링, 다른 횡오프셋, 경계 배치의 영향을 받으므로 의미 있는 성능 역전으로 해석할 수 없다. 상세는 `segment_differences.csv`.

### 직선 가속 성능

기존 Step51 분류를 따라 |κ|<0.12/m를 엄격한 직선으로 정의했다. 이 소형 맵의 해당 구간 체류시간은 PP 0.200, MPC 0.182 s/랩에 불과하다. 기준 조건 평균 종가속도는 PP −1.336, MPC −0.765 m/s²로 **양쪽 모두 이미 감속 중인 표본이 많다**. 양의 가속(a>0.1) 표본은 PP 없음, MPC 평균 0.254 m/s²다. 따라서 여기서 'MPC의 직선 가속이 우수하다'거나 0→3 m/s 가속시간을 주장할 근거가 없다. 코너1 탈출 평균 가속은 PP 2.811, MPC 2.408 m/s²이나 MPC가 더 높은 진입 속도를 유지하므로, 순간 가속이 작아도 랩은 빠를 수 있다. 측정되지 않은 가속시간은 만들지 않았다.

## 원인 분리: 확인된 작동과 미확인 인과효과

| 요소 | 코드 및 이번 로그에서 확인된 내용 | 주장 가능한 범위 |
|---|---|---|
| 목적함수 | 위치/방향 오차·입력·조향변화 페널티와 진행거리 보상. 상태 속도 오차의 대각 가중치는 0이며 q_speed=8은 진행거리 보상에 사용됨 | 단순한 목표속도 추종 QP로 설명하면 부정확. 랩 차이는 PP 속도 배율 차이도 포함 |
| 가속도 페널티 | r_accel=0.05, a² 페널티. 첫 20 ms와 뒤 50 ms 단계의 페널티 시간 가중은 OFF | 가속 억제/미래로 가속을 미루는 경향은 구조적 가능성. 가중치 변경 실험을 하지 않았으므로 '몇 초 손실'로 인과 정량화하지 않음 |
| 곡률 속도 제한 | sqrt(0.85×4.5/|κ|), brake_lead=0.12 s, g-g 제동 역방향 pass, horizon 끝 정지 조건. MPC는 PP waypoint 속도열 무시 | 코너 속도와 제동 개시를 제한. 제한 해제 실험 없이 각 항의 독립 기여 초 단위 추정은 불가 |
| 모터/속도 명령 | 성공 시 가속 적분→jerk 제한→예측 최고속도·측정속도+lead·5 m/s 중 최소로 clip. 실제 하드웨어 cap=5.039 m/s도 공통 적용 | 기준 MPC lead clip {f(m,'lead_limited_pct')}%, plan peak clip {f(m,'plan_limited_pct')}%, jerk 제한 {f(m,'jerk_limited_pct')}%, 양쪽 하드웨어 cap 0%. 따라서 이번 조건에서 모터 최대속도 cap/lead clip이 주원인이라는 근거 없음 |

MPC 1.2 m/s lead가 선택된 비율은 {f(m,'lead_1p2_pct')}%이며, 실제로 잘렸는지는 별도 lead_limited 플래그로 판정했다. 확장 lead가 선택됐다는 사실 자체는 포화를 뜻하지 않는다. 모든 비율은 출발 랩 제외 전체 tick 기준이고 서로 배타적이지 않다. 실제 모터 응답 tau와 독립 가감속 제한도 작동하므로 optimizer 가속=실제 가속으로 취급하지 않았다. 기록은 `*_diagnostics.jsonl`에 보존했다.

## 지연 민감도와 물리 검증의 한계

{st}
120/140 ms에서도 같은 조건을 양쪽에 적용했다. MPC의 0.18 s 보상 설정은 변경하지 않았다. 120 ms 위치 지연+60 ms servo 지연 조건은 이 보상과 맞으므로 기준 0 ms보다 추종 오차가 줄었다. 이를 실차 지연이 실제 120 ms라는 증거로 사용할 수 없다.

기준 0 ms는 개별 횡가속도 4.5 m/s²를 양쪽 모두 넘지 않았지만, 결합 마찰 검사 `(a/A)^2+(ay/4.5)^2≤1`에서는 양쪽 모두 위반했다(A=가속 4/제동 6, 공통 플랜트 값). 최대 지표는 PP {f(p,'combined_ellipse_max')}, MPC {f(m,'combined_ellipse_max')}. 이 검사는 가정한 타원이며 실측 타이어 성능 곡선이 아니다. 기존 플랜트는 해당 타원을 강제하지 않아 실제 가능한 궤적으로 인증할 수 없다. 지연 조건의 PP 최대 횡가속도는 8.06/8.66 m/s²로 단순 모델의 한계가 더 분명하다.

완전한 물리 한계·충돌 검증을 포함한 비교에는 검증된 동역학/타이어 모델, 차체 외형·기준점, 충돌 검출이 추가로 필요하다. 이번에는 환경을 새로 구축하거나 가중치를 튜닝하지 않았고, 측정되지 않은 결과를 채우지 않았다.

## 실차 자료와의 분리

저장소 `docs/results/pp_vs_mpc_existing_20261126/summary.md`는 기존 실차 기록의 별도 분석이다. cap/layout/controller 미상, 서로 다른 운용 조건이 포함되어 이 실험과 병합하지 않았다. `STEP53B_RUN.md`의 최근 실차 시험은 lowspeed 1 m/s·장애물 회피 주행이므로 본 5p0·장애물 없는 오프라인 결과와 직접 비교하지 않았다. **새로운 실차 실험은 수행하지 않았다.**

## 발표용 파일

- `comparison_delay0.png`: 기준 랩타임·속도·횡오차·구간 시간. `comparison_delay6.png`, `comparison_delay7.png`: 민감도 전용.
- `corner_speeds.png`: 코너 진입/중간/탈출 속도. `steering_speed.png`: 마지막 랩 속도·조향·조향 변화율.
- `delay_sensitivity.png`: 지연 민감도(물리적으로 검증된 성능으로 제시하지 말 것).
- `physical_limit_audit.png`: 양쪽의 결합 마찰 한계 초과를 보여주는 필수 한계 설명 그래프.
- `summary.csv`, `laps.csv`, `segments.csv`, `segment_differences.csv`, `corner_thirds.csv`: 정량값.
- `phases.csv`: 기존 Step51 κ 및 dκ/ds 위상 정의도 보존. 중간 구간 표본이 작으므로 발표에는 corner_thirds를 사용.
- `*_trace.csv`, `*_diagnostics.jsonl`, `*_run.json`: 원시 시계열·진단·실제 설정. `manifest.json`: 커밋·소스 SHA256.

발표 문구: “기존 오프라인 자전거 모델의 동일 조건에서, 기존 PP 속도 계획 대비 최종 Integrated MPC 설정의 평균 랩이 약 20% 짧았다. 차이는 주로 코너 속도에서 발생했다. 타이어 결합 한계 및 차체 충돌은 검증하지 못했으므로 실차 성능 향상 수치로 일반화하지 않는다.”
'''
(O/'REPORT.md').write_text(text)
print(O/'REPORT.md')
