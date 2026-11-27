import csv,json,math
from pathlib import Path
O=Path(__file__).parent/'results'
R=list(csv.DictReader((O/'summary.csv').open()))
def find(mode,ms,servo=60,tau=120):return next(r for r in R if r['mode']==mode and float(r['pose_delay_ms'])==ms and float(r['servo_delay_ms'])==servo and float(r['motor_tau_ms'])==tau)
def fmt(x,n=3):
    try:return f'{float(x):.{n}f}' if math.isfinite(float(x)) else '미측정'
    except (TypeError,ValueError):return '미측정'
def table(keys,labels,rows):
    text='| '+' | '.join(labels)+' |\n|'+'---|'*len(keys)+'\n'
    for r in rows:text+='| '+' | '.join(str(r.get(k,'')) if k in ['mode','run','scenario'] else fmt(r.get(k)) for k in keys)+' |\n'
    return text
primary=[find(mode,ms) for ms in [0,50,100,150,180,200] for mode in ['PP','MPC ON','MPC OFF']]
t=table(['pose_delay_ms','mode','stat_laps','lap_mean_s','lateral_rmse_m','stops_0p5_full'],
 ['위치 지연 ms','제어기','완료 통계 랩','평균 랩 s','횡오차 RMSE m','전체 정지 ≥0.5s'],primary)
ar=[]
for se,ta in [(0,120),(60,120),(100,120),(60,450)]:
 for mode in ['PP','MPC ON','MPC OFF']:
    r=dict(find(mode,100,se,ta));r['scenario']=f'servo {se} / tau {ta} ms';ar.append(r)
act=table(['scenario','mode','stat_laps','lap_mean_s','lateral_rmse_m','stops_0p5_full'],
 ['구동기 조건','제어기','완료 통계 랩','평균 랩 s','횡오차 RMSE m','전체 정지 ≥0.5s'],ar)
reuse=sum(r['reused']=='True' for r in R);fresh=len(R)-reuse
r180=find('MPC OFF',180);r200=find('MPC OFF',200)
on100=find('MPC ON',100);off100=find('MPC OFF',100)
text=f'''# 위치 지연·구동기 응답·MPC 지연 보상과 실차 차이 분석

2026-11-26 / JSND_MPC `final-mpc` 기준 `54aa264`. **실차 재시험이 아니라 기존 오프라인 시뮬레이션 확대와 저장소의 기존 실차 분석 자료 비교**다.

## 결론

- 최종 180 ms 보상을 유지한 MPC는 요청한 위치 지연 0/50/100/150/180/200 ms의 모든 조건에서 11랩을 완료했다(출발 제외 10랩). PP도 해당 6조건에서 모두 11랩 완료.
- MPC 보상 OFF는 180 ms에서 총 {r180['completed_laps']}랩, 200 ms에서 총 {r200['completed_laps']}랩만 완료하고 300 s 제한에 도달했다. 각각 통계 랩 {r180['stat_laps']}/{r200['stat_laps']}개다. 미완주/정지 구간을 버린 뒤 안정적 10랩 성능으로 제시하지 않았다.
- 보상은 모든 조건의 최단 랩을 보장하지 않는다. 예를 들어 100 ms의 평균 랩은 OFF {fmt(off100['lap_mean_s'])} s, ON {fmt(on100['lap_mean_s'])} s. 다만 횡오차·물리 한계·완주 여부를 함께 보아야 한다. 랩만으로 우열을 결정하지 않는다.
- 실차 기록의 느린 주행을 **고정 위치 지연 하나로 설명할 수 없다**. 실차 자료에는 PF 점프/정체, 구동 응답, 장애물 관련 상태 전환, 과거 코드의 명령 제한, 계산 예산 초과가 함께 관측돼 있다. 그 각각의 인과 기여율이나 실차의 실제 위치 지연값은 이번 자료로 식별하지 못했다.

## 실험 설계와 재사용

총 {len(R)}개 조건: 기존 {reuse}개 재사용 + 신규 {fresh}개. 요청 지연 6개×PP/MPC ON/MPC OFF=18개, 기존 120/140 ms 보조 자료 4개, 구동기 민감도 9개다. 기존 0/120/140 ms의 PP 및 MPC ON 원시 trace/진단을 byte-identical 복사하여 재사용했고 `reuse_manifest.csv`에 출처와 SHA256을 기록했다. 재사용 결과를 다시 실행하거나 좋은 결과만 선택하지 않았다.

공통 조건은 기존 비교와 동일: 11_18_6, 동일 초기 rear-axle pose/정지 출발, 50 Hz, wheelbase 0.324 m, 속도 cap 5.039 m/s, a=+4/−6 m/s², 조향 ±0.49 rad/±6 rad/s. PP 원래 속도 프로파일과 sector 0.6 배율 유지, MPC 5p0와 최종 가중치 유지. 장애물 없음, 잡음 없음, GB_TRACK. 11랩 또는 300 s까지, 첫 랩 제외. 원래 제어 코드/YAML을 편집하지 않았다.

**위치 지연**: Cartesian x/y/yaw 입력의 지연. 50/150 ms는 20 ms tick 사이 선형 보간(연속 yaw), 나머지는 정수 tick. 단순히 40/60 ms로 반올림하지 않았다. 초기 과거 값은 출발 pose로 채운다. 원시 trace의 pose_x/y/yaw로 주입 지연을 검증했다.

**중요한 범위 제한**: 기존 실행기와의 비교·재사용을 유지하기 위해 Frenet s/d, waypoint window, 속도/yaw-rate 피드백은 현재값이다. 따라서 이것은 Cartesian pose 경로만 지연한 격리 실험이며, 실제 PF→Frenet→state_machine 전체 파이프라인의 일괄 지연을 재현하지 않는다. 노이즈, 패킷 손실, PF 위치 점프/정체, 가변 지연도 넣지 않았다.

**구동기**: 주 실험은 steering dead time=60 ms, motor first-order tau=120 ms 고정. 조향 지연은 명령 큐의 순수 시간 지연이고, motor tau는 오차에 비례한 가속 응답 시정수다. **tau를 위치 지연과 더해 총 통신 지연으로 해석하지 않는다.** 이 수치는 실차 식별값이 아니라 기존 모델 가정이다.

**MPC 보상 ON/OFF**: ON은 최종 `mpc_latency_comp_s=0.18`, history prediction=true. OFF는 별도 실험용 Controller 인스턴스의 보상 horizon만 0으로 둬 예측을 우회한다. history flag, optimizer/속도/조향/jerk/lead 한계·가중치는 바꾸지 않았다. 저장된 params는 원본 설정이며 `compensation_s`가 실제 실험 개입값이다. 주입 지연에 따라 보상값을 맞춰 튜닝하지 않았다. 이 OFF 반사실 실험은 사용자 요청의 적용 전후 비교를 위한 것이며 배포 설정 변경이 아니다.

구동기 민감도는 pose=100 ms에서 servo 0/60/100 ms, tau 120/450 ms를 한 요소씩 바꿨다. tau=450 ms도 기존 Step51 민감도 가정을 재사용한 것이지 실차 추정 결과가 아니다. 16 ms controller deadline/8 ms solver budget은 그대로이며 노트북 실행시간 변동도 결과에 영향을 줄 수 있다. 단일 궤적의 연속 랩 결과로 독립 반복실험/신뢰구간을 주장하지 않는다.

## 요청 지연별 결과

{t}
평균 랩/RMSE/코너 속도는 **완료한 출발 이후 랩만** 집계. 180 ms OFF는 해당 표본이 없어 미측정, 200 ms OFF의 평균은 6랩까지의 조건부 평균으로 10랩 성공과 동등하게 비교할 수 없다. 미완주 후 장시간 정체도 관측 시간이므로 전체 정지·실패율·전체 주행 오차는 별도로 CSV에 저장했다.

정지: 주행 속도가 처음 0.3 m/s를 넘은 뒤 v<0.1이 ≥0.5 s 지속되는 사건(기존 실차 집계 정의에 맞춤). 이전 시뮬레이션과의 연결을 위해 완료 flying lap의 ≥0.2 s 정지도 별도 열로 보존했다. 정지 1회가 오래 지속될 수 있으므로 횟수만 보지 말고 `run_low_speed_pct`와 총 관측 시간을 함께 확인해야 한다. solver failure는 물리 정지 횟수와 다르다.

180 ms OFF의 상태 로그는 `ValueError:outside_tracking_envelope`가 대부분이고 최종 |d|≈1.10 m에서 0 명령/정체가 유지된다. 이는 **이 시뮬레이션에서 확인한 경로 이탈 후 제어 중단 흐름**이다. 실차 정지의 원인이 모두 같은 종류라는 의미는 아니다.

## 코너 통과 속도

코너1 s=1.491–4.969 m, 코너2 s=8.050–12.323 m. |κ|≥0.35/m 연속 영역을 길이 3등분하여 진입/중간/탈출을 정의했고, 모든 조건에 같은 경계를 사용했다. `corners.csv`와 `02_corner_speeds.png`에 전체 결과와 표본 수를 저장했다. 미완주 조건의 0 표본은 속도 0으로 채우지 않았다. 실제 로그에서는 이와 동일한 경계의 코너 속도 원시 표본을 확보하지 못했으므로 실차 코너별 수치를 새로 만들지 않았다.

## 구동기 응답을 따로 바꾼 결과

{act}
MPC ON의 motor tau를 120→450 ms로 바꾸면 평균 랩이 5.090→5.792 s, 횡오차 RMSE가 0.021→0.275 m로 증가했다. 보상 horizon은 그대로여서 느린 구동 응답까지 해결하지는 못했다. 반대로 PP는 같은 tau 변경에서 6.188→5.644 s로 짧아졌다. 이를 실제 차량의 느린 모터가 더 유리하다는 결론으로 일반화할 수 없다. 모델 내 지연된 가감속/경로/속도 상호작용이며 타이어 동역학을 검증하지 않았다.

이 표의 위치 지연은 모두 100 ms다. 행 사이 차이는 위치 지연 증가의 효과가 아니라 지정한 조향 dead time 또는 모터 tau 변화의 효과다. MPC ON도 보상 180 ms를 고정했으므로 servo 조건을 바꾸면 보상-플랜트 불일치가 달라진다. '가장 빠른 조합이 실제 차량의 지연값'이라는 역추정은 하지 않았다.

## 기존 실차 자료와의 비교

원본 bag은 저장소 밖 Jetson의 `/home/jsnd/bench_logs/bags/...` 경로로 기록돼 있으며 이번 작업 공간에서 확보하지 못했다. 접근 가능한 `/home/sk/bagData`는 `/turtle1/cmd_vel` 24개 메시지의 별개 자료라 제외했다. 원본 재계산 대신 저장소 `docs/results/pp_vs_mpc_existing_20261126/runs.csv`, `laps.csv` 및 Step51/52/53/53B 분석 기록을 재사용했다. 실차 수치는 **기존 추출 결과**로 표기한다.

| 자료 | 코드/프로파일 | 통계 랩 | 평균 랩 s | 횡 RMSE m: 전체 / GB_TRACK | 정지 | 차이 |
|---|---|---:|---:|---|---:|---|
| 2026-11-26 18:20 실차 | Step51 `7ec7263`, 5p0 | 13 | 10.551 | 0.4683 / 0.3818 | 8 | 장애물/회피·추정 오차·구버전 명령 제한 포함 |
| 2026-11-26 18:47 실차 | Step52 `a5e7044`, 7p0 | 14 | 6.508 | 0.4180 / 0.3501 | 0 | 프로파일·코드가 다르고 회피/복귀 전환 포함 |
| 이번 0 ms MPC ON | final-mpc, 5p0 | 10 | {fmt(find('MPC ON',0)['lap_mean_s'])} | {fmt(find('MPC ON',0)['lateral_rmse_m'])} (모델 진실값) | {find('MPC ON',0)['stops_0p5_full']} | 이상적 센서·고정 응답·장애물 없음 |

실차 18:20 기록의 원문 최고 랩 6.607 s와 추출 CSV 6.612 s는 집계 방식/반올림 차이 가능성이 있어 본 표와 그래프는 CSV를 일관되게 사용했다. 원문 14랩과 CSV 13랩도 수동 개입 제외 기준 차이가 있으므로 합치지 않았다.

실차 RMSE는 추정된 `/car_state/odom_frenet d`이며 v>0.3인 표본이고, 실제 추종 진실값이 아니다. GB_TRACK만 골라도 PF 오차와 제어 오차가 섞인다. 시뮬레이션은 실제 모델 좌표의 d이며 이동 표본 RMSE도 별도로 계산했다. 실차 속도 평균은 v>0.1 표본이며, sim의 시간 평균과 별도로 moving mean을 제공한다. 실차 전체 정지와 시뮬레이션 전체 정지는 관측시간이 다르다. 실차 랩의 첫 출발 제외가 시뮬레이션과 완전히 같은지도 원본 없이 보장하지 않는다.

추출 CSV에는 controller=PP로 확인된 run이 없다. controller 공란 기록을 PP로 추정하지 않았다. 따라서 **실차 PP 대 시뮬레이션 PP의 정량 차이와 실차에서의 보상 ON/OFF 차이는 확보하지 못했다.** `04_real_context_NOT_MATCHED.png`는 다른 조건의 참고값이며 인과효과/개선율 그래프가 아니다.

## 원인별 증거 수준

| 요인 | 확인된 자료 | 해석과 제한 |
|---|---|---|
| 고정 Cartesian 위치 지연 | 이번 주입 실험, ON/OFF·완주·오차 | 모델 내에서는 영향 확인. 실차의 PF 지연값이나 고정 지연만의 기여율을 식별한 결과 아님 |
| PF 점프/정체·방향 불일치 | Step51 분석: PF yaw/IMU 불일치, 위치 점프·정체와 heading STOP 동시 관측 | 상관/상태 전환은 기존 분석에 기록. 센서 시계·독립 GT가 없어 지연, 위치 편향, 맵 불일치를 분리 확정 불가 |
| 조향 응답 | Step51: 조향 명령→IMU yaw 응답 상관 지연 약 0.10 s, 상관 0.98 | 명령 전달+servo+차체 yaw dynamics+센서 영향이 섞인 응답 지연. PF 위치 지연 100 ms 또는 순수 servo delay 100 ms의 측정값이 아님 |
| 모터 응답 및 lead | Step51: 직선 lead_limited 62%, optimal a p50 2.4, 실제 가속 1.4–1.6 m/s²; tau≈0.35는 불안정 회귀 추정 | 당시 0.5 lead의 명령 clipping은 기존 로그 근거. final에는 1.2 확장/전환 완화가 이미 있어 과거 비율을 현재 코드로 옮겨 적용하면 안 됨. tau 확정 불가 |
| 코드 차이 | Step52 lead 확장·제동복구, Step53 lead 전환 ramp, Step53B 복귀 판단 변경 | 과거 bag과 final의 성능 차이를 지연 하나로 귀속할 수 없는 교란 요인 |
| 장애물/상태머신 | Step51/53: TRAILING, trailing_without_target, 회피 후 복귀, TTC 이벤트 | 장애물 없는 GB_TRACK 모델에는 없는 동작. 실차 시간 손실 원인 후보/기록된 사건이며 새 실차 실험으로 검증하지 않음 |
| 계산시간 | 추출 실차 compute p99 22.277/23.497 ms, final model의 노트북 실행은 별도 로그 | 일부 real tick이 20 ms를 넘었다는 기록. CPU·solver·센서 부하가 다르고 원본 재현 없어 성능 격차의 기여 초/비율 계산 불가 |
| 실제 물리 모델 | 시뮬레이터는 결합 타이어 마찰/slip/차체 충돌 미구현 | 지연 커짐 또는 보상 OFF에서 코너 절삭/높은 횡가속으로 랩이 짧아질 수 있음. `05_model_audit.png`를 함께 제시할 것 |

Step53B 기록의 헤더→도착 p50≈1/max≈13 ms는 PF 계산에 사용한 스캔 시각부터의 지연이 아니다. 스캔과 PF 스탬프 대응이 낮다는 기존 분석 때문에 이 값을 위치 추정 지연으로 사용하지 않았다.

기존 근거: [실차 추출 CSV](../../docs/results/pp_vs_mpc_existing_20261126/runs.csv), [Step51 실차 분석](../../docs/STEP51_VEHICLE_ANALYSIS.md), [Step52 변경 기록](../../docs/STEP52.md), [Step53 분석](../../docs/STEP53.md), [Step53B 실차 분석](../../docs/STEP53B_RUN.md), [기존 집계 정의](../../jsnd_int/compare_pp_mpc/cmp_analyze.py).

## 파일·재현·검증

- `01_pose_delay.png`: 요청한 지연별 랩·횡오차·정지·완료 랩.
- `02_corner_speeds.png`: 두 코너의 진입/중간/탈출 속도.
- `03_actuator_sensitivity.png`: 위치 지연 고정, 구동기만 변경.
- `04_real_context_NOT_MATCHED.png`: 조건이 다른 실차 참고 비교.
- `05_model_audit.png`: 결합 마찰 한계와 solver 실패.
- CSV: `summary`, `corners`, `laps`, `reuse_manifest`, `real_existing_records`.
- 원시 `*_trace.csv`, `*_diagnostics.jsonl`, `*_run.json`, `*_status_counts.json`.

```bash
cd /home/sk/JSND_MPC_final_comparison
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python delay_study/run.py
OPENBLAS_NUM_THREADS=1 /home/sk/JSND_MPC/.venv/bin/python delay_study/analyze.py
/home/sk/JSND_MPC/.venv/bin/python delay_study/report.py
```

기존 완료 결과는 건너뛰며 덮어 재실험하지 않는다. 새 실험은 별도 결과 디렉터리를 사용해야 한다. 원본 제어 파일 무변경, 재사용 trace hash 일치, 소수 tick 지연 재구성, MPC 보상 이외 설정 동일성, 공통 물리 한계, 완료 랩/시간 일치를 검증했다. 정확한 체크 결과와 적용 범위는 `validation.txt`.
'''
(O/'REPORT.md').write_text(text)
print(O/'REPORT.md')
