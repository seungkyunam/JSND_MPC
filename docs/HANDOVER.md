# HANDOVER — 통합 MPC 실차 작업 인수인계 (2026-11-25, Step 50 시점)

새 담당자는 이 문서 → `STEP50_STEP49_RUN_ANALYSIS.md` → `STEP49_*`~`STEP45_*` 순으로 읽는다. `WORK_STATUS.md`/`NEXT_ACTION.md` 앞부분은 11-22~24 기록이라 오래됐다.

## 1. 현재 상태
- 목표: 기존 JSND `race.launch.xml`의 controller_manager에서 `controller:=mpc`(Integrated MPC, 속도+조향)로 주행. PP 속도 대체(`mpc_speed_source: jsnd`)는 중단, `integrated` 사용.
- 최신 코드: Step 49. Jetson `~/JSND_INT/src/racing_stack` 브랜치 `integrated-mpc` HEAD `89e1011` = 빌드(`~/JSND_INT/install`, controller·stack_master). 미커밋 변경 없음.
- 이 저장소(`seungkyunam/JSND_MPC`, 브랜치 `integrated-mpc-v3`)의 `jsnd_int/racing_stack/`가 JSND_INT 수정 파일 사본(2026-11-25 대조: 일치. v3에만 `mpc_profile_8p88.yaml` 추가 존재). 패치 `jsnd_int/0000~0017-*.patch`.
- Step 49 실차(16:30) 결과: 계산 계열 STOP은 크게 감소, 그러나 코너2 진입 정지가 랩의 67%에서 남아 랩 중앙값 11.5 s(빠른 랩 6.7~7.7 s). 원인 분석은 `STEP50_STEP49_RUN_ANALYSIS.md`. **코드 수정은 아직 안 함.**

## 2. 커밋 대응표 (Step 45~50)
| Step | JSND_INT | v3 | 내용 |
|---|---|---|---|
| 45 | `e16d3ef` | `e6e8e19`, `4959165` | heading_robust/history_prediction 옵션, 배포·rec/check 스크립트 |
| 46 | – | `bc7af3b` | improved(heading_robust) 직선 진동 진단 → `mpc` 모드만 사용 |
| 47 | `4ca035e` | `f23cc92` | r_rate 3.0, brake lead 0.12 s, steer preview 0.03 s, accel jerk 20 |
| 48 | `0f5e086` | `fbeeefd` | r_accel 0.05, q_heading 16, OSQP eps 1e-4, braking_recovery 확대 |
| 49 | `89e1011` | `1cd9736` | 지연 보정 0.18 s(history_prediction), 연속계획 10틱, odom −0.3 허용, trailing_without_target 소프트 STOP |
| 50 | – | 이 커밋 | Step 49 실차 분석(문서·로그·스크립트만) |

롤백: JSND_INT에서 `git revert <커밋>` 후 `colcon build --base-paths src --packages-select controller` (`~/JSND/install` source 후). 백업 `~/JSND_backups/step47_*`, `step48_*`, `step49_20261125_162411`.

## 3. 실행 (Jetson, 사용자 직접)
```bash
bash ~/JSND_MPC_v3/jsnd_int/run/step45_run.sh mpc 11_18_6 5p0     # foreground, 로그 ~/bench_logs/step45_*.log
bash ~/JSND_INT/run/step45_rec.sh mpc                             # 별도 창: tmux rec bag (다음 런은 반드시 기록)
bash ~/JSND_INT/run/step45_check.sh mpc                           # 읽기 전용 param/hz 확인
```
- 환경: ROS 2 Jazzy, `ROS_DOMAIN_ID=173`, cyclonedds. osqp 1.1.3은 `~/JSND_INT/pydeps`(MPC 모드 controller_manager만).
- 버튼(MPC 모드 safety_mode): RB 한 번=자율, LB 유지=수동(1.0 m/s), A=비상정지 래치. 기동 시 래치 → `/simple_mux/reset_estop` 후 RB.
- `improved` 모드 사용 금지(Step 46). 원본 `~/JSND`는 수정 금지. 스택 중복 실행 금지(VESC 2개 사건).
- MPC 파라미터는 동적 변경 불가 → yaml 수정 후 노드 재기동. install yaml은 symlink라 yaml만 바꾸면 재빌드 불필요.

## 4. 미해결 과제 (우선순위, 사용자 결정 대기)
1. 코너2 진입 정지: 고속 OVERTAKE→TRAILING(`NO_STATIC_PATH`) 전환. 지나친 정적 장애물이 트랙 래핑으로 앞쪽에 재계산되는지 bag으로 확인 필요.
2. s≈4.1 RIGHT 정적 장애물(매 랩 새 id): 실물인지 맵·스캔 불일치(헤어핀 s3.0~3.7 잔차 0.11~0.16 m)인지 `/scan`으로 확인.
3. TTC EStop 거의 매 랩(s≈12): 원본 `estop.py`(±70°/0.35 s) 소관, 무변경 상태.
4. 직선 가속 1.1 m/s²: `mpc_cmd_max_lead_mps` 0.5 상한 의심, 가속도 명령 기록 필요.
5. 15:47 기동 직후 `OSQPException:4`(NONCVX) 영구 STOP: 미재현.
6. 15:49 로그 `VOLTS TOO LOW` 248회: 배터리 확인.

## 5. 작업 규칙 (사용자 지시)
- 모터·조향·arm·drive 전환은 사용자 명시 승인 없이는 금지. 실차는 읽기 전용 진단 우선.
- 반복 실험·전체 테스트 최소화(관련 테스트 1~2개만). 보고는 짧게, 수치 위주.
- 테스트: `PYTHONPATH=src/racing_stack/controller:src/racing_stack/state_machine:pydeps` + `/opt/ros/jazzy`, `~/JSND_INT/tests_offline`. 일부 `test_mpc_integration`은 부하 의존 플래키, `test_stabilization` 2건 등 기존 실패 있음.
- v3 커밋: `git -c user.name=jsnd -c user.email=jsnd@localhost commit`. racing_stack 커밋은 `-c user.email=jsnd@localhost`. JSND_INT의 origin은 로컬 `~/JSND/src/racing_stack`이라 GitHub로는 이 저장소 사본으로만 전달.
