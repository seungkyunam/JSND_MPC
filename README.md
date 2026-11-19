> 속도·조향 통합 MPC의 현재 검증 상태와 실행 명령: [INTEGRATED_MPC_STATUS.md](INTEGRATED_MPC_STATUS.md). 실제 11_16_3 ROS 3랩 및 실차 검증은 미완료입니다.

# JSND_MPC

JSND 소스에서 분리한 LTV-MPC 시뮬레이션 개발 저장소입니다.
시뮬레이션은 속도·조향 통합 MPC만 실행합니다. PP와 조향 전용 MPC 소스는 복구용으로 보존합니다.

이 PC의 기존 `test0` 맵 실행:

```bash
cd /home/sk/JSND_MPC
SIM_ONLY_ISOLATED=YES bash run_test0_sim.sh
```

## 범위와 검증 상태

- 기존 ROS 패키지 소스·설정·지도·라이선스와 MPC 구현·테스트·배포 스크립트 포함
- MPC: `controller/controller/mpc/`
- 배포 및 실행 절차: [mpc_deployment/README.md](mpc_deployment/README.md)
- 시뮬레이션 기록과 제한: [stack_master/README_MPC_SIM.md](stack_master/README_MPC_SIM.md)
- Humble 빌드·제한된 시뮬레이션 관찰 완료. Jazzy 코드 대응 및 배포 회귀 테스트
  18개 통과. 실제 Jazzy 빌드·Python ABI·DDS·PP/MPC 실행은 미검증입니다.

이 저장소의 소스 복사만으로 전체 시뮬레이터 환경이 구성되지는 않습니다.
배포 스크립트는 동일한 ROS 배포판으로 구성된 별도 underlay를 요구합니다.
MPC 의존성은 `mpc_deployment/requirements.txt`를 기준으로 하며, 루트의
기존 Conda 설정과 requirements는 이전 환경 자료이므로 MPC 설치 절차와
혼용하지 마세요. 원래 운영 문서는 `docs/legacy/`에 보존했습니다.

실차는 별도 `mpc_vehicle` Shadow/Bench/Drive 안전 게이트 경로만 사용하며,
현장 검증과 승인 전에는 하드웨어 출력이 차단됩니다. 시뮬레이션은 실제 차량과
분리된 localhost ROS 도메인에서 실행합니다. 사용자 승인에 따라 기존 `test0` 맵을
사용하며 `11_16_3`와 실차 검증을 대신하지 않습니다.

## 소스 출처

원본: https://github.com/msms6921/JSND (기준 커밋은 SOURCE_PROVENANCE.md 참조).
현재 작업 트리의 미커밋 MPC 변경을 포함한 독립 스냅샷이며 원본 Git 이력은
포함하지 않습니다. 포함된 외부 프로젝트의 LICENSE/COPYING과 저작권 표기를
그대로 유지합니다. 저장소 전체에 새로운 단일 라이선스를 부여하지 않습니다.
