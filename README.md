# JSND_MPC

JSND 소스에서 분리한 LTV-MPC 시뮬레이션 개발 저장소입니다.
기존 PP 제어기가 기본값이며 MPC는 `sim:=true`에서만 사용할 수 있습니다.

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

실차 MPC 사용은 금지되어 있습니다. 시뮬레이션도 구동계를 물리적으로
비활성화하고 실제 ROS 네트워크·브리지와 격리한 환경에서 실행해야 합니다.
`11_16_3` 지도는 제공되지 않습니다. 기존 launch의 PP 기본 설정이 실차용일 수
있으므로 시뮬레이션은 위 문서의 `run_mpc.sh` 절차를 따르세요.

## 소스 출처

원본: https://github.com/msms6921/JSND (기준 커밋은 SOURCE_PROVENANCE.md 참조).
현재 작업 트리의 미커밋 MPC 변경을 포함한 독립 스냅샷이며 원본 Git 이력은
포함하지 않습니다. 포함된 외부 프로젝트의 LICENSE/COPYING과 저작권 표기를
그대로 유지합니다. 저장소 전체에 새로운 단일 라이선스를 부여하지 않습니다.
