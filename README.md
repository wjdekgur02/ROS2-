# ROS2 + 2d lidar 
study ROS2
Low-cost pseudo-3D LiDAR mapping using YDLIDAR G4 and Z-axis linear scanning, with Ouster comparison
# Pseudo-3D LiDAR Mapping Project

이 레포지토리는 2D LiDAR(YDLIDAR G4)에 Z축 리니어 스캔을 결합하여 Pseudo-3D 점군을 생성하고, 이를 3D LiDAR(Ouster) 점군과 비교하기 위한 코드와 문서를 정리한 프로젝트이다.

본 프로젝트의 목적은 고정형 2D LiDAR가 가지는 높이(Z) 정보의 한계를 보완하고, 제한된 예산 환경에서 저비용으로 3차원 형상 정보를 확장할 수 있는 방법을 구현하고 검증하는 데 있다. 이를 위해 2D LiDAR 데이터를 Z축 방향으로 적층하여 Pseudo-3D 점군을 생성하고, 동일 환경에서 획득한 3D LiDAR(Ouster) 결과와 비교하였다.

## 주요 내용

- 2D LiDAR(YDLIDAR G4) 기반 Pseudo-3D 점군 생성
- Z축 리니어 스테이지와 스텝모터(TB6600, Arduino) 제어
- ROS2 Humble 환경에서 `/scan` 및 PointCloud2 데이터 수집
- Ouster 3D LiDAR 점군 저장 및 비교
- CloudCompare를 이용한 ROI, 정렬, 점군 비교
- 실측 치수와 점군 측정값 비교를 통한 정량 평가

## 개발 환경

- Jetson Nano
- Ubuntu / ROS2 Humble
- YDLIDAR G4
- Ouster 3D LiDAR
- Arduino + TB6600 + Linear Stage
- CloudCompare

## 폴더 설명

- `configs/`  
  CycloneDDS 등 실행 환경에 필요한 설정 파일

- `docs/`  
  ROS2와 G4 LiDAR 연결 방법, Ouster 연결 방법, 실험 과정, CloudCompare 비교 절차 등 문서 정리

- `lidar_tools/`  
  점군 저장 및 변환을 위한 Python 스크립트 모음  
  예: Ouster PointCloud2 → PLY 저장, Pseudo-3D 점군 저장

- `scripts/`  
  실험용 보조 스크립트 및 테스트 코드

- `ros2_ws/`  
  ROS2 워크스페이스 관련 코드

- `ydlidar_ws/`  
  YDLIDAR ROS2 드라이버 및 Z축 관련 패키지 코드

## 프로젝트 흐름

1. YDLIDAR G4에서 2D 스캔 데이터를 수집한다.
2. 리니어 스테이지를 이용해 LiDAR를 Z축 방향으로 이동시킨다.
3. 시간 또는 위치 기준으로 각 스캔에 Z좌표를 부여하여 3차원 점군을 누적 생성한다.
4. 생성된 Pseudo-3D 점군을 PLY/PCD 형식으로 저장한다.
5. 동일 환경에서 획득한 Ouster 3D LiDAR 점군과 비교한다.
6. CloudCompare를 이용하여 ROI 추출, 시각 비교, 실측 기반 치수 비교를 수행한다.

## 연구 목적

이 프로젝트는 3D LiDAR를 완전히 대체하기 위한 것이 아니라, 제한된 예산과 장비 조건에서 2D LiDAR를 활용해 3차원 정보를 확장할 수 있는 가능성을 확인하기 위한 것이다. 특히 정적 또는 준정적 환경에서 ROI 중심의 형상 파악, 구조 확인, 교육/연구용 프로토타이핑에 유용한 대안이 될 수 있다.

## 비고

본 레포지토리에는 실험 및 비교 과정에서 사용한 코드, 설정, 문서가 함께 포함되어 있으며, 일부 폴더는 외부 드라이버 또는 연동 패키지를 포함할 수 있다.
