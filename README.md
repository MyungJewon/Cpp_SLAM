# Cpp_SLAM

외부 라이브러리 없이 C++17로 구현한 LiDAR SLAM 시스템입니다.  
ROS1 bag 파일을 입력받아 실시간 3D 지도를 생성하고 Pangolin으로 시각화합니다.

---

## 파이프라인

```
ROS1 Bag
  ├── PointCloud2 / Livox CustomMsg ──→ BagParser
  │                                         └── VoxelGrid (0.3m 다운샘플링)
  │                                                 └── Odometry (Scan-to-Map)
  │                                                       ├── Point-to-Plane ICP
  │                                                       ├── IMUPreintegrator (Loose Coupling)
  │                                                       └── LoopCloser
  └── IMU ──→ IMUPreintegrator ──→ ICP 초기 회전 힌트

Odometry ──→ MapBuilder ──→ output/slam_map.ply
                          ──→ output/slam_trajectory.ply
                          ──→ output/slam_map_2d.ppm
           ──→ PangolinViewer (실시간 시각화, 메인 쓰레드)
```

---

## 모듈 구성

| 파일 | 역할 |
|------|------|
| `BagParser` | ROS1 bag 파싱. PointCloud2 / Livox CustomMsg / IMU 지원 |
| `VoxelGrid` | Voxel Grid Filter (다운샘플링) |
| `KDTree` | 3D KD-Tree. ICP 대응점 탐색 및 법선 추정에 사용. `nearestIdx()` 지원 |
| `ICP` | Point-to-Plane ICP. 법선은 k-NN(k=10) PCA로 추정, 6×6 선형 시스템으로 최적화 |
| `IMUPreintegrator` | IMU 적분 (Rodrigues 공식). ICP 초기 회전값 제공 (Loose Coupling) |
| `Odometry` | Scan-to-Map 포즈 추정. 슬라이딩 윈도우 로컬 맵 유지 |
| `LoopCloser` | 반경 탐색 + Point-to-Plane ICP 검증 기반 루프 클로저 |
| `MapBuilder` | 전역 점군 누적 및 PLY 저장 |
| `PangolinViewer` | 실시간 3D 시각화. SLAM은 백그라운드 쓰레드, Pangolin은 메인 쓰레드 |

---

## 알고리즘 상세

### Point-to-Plane ICP

Point-to-Point 대비 평면 방향 미끄러짐(tangential drift)을 억제합니다.

```
사전 처리 (루프 외부, 1회):
  dstNormals = estimateNormals(dst, k=10)
    └── 각 점의 k-NN → 공분산 PCA → 최소 고유벡터 = 법선

반복 (최대 20회):
  1. 대응점 탐색: KDTree.nearestIdx() → 법선 조회
  2. residual r_i = (p_i - q_i) · n_i
  3. Jacobian J_i = [(p_i × n_i), n_i]  (6차원)
  4. 정규방정식 A·x = b 구성 (6×6)
  5. 가우스 소거법으로 [ω, t] 풀기
  6. R = Rodrigues(ω), 누적 변환 갱신
  7. 오차 수렴 시 종료
```

### Scan-to-Map Odometry

매 프레임을 직전 1프레임이 아닌 누적 로컬 맵과 매칭합니다.

- **로컬 맵**: 월드 좌표로 변환된 최근 프레임 점군 슬라이딩 윈도우
- **크기 제한**: 최대 5,000점 (초과 시 Voxel Filter → 강제 절삭)
- **포즈 갱신**: ICP 결과 = 절대 변환(로컬→월드) → `position = icp.t`, `rotation = icp.R`
- **이상값 거부**: 프레임당 이동 거리 2.0m 초과 시 해당 프레임 무시
- **지면 모드**: z=0 고정, Yaw만 추적 (평지 주행용)

### IMU Loose Coupling

- IMU 샘플을 Rodrigues 공식으로 적분 → 프레임 간 회전 추정
- 현재 누적 회전에 IMU 회전을 합성해 ICP 초기값으로 사용
- 매 프레임 사용 후 자동 reset (누적 없음)

### Loop Closure

- 현재 위치 반경 **10m** 내 키프레임을 후보로 탐색
- 후보마다 Point-to-Plane ICP 검증 (오차 < 0.5 시 루프 확정)
- 루프 구간을 선형 보간으로 경로 보정

---

## 빌드

### 의존성

```bash
# Eigen, GLEW
brew install eigen glew

# Pangolin (소스 빌드 필요 — brew cask와 다른 라이브러리)
git clone --depth=1 https://github.com/stevenlovegrove/Pangolin.git
cmake -S Pangolin -B Pangolin/build \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_FFMPEG=OFF \
  -DGLEW_INCLUDE_DIR=/opt/homebrew/include \
  -DGLEW_LIBRARY=/opt/homebrew/lib/libGLEW.dylib
cmake --build Pangolin/build -j8
sudo cmake --install Pangolin/build
```

### 컴파일

```bash
cmake -S . -B build
cmake --build build -j8
```

---

## 실행

```bash
# 토픽 목록 확인
build/slam --info <bag파일>

# SLAM 실행
build/slam <bag파일> <포인트클라우드토픽> <IMU토픽>

# 예시 (Velodyne)
build/slam data.bag /velodyne_points /imu/data

# 예시 (Livox)
build/slam data.bag /livox/lidar /livox/imu
```

실행하면 Pangolin 뷰어가 열리며 실시간으로 점군과 경로가 표시됩니다.

| 조작 | 동작 |
|------|------|
| 마우스 드래그 | 3D 뷰 회전 |
| 스크롤 | 줌 인/아웃 |
| 좌측 패널 | 프레임, 위치 XYZ, ICP 오차, 맵 포인트 수 실시간 표시 |

---

## 출력

| 파일 | 내용 |
|------|------|
| `output/slam_map.ply` | 전역 점군 지도 (ASCII PLY) |
| `output/slam_trajectory.ply` | 경로 점군 (ASCII PLY) |
| `output/slam_map_2d.ppm` | 탑다운 2D 지도 이미지 (1024×1024) |

---

## 주요 파라미터 (`main.cpp`)

| 설정 | 기본값 | 설명 |
|------|--------|------|
| `setGroundMode(true)` | true | z=0 고정, Yaw만 추적 |
| `setLocalMapMaxPts(5000)` | 5000 | ICP 대상 로컬 맵 최대 점 수. 많을수록 정확하지만 느림 |
| `setMaxStepDist(2.0f)` | 2.0m | 프레임당 최대 허용 이동 거리. 초과 시 ICP 실패로 무시 |
| `LoopCloser(10.0f, 0.5f, 50)` | — | 탐색 반경 10m / ICP 임계값 0.5 / 최소 프레임 간격 50 |

---

## 구현 현황

### 완료
- [x] ROS1 bag 파싱 (PointCloud2, Livox CustomMsg, IMU)
- [x] Voxel Grid Filter
- [x] KD-Tree (nearestIdx 포함)
- [x] Point-to-Plane ICP (법선 추정 포함)
- [x] IMU Loose Coupling
- [x] Scan-to-Map Odometry (슬라이딩 윈도우)
- [x] 이상값 거부 (프레임당 최대 이동 거리)
- [x] Loop Closure (반경 탐색 + ICP 검증 + 선형 경로 보정)
- [x] Pangolin 실시간 뷰어 (macOS 메인 쓰레드 구조)
- [x] PLY / PPM 출력 (실행 경로 무관한 절대 경로)

### 향후 과제
- [ ] FPFH 기반 3D Feature 추출/매칭 (루프 클로저 강화)
- [ ] Tight Coupling IMU (상태벡터에 IMU bias 포함)
- [ ] Pose Graph Optimization (g2o / GTSAM)
- [ ] 정지 감지를 통한 노이즈 억제
- [ ] Voxel 맵 구조 (평면 모델 저장, FAST-LIVO2 방식)

---

## 참고

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) — 본 프로젝트의 알고리즘 참고 대상
- [KISS-ICP](https://github.com/PRBonn/kiss-icp) — ICP만으로 실시간 SLAM을 구현한 최소 구현체
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — IMU + 포즈 그래프 기반 SLAM
