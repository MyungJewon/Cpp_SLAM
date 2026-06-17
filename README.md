# Cpp_SLAM

ROS 없이 C++17로 처음부터 구현한 LiDAR SLAM 시스템입니다.  
ROS1 bag 파일을 입력받아 3D 지도를 생성하고 Pangolin으로 시각화합니다.

---

## 파이프라인

```
ROS1 Bag
  ├── PointCloud2 / Livox CustomMsg ──→ BagParser
  │                                         └── VoxelGrid (다운샘플링)
  │                                                 └── Odometry (Scan-to-Map Voxel-GICP)
  │                                                       ├── Constant Velocity 예측
  │                                                       ├── Innovation Gating
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
| `VoxelMap` | per-voxel mean/covariance/normal 관리, PCA 기반 is_planar 판정 |
| `KDTree` | 3D KD-Tree. ICP 대응점 탐색에 사용 |
| `ICP` | Voxel-GICP. per-voxel Mahalanobis 가중치, point-to-plane fallback |
| `IMUPreintegrator` | IMU 적분 (Rodrigues 공식). ICP 초기 회전값 제공 |
| `Odometry` | Scan-to-Map 포즈 추정. Constant velocity 예측, sliding window 로컬 맵 |
| `LoopCloser` | 반경 탐색 + ICP 검증 기반 루프 클로저 |
| `PoseGraph` | 루프 클로저 결과 관리 |
| `MapBuilder` | 전역 점군 누적 및 PLY 저장 |
| `PangolinViewer` | 실시간 3D 시각화 |

---

## 알고리즘 상세

### Voxel-GICP (Generalized ICP)

Voxel 단위로 점군의 분포(mean, covariance)를 누적해 유지하고 Mahalanobis 거리 기반으로
정합합니다. 평면 여부를 하드 게이트로 거르지 않고, **공분산이 기하(평면/선/구)를 인코딩**해
가중을 결정하는 본래 VGICP 방식입니다.

```
VoxelMap 구성:
  각 voxel → count, mean, covariance (누적 통계로 분포 추정), normal (최소 고유벡터)
  공분산 정규화: 고유값 상대 플로어 — λ_i ← max(λ_i, λ_max · 1e-3)
                 → 이방성(disc 모양) 보존 + 특이행렬 방지 (조건수 ≤ 1000)

반복 (최대 20회):
  1. 현재 pose로 source 점 변환
  2. voxel 조회로 대응점 탐색 (point_count >= 6 인 voxel만 — 공분산 신뢰 확보)
  3. W = (C_src + C_dst)^{-1} 계산  (양쪽 모두 고유값 플로어 적용)
  4. J^T W J · Δx = -J^T W r 정규방정식 풀기
  5. 회전/이동 블록별 적응형 Tikhonov damping (degenerate 방향 안정화)
  6. 변환 누적 후 수렴 판정:
     delta_t norm < 1e-4 AND Frobenius(I-deltaR) < 1e-4 AND error 개선 < 1e-4
  7. GICP fitness=0 시 point-to-point ICP로 fallback

KDTree k-NN 으로 source 점별 공분산을 O(N log N)에 추정 (brute-force O(N²) 제거).
```

### Scan-to-Map Odometry

```
매 프레임:
  1. Constant velocity 예측: predictedT = position + lastDeltaT
  2. 예측값으로 ICP 초기화
  3. GICP 실행 (sliding window 로컬 맵 대상, 반경 50m)
  4. Innovation gating: 예측 대비 편차 > max(2.0, predStep*3.0) AND fitness < 0.15 → 거부
  5. maxStepDist(3.5m) 초과 → 거부
  6. 정지 감지(0.03m 미만) → 포즈 고정, 맵만 업데이트
  7. 수락된 프레임만 delta 갱신 및 맵 삽입
```

---

## 빌드

### 의존성

```bash
# Eigen, GLEW
brew install eigen glew

# Pangolin (소스 빌드)
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
cmake --build build --parallel 4
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
| `output/slam_map_2d.ppm` | 탑다운 2D 지도 이미지 |

---

## 주요 파라미터 (`main.cpp`)

| 설정 | 현재값 | 설명 |
|------|--------|------|
| `setGroundMode(false)` | false | 씬 무관 3D 추적 |
| `setMaxStepDist(3.5f)` | 3.5m | 프레임당 최대 허용 이동 거리 |
| `_stationaryStepM` | 0.03m | 정지 판정 이동 임계값 |
| `_voxelSize` | 0.5m | Voxel 크기 |
| `setEigenFloor` | 1e-3 | 공분산 고유값 상대 플로어 |
| `kMinCovPoints` (ICP) | 6 | 대응에 쓸 voxel 최소 점수 |

---

## 구현 현황

### 완료
- [x] ROS1 bag 파싱 (PointCloud2, Livox CustomMsg, IMU)
- [x] Voxel Grid Filter
- [x] KD-Tree (k-NN 포함)
- [x] Voxel-GICP (per-voxel 누적 공분산, Mahalanobis weighting, 고유값 플로어 정규화)
- [x] VoxelMap (분포 누적, 고유값 상대 플로어 — 평면 하드게이트 제거)
- [x] Scan-to-Map Odometry (sliding window, constant velocity 예측)
- [x] Innovation gating + 적응형 Tikhonov damping (degenerate 방향 안정화)
- [x] 정지 감지 (포즈 고정, 맵 갱신 유지)
- [x] 성능 최적화 (k-NN 공분산 O(N log N))
- [x] Pangolin 실시간 뷰어
- [x] PLY / PPM 출력
- [x] **순수 LiDAR front-end 안정화** (freeway 전 구간 끊김 없이 추적)

### 진행 중 / 향후 과제 (back-end 통합)
- [ ] Loop Closure 연결 정비 (포즈그래프 경유로 통일)
- [ ] PoseGraph 최적화 (iSAM2 증분화)
- [ ] IMU preintegration factor (back-end tight coupling)
- [ ] Z 잔여 드리프트 보정 (back-end로)

> 참고: front-end 중력 prior(IMU 가속도)는 주행 중 가속도계 편향으로 불안정해 제외.
> IMU는 back-end preintegration factor로만 사용 예정.

---

## 참고

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
- [KISS-ICP](https://github.com/PRBonn/kiss-icp)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
