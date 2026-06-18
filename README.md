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
  │                                                       └── IMUPreintegrator (회전 deskew)
  └── IMU ──→ IMUPreintegrator ──→ 회전 deskew / gravity 측정 보관

백엔드 (GLIM식 서브맵 루프 클로저):
  Odometry 포즈 ──→ PoseGraph (GTSAM iSAM2)
       └── 키프레임 ──→ SubmapBuilder (앵커 로컬 좌표계로 누적)
                          └── 완성된 Submap ──→ LoopCloser
                                  └── 근방 과거 Submap 검색 → SubmapRegistration
                                        (coarse-to-fine GICP, overlap 게이트)
                                          └── 통과 시 PoseGraph에 BetweenFactor 추가
                                                (신뢰도 가중 노이즈)

옵션:
  --imu    ImuOdometry 타이트커플링 실험 경로 활성화 (미완성, 크래시 가능)
  기본값   타이트커플링 OFF (IMU는 회전 deskew 전용)

PoseGraph 최적화 결과 ──→ MapBuilder ──→ output/slam_map.ply
                                       ──→ output/slam_trajectory.ply
                                       ──→ output/slam_map_2d.ppm
                                       ──→ output/loops.log (루프 후보/채택 기록)
                       ──→ PangolinViewer (실시간 시각화 + 루프 스냅, 메인 쓰레드)
```

---

## 모듈 구성

| 파일 | 역할 |
|------|------|
| `BagParser` | ROS1 bag 파싱. PointCloud2 / Livox CustomMsg / IMU 지원 (압축 chunk 미지원 — none만) |
| `VoxelGrid` | Voxel Grid Filter (다운샘플링) |
| `VoxelMap` | per-voxel mean/covariance/normal 관리, 고유값 상대 플로어 정규화 |
| `KDTree` | 3D KD-Tree. ICP 대응점 탐색에 사용 |
| `ICP` | Voxel-GICP. per-voxel Mahalanobis 가중치, point-to-plane fallback |
| `IMUPreintegrator` | IMU 적분 (Rodrigues 공식). 회전 deskew와 gravity 측정 저장에 사용 |
| `ImuOdometry` | GTSAM IMU preintegration 타이트커플링 실험 경로. **미완성**, `--imu`에서만 사용 |
| `Odometry` | Scan-to-Map 포즈 추정. Constant velocity 예측, sliding window 로컬 맵 |
| `PoseMath` | Pose3D 변환 수학 (compose/invert/relative). 실시간·오프라인 공용 |
| `Submap` | 점군 덩어리 + 앵커 pose + AABB. 키프레임 누적 결과 / 향후 PLY 1장 |
| `SubmapBuilder` | 키프레임을 앵커 로컬 좌표계로 누적해 Submap 생성 |
| `SubmapRegistration` | **재사용 정합 코어**. 두 Submap을 coarse-to-fine GICP로 정합 (실시간 루프 + 향후 PLY 병합 공용) |
| `LoopCloser` | GLIM식 서브맵 기반 연속 루프 클로저. 위치 후보 → AABB → 정합 → overlap 게이트 |
| `PoseGraph` | GTSAM iSAM2 pose graph. odometry + 신뢰도 가중 루프 BetweenFactor (attitude factor는 비활성) |
| `MapBuilder` | 전역 점군 누적, keyframe 저장, PLY 저장 |
| `PangolinViewer` | 실시간 3D 시각화 + 루프 클로저 스냅/연결선, 종료 후 창 유지 |

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

### 루프 클로저 (GLIM식 서브맵 기반)

디스크립터로 "같은 곳"을 찾는 대신, **현재 추정 위치 근방의 과거 서브맵을 자주 정합**해
드리프트가 커지기 전에 연속적으로 닫습니다. (GLIM의 overlap 기반 접근)

```
1. SubmapBuilder: 키프레임 5개당 Submap 생성
   - 각 키프레임 센서-로컬 점 → 앵커 노드 로컬 좌표계로 변환 누적 (0.15m 다운샘플)
   - 점군은 앵커 센서 기준 상대 기하 → front-end/graph 프레임 발산과 무관하게 정확

2. LoopCloser: 새 Submap마다
   - 후보: 현재 추정 위치 반경 12m 내 + 노드 간격 200 이상 과거 Submap
   - AABB 1차 필터 → SubmapRegistration
   - 채택 시 PoseGraph에 BetweenFactor(past앵커 → new앵커) 추가
     · 측정 상대 pose = inv(T_past)·T_new = registerSubmaps 결과 (역변환 없음)
     · 신뢰도(overlap×fitness) 가중 노이즈: sigma = base / confidence

3. SubmapRegistration (재사용 코어):
   - dst Submap으로 VoxelMap 구성 → coarse-to-fine GICP (1.0m → 0.5m)
   - 게이트: fitness ≥ 0.25, overlap ≥ 0.4, rmse ≤ 0.5
     · overlap이 진짜/가짜 루프의 주 판별자 (진짜 0.5+, 가짜 0.1-)
     · fitness는 voxel 6점 게이트 때문에 더 빡빡 → floor로만 사용
```

보정은 PoseGraph에만 누적하고 front-end(Odometry) 내부 상태는 건드리지 않습니다.
(위치만 끌어당기면 다음 스캔이 옛 프레임 로컬맵과 매칭돼 깨지는 teleportation 방지)
Pangolin 뷰어는 매 프레임 PoseGraph 추정 궤적을 그려 루프가 닫히는 순간 화면에서 스냅되며,
채택된 루프는 노란 연결선으로 표시됩니다. 모든 후보 시도는 `output/loops.log`에 기록됩니다.

> **재사용 의도:** `SubmapRegistration`은 향후 "여러 PLY를 겹침 기반으로 자동 병합"하는
> 오프라인 도구(MapMerger)에서 그대로 호출할 수 있도록 실시간 경로와 분리되어 있습니다.

### IMU / Gravity 상태

- **기본 실행은 IMU를 위치 추정에 사용하지 않습니다.** IMU 샘플은 `IMUPreintegrator`로
  들어가 **회전 deskew와 gravity 측정 보관에만** 쓰입니다. 실제 포즈는 순수 LiDAR GICP로 추정.
- `Odometry`/`ICP`의 gravity prior 훅(`_gravityScale = 0.0f`), `PoseGraph` attitude factor는 비활성.
  (freeway 데이터에서 가속도계 중력이 z 드리프트를 악화시킴이 확인됨)
- `--imu` 타이트커플링(`ImuOdometry`) 경로는 **미완성**입니다. IMU preintegration 공백 구간에서
  velocity/bias 노드가 underconstrained가 되어 GTSAM `IndeterminantLinearSystem` 크래시가 발생할 수 있습니다.
- 좁은 FOV Livox(예: hku_main_building)는 순수 LiDAR로 pitch/z 드리프트가 크며,
  이 격차를 메우려면 IMU 타이트커플링 완성이 필요합니다 (다음 작업).

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

# 루프 클로저 비활성화
build/slam data.bag /velodyne_points /imu/data --no-lc

# IMU 타이트커플링 실험 경로 활성화
build/slam data.bag /velodyne_points /imu/data --imu
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
| `output/slam_map_2d.ppm` | 탑다운 2D 지도 이미지 (루프 엣지 포함) |
| `output/loops.log` | 루프 클로저 후보/채택 기록 (앵커, dist, fitness, overlap, rmse) |

현재 `main.cpp`는 처리 종료 시 pose graph 결과로 `MapBuilder`를 재구성합니다.
재구성은 저장된 keyframe 데이터를 사용하므로, 저장 주기와 pose graph 보정 여부가 최종 맵 밀도에 영향을 줍니다.

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
| `useImuOdom` | false | 기본 IMU 타이트커플링 비활성. `--imu` 옵션에서 활성 |
| `_gravityScale` | 0.0 | front-end gravity prior 비활성 |
| SubmapBuilder 키프레임 | 5 | 서브맵당 키프레임 수 (작을수록 재방문 앵커 정렬↑) |
| SubmapBuilder 다운샘플 | 0.15m | 0.5m fine voxel당 6점 이상 확보용 |
| LoopCloser searchRadius | 12m | 루프 후보 검색 반경 (앵커 겹침 기준) |
| LoopCloser minNodeGap | 200 | 인접 서브맵 제외 |
| registration resolutions | {1.0, 0.5} | coarse-to-fine GICP voxel |
| registration minOverlap | 0.4 | 루프 채택 주 판별자 |
| registration minFitness | 0.25 | 루프 채택 floor |
| 루프 노이즈 base | 0.02rad / 0.05m | confidence로 나눠 가중 |

---

## 구현 현황

### 완료
- [x] ROS1 bag 파싱 (PointCloud2, Livox CustomMsg, IMU)
  - Livox CustomMsg 가변배열 길이 접두사 처리 (점군 평면붕괴 버그 수정)
- [x] Voxel Grid Filter
- [x] KD-Tree (k-NN 포함)
- [x] Voxel-GICP (per-voxel 누적 공분산, Mahalanobis weighting, 고유값 플로어 정규화)
- [x] VoxelMap (분포 누적, 고유값 상대 플로어 — 평면 하드게이트 제거)
- [x] Scan-to-Map Odometry (sliding window, constant velocity 예측)
- [x] Innovation gating + 적응형 Tikhonov damping (degenerate 방향 안정화)
- [x] 정지 감지 (포즈 고정, 맵 갱신 유지)
- [x] 성능 최적화 (k-NN 공분산 O(N log N))
- [x] Pangolin 실시간 뷰어 (루프 스냅 + 연결선, 종료 후 창 유지)
- [x] PLY / PPM / loops.log 출력
- [x] **순수 LiDAR front-end 안정화** (360° 라이다 전 구간 끊김 없이 추적)
- [x] **GLIM식 서브맵 루프 클로저** (PoseMath/Submap/SubmapBuilder/SubmapRegistration/LoopCloser)
  - coarse-to-fine GICP, overlap 주 판별, 신뢰도 가중 루프 노이즈, 연속 폐합
- [x] PoseGraph (GTSAM iSAM2) odometry + 루프 BetweenFactor 통합

### 진행 중 / 향후 과제
- [ ] **IMU 타이트커플링 완성** (`--imu` ImuOdometry 크래시 수정 + 튜닝) — 좁은 FOV/드리프트 핵심
  - 현재 IMU preintegration 공백 구간에서 velocity/bias underconstrained → GTSAM 크래시
- [ ] Z / pitch 드리프트 보정 (IMU 또는 신뢰 가능한 중력 정렬)
- [ ] BagParser 압축 chunk(lz4/bz2) 지원 (현재 none만)
- [ ] 오프라인 PLY 병합 도구 (MapMerger — SubmapRegistration 재사용)

> 참고: front-end gravity prior와 PoseGraph attitude factor는 구현 훅은 있으나 현재 기본 비활성입니다.
> IMU 타이트커플링은 `--imu` 옵션으로 분리돼 있으며 아직 미완성입니다.

---

## 참고

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
- [KISS-ICP](https://github.com/PRBonn/kiss-icp)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
