# Cpp_SLAM — LiDAR SLAM 학습 프로젝트

ROS bag 파일에서 LiDAR 포인트 클라우드를 읽어 3D 맵을 생성하는 **학습용 SLAM 파이프라인**입니다.  
외부 라이브러리 없이 C++17만으로 SLAM의 핵심 알고리즘을 직접 구현했습니다.

> ⚠️ 이 프로젝트는 알고리즘 학습이 목적입니다.  
> 장거리·실시간 매핑에는 LOAM, LeGO-LOAM, LIO-SAM 같은 프로덕션 프레임워크를 사용하세요.

---

## 빌드 및 실행

```bash
g++ -std=c++17 -g main.cpp PlyParser.cpp KDTree.cpp ICP.cpp VoxelGrid.cpp \
    Odometry.cpp LoopCloser.cpp MapBuilder.cpp BagParser.cpp -o slam

./slam <ply파일경로>
# 예: ./slam PLY/points3D.ply
```

실행하면 아래 파일이 생성됩니다.

| 파일 | 설명 |
|---|---|
| `slam_map.ply` | 3D 포인트 클라우드 맵 |
| `slam_trajectory.ply` | 센서 이동 경로 (초록=시작, 빨강=끝) |
| `slam_map_2d.ppm` | 탑다운 2D 이미지 (Preview.app으로 열기) |

---

## 시스템 플로우

### 전체 데이터 흐름

```
[data.bag]
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│  BagParser                                                  │
│                                                             │
│  open()                                                     │
│   └─ 파일 열기 + "#ROSBAG V2.0" 헤더 검증                    │
│                                                             │
│  nextFrame()                                                │
│   ├─ readRecord()          파일에서 레코드 1개 읽기           │
│   │   ├─ op=0x03 (BagHeader)   → 무시                       │
│   │   ├─ op=0x07 (Connection)  → 토픽명 확인, connId 저장    │
│   │   └─ op=0x05 (Chunk)       → _chunkBuf 에 저장          │
│   │                                                         │
│   └─ processChunkStep()    CHUNK 내부 서브레코드 순회         │
│       ├─ op=0x07 (Connection)  → connId 설정                │
│       └─ op=0x02 (MessageData) → parsePointCloud2()         │
│           └─ ROS1 직렬화 파싱                                │
│               ├─ Header (seq, stamp, frame_id) 읽기·스킵     │
│               ├─ height × width → 포인트 수                  │
│               ├─ fields 배열에서 x/y/z 오프셋 찾기           │
│               └─ pointStep 간격으로 x,y,z float 추출         │
│                                                             │
│  반환: vector<array<float,3>>  rawPoints (~30,000개/프레임)  │
└─────────────────────────────────────────────────────────────┘
    │ rawPoints
    ▼
┌─────────────────────────────────────────────────────────────┐
│  Odometry::addFrame(rawPoints)                              │
│                                                             │
│  1. voxelGridFilter(rawPoints, 0.3m)                        │
│      └─ 30,000개 → ~2,000개로 축소                           │
│          ├─ floor(x/0.3), floor(y/0.3), floor(z/0.3)        │
│          │   으로 VoxelKey 계산                              │
│          └─ 같은 복셀의 점들을 평균내서 대표점 1개로 합침     │
│                                                             │
│  2. runICP(currentFrame, prevFrame)                         │
│      │                                                      │
│      │  KDTree::build(prevFrame)                            │
│      │   └─ X→Y→Z 축 순환, 중앙값 기준으로 분할              │
│      │       → 이진트리 구조로 저장                           │
│      │                                                      │
│      │  for 최대 50회 반복:                                  │
│      │   ├─ 대응점 탐색: 현재 프레임 각 점마다               │
│      │   │   KDTree::nearest() 호출                         │
│      │   │    └─ 루트부터 내려가며 가까운 쪽 먼저 탐색        │
│      │   │        반대쪽은 축 거리 < bestDist 일 때만 탐색    │
│      │   │                                                  │
│      │   ├─ 중심점 계산: src 평균, dst 평균                  │
│      │   ├─ SVD (Jacobi 반복법): 공분산 행렬 분해            │
│      │   │    → 최적 회전행렬 R 추출                         │
│      │   ├─ t = dst_mean - R × src_mean                     │
│      │   ├─ 누적 R, t 갱신                                   │
│      │   └─ 오차 변화량 < 1e-6 이면 조기 종료               │
│      │                                                      │
│      └─ 반환: ICPResult { R(3×3), t(3), iterations, error } │
│                                                             │
│  3. 누적 위치 갱신                                           │
│      _position -= _rotation × icp.t   ← 부호 반전(src→dst)  │
│      _rotation  = icp.R × _rotation   ← 회전 누적           │
│                                                             │
│  4. 지면 구속 (setGroundMode=true 시)                        │
│      _position[2] = 0                 ← z 고정              │
│      yaw = atan2(R[1][0], R[0][0])                          │
│      _rotation = yaw만 남긴 행렬      ← Roll/Pitch 제거      │
│                                                             │
│  5. _trajectory 에 현재 _position 추가                       │
│                                                             │
│  반환: position(3), rotation(3×3)                           │
└─────────────────────────────────────────────────────────────┘
    │ position, rotation
    │ rawPoints (다시 voxelGridFilter → ds)
    ▼
┌─────────────────────────────────────────────────────────────┐
│  MapBuilder::addFrame(ds, rotation, position)               │
│                                                             │
│  for each point p in ds:                                    │
│      global = rotation × p + position                       │
│       └─ 로컬(센서 기준) 좌표 → 전역(월드) 좌표 변환         │
│                                                             │
│  _map 에 global points 추가                                  │
│                                                             │
│  10프레임마다:                                               │
│      _map = voxelGridFilter(_map, 0.3m)                     │
│       └─ 중복 점 제거 → 메모리 관리                          │
└─────────────────────────────────────────────────────────────┘
    │ (5프레임마다 병렬 실행)
    ▼
┌─────────────────────────────────────────────────────────────┐
│  LoopCloser::detect(currentPos, ds, trajectory)             │
│                                                             │
│  1. 후보 탐색                                                │
│      for each kf in _keyFrames:                             │
│          dist = ||currentPos - kf.position||                │
│          dist < searchRadius(3m)                            │
│          AND keyframe index 차이 > minFrameGap(50)          │
│          → 후보 목록에 추가                                  │
│                                                             │
│  2. 후보마다 ICP 검증                                        │
│      icp = runICP(currentPoints, kf.points)                 │
│      icp.error < threshold(0.15) → 루프 감지                 │
│                                                             │
│  3. 루프 감지 시 경로 선형 보간 보정                          │
│      drift = currentPos - kf.position                       │
│      for i in [loopStart .. loopEnd]:                       │
│          trajectory[i] -= drift × (i-start)/(end-start)    │
│      → Odometry::setTrajectory(), setPosition() 로 반영     │
│                                                             │
│  루프 없으면: addKeyFrame(id, pos, ds) 로 현재 위치 저장     │
│                                                             │
│  ※ 한계: 맵 포인트는 소급 보정 안 됨 (경로만 보정)           │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│  출력                                                        │
│                                                             │
│  MapBuilder::saveToPly("slam_map.ply")                      │
│   └─ ASCII PLY 포맷, 전역 좌표 포인트 저장                   │
│                                                             │
│  saveTrajectoryPly("slam_trajectory.ply")                   │
│   └─ 경로 점들을 초록→빨강 그라디언트 색상으로 PLY 저장      │
│                                                             │
│  saveMapImage("slam_map_2d.ppm")                            │
│   ├─ 바운딩 박스 계산 (경로 + 맵 점)                         │
│   ├─ 좌표 → 픽셀 변환 (y축 반전)                            │
│   ├─ 맵 점: 청회색 / 경로: 초록→빨강                        │
│   └─ PPM P6 바이너리 저장 (라이브러리 없음)                  │
└─────────────────────────────────────────────────────────────┘
```

---

## 모듈별 핵심 요약

### KDTree
3D 공간을 이진트리로 분할해 최근접 이웃 탐색 속도를 높입니다.

- **build()**: X→Y→Z 축 순환, 점들을 중앙값 기준으로 분할
- **nearest()**: 가까운 쪽 우선 탐색, 반대쪽은 `축 거리 < bestDist`일 때만 탐색
- **성능**: 브루트포스 대비 약 800배 빠름 (246,654개 기준)

### VoxelGrid
3D 공간을 격자(복셀)로 나누고 같은 격자 안의 점들을 평균으로 합칩니다.

- `VoxelKey = { floor(x/size), floor(y/size), floor(z/size) }`
- `unordered_map`으로 O(N) 처리
- 30,000개 → ~2,000개 (약 93% 축소)

### ICP (Iterative Closest Point)
두 포인트 클라우드가 최대한 겹치도록 최적의 회전(R)과 이동(t)을 찾습니다.

```
반복:
  1. KDTree 로 각 점의 대응점(가장 가까운 점) 탐색
  2. src, dst 각각 중심점 계산
  3. 공분산 행렬 H = (src-mean)^T × (dst-mean)
  4. H 를 SVD(Jacobi 반복법)로 분해 → 최적 R 추출
  5. t = dst_mean - R × src_mean
  6. 오차가 수렴하면 종료
```

### Odometry
ICP 결과를 프레임마다 누적해 센서의 절대 위치를 추적합니다.

- ICP 는 `src → dst` 방향 변환 반환 → 실제 이동은 **부호 반전**
- `position -= R × t` / `rotation = R_new × R_old`
- 지면 구속 모드: z=0 고정, Yaw 만 유지

### LoopCloser
과거에 방문한 위치 근처로 돌아올 때 누적 오차를 보정합니다.

- 위치 거리 기반 후보 탐색 → ICP 로 매칭 검증 → 선형 보간으로 drift 제거
- **구조적 한계**: 경로(trajectory)만 보정, 맵 포인트는 소급 보정 불가
- 실제 시스템은 포즈 그래프 최적화(g2o, GTSAM)로 전체를 동시에 보정

### MapBuilder
각 프레임의 로컬 포인트를 전역 좌표로 변환해 누적 맵을 만듭니다.

- `global = R × local + position`
- 주기적 VoxelGrid 정리로 메모리 관리

### BagParser
ROS1 bag 바이너리 포맷을 파싱해 프레임별 PointCloud2 메시지를 추출합니다.

```
파일 구조:
  #ROSBAG V2.0\n
  BagHeader  (op=0x03)
  Chunk      (op=0x05)
    ├─ Connection  (op=0x07) ← 토픽명, connId
    └─ MessageData (op=0x02) ← PointCloud2 직렬화 데이터
  ...반복
```

---

## 왜 실제 SLAM 과 다른가

이 프로젝트를 만들면서 실제 SLAM 시스템이 왜 복잡한지 이해할 수 있습니다.

| 문제 | 이 구현의 한계 | 실제 시스템의 해법 |
|---|---|---|
| ICP 속도 | 전체 점 다운샘플 후 매칭 | 엣지/평면 **피처만 추출** (~200개)로 매칭 |
| 드리프트 | 프레임마다 오차 누적 | **IMU 사전 적분**으로 초기값 보정 |
| 루프 클로저 탐지 속도 | ICP 로 직접 비교 | **Scan Context** 디스크립터 (~0.1ms) |
| 루프 클로저 보정 범위 | 경로만 선형 보간 | **포즈 그래프 최적화**로 전체 맵 동시 보정 |

---

## 이후 학습 경로

1. **KISS-ICP** — ICP 만으로 실시간 SLAM 이 가능함을 증명한 최소 구현 (논문 + 오픈소스)
2. **LeGO-LOAM** — 지면 분리 + 피처 기반 경량 SLAM
3. **LIO-SAM** — IMU 결합 + 포즈 그래프 최적화 (GTSAM)
