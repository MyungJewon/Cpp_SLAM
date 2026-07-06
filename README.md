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
  --imu    IMU 타이트커플링(LIO) 실험 경로 활성화 (안전 동작하나 맵 품질 낮음 — 봉인, 기본 비권장)
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
| `ICP` | Voxel-GICP. per-voxel Mahalanobis 가중치, point-to-plane fallback. **대응점 탐색·공분산 추정 멀티스레드**(청크 순서 병합 → 결과 결정적) |
| `IMUPreintegrator` | IMU 적분 (Rodrigues 공식). 회전 deskew와 gravity 측정 저장에 사용 |
| `ImuOdometry` | (레거시) 별도 그래프 IMU 융합. pose override 방식이 점군을 망가뜨려 폐기, 현재 미사용 |
| `Odometry` | Scan-to-Map 포즈 추정. Constant velocity 예측, **속도 적응**(로컬맵·대응 반경·예측유지) + 입력 range 필터. 가변 속도(도보~고속) 대응 |
| `PoseMath` | Pose3D 변환 수학 (compose/invert/relative). 실시간·오프라인 공용 |
| `Submap` | 점군 덩어리 + 앵커 pose + AABB. 키프레임 누적 결과 / 향후 PLY 1장 |
| `SubmapBuilder` | 키프레임을 앵커 로컬 좌표계로 누적해 Submap 생성 |
| `SubmapRegistration` | **재사용 정합 코어**. 두 Submap을 coarse-to-fine GICP로 정합 (실시간 루프 + 향후 PLY 병합 공용) |
| `LoopCloser` | GLIM식 서브맵 연속 루프 클로저. **2단 검색**: 위치 후보(1순위) → ScanContext 형태 폴백(2순위, 강화 게이트) → AABB → 정합 → overlap 게이트 |
| `PoseGraph` | GTSAM iSAM2 pose graph. odometry + 신뢰도 가중 루프 BetweenFactor. `--imu` 시 X/V/B 노드 + CombinedImuFactor 통합(LIO-SAM식, 실험적) |
| `ScanContext` | 20링×60섹터 디스크립터. 위치 무관 장소 인식 + yaw 추정(bestShift). 멀티세션 병합 후보 검색에 사용 |
| `Session` / `SessionIO` | 세션(서브맵 목록 + 메타) 표현 및 디스크 저장/로드 (서브맵별 PLY + poses.txt) |
| `SessionMerger` | **멀티세션 병합 코어**. 전역 정렬(yaw 스윕 GICP) → ScanContext 후보/클러스터링 → GTSAM 통합 그래프 → 병합 점군. (대칭 구조에서 자동 정렬 불안정 — 아래 한계 참조) |
| `MapBuilder` | 전역 점군 누적, keyframe 저장, PLY 저장. **증분 voxel 중복제거**(신규 점만 O(1) 삽입 — 주기적 전체 재필터 제거) |
| `PangolinViewer` | 실시간 3D 시각화 + 루프 클로저 스냅/연결선, 종료 후 창 유지 |

### 실행 파일

| 타깃 | 역할 |
|------|------|
| `slam` | 단일 bag SLAM (실시간 뷰어 포함). `--save-session`으로 병합용 세션 저장 |
| `mapmerge` | 저장된 세션 N개를 하나의 맵으로 병합 (GUI 불필요) |
| `test_merge` | 멀티세션 병합 단위 테스트 (SessionIO 왕복, 클러스터링) |

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
  0. Range 필터: 센서에서 max-range(기본 80m, 0=무제한) 초과 점 제거 — 먼 노이즈 방어
  1. Constant velocity 예측: predictedT = position + lastDeltaT
  2. 예측값으로 ICP 초기화
  3. GICP 실행 (sliding window 로컬 맵. 반경은 속도 적응: clamp(15+speed·80, 20, 55)m)
     - 대응 반경도 속도 적응: speed>0.3 시 speed·1.5 (최대 5m)로 확장
  4. Innovation gating: 예측 대비 편차 > max(2.0, predStep*3.0) AND fitness < 0.15 → 거부
  5. maxStepDist(3.5m) 초과 → 거부
  6. 정지 감지(0.03m 미만) → 포즈 고정, 맵만 업데이트
  7. 스킵(품질부족/게이팅) 시 예측을 리셋하지 않고 90% 감쇠 유지 → 빠른 구간 재정합 복구
  8. GICP 전멸 시 point-to-point 폴백(KDTree 전역 탐색, 속도 비례 반경) — 빠른 구간 복구 경로
  9. 수락된 프레임만 delta 갱신 및 맵 삽입
```

> **속도 적응 설계 (가변 속도 대응):** 로컬맵 반경·대응 반경·예측 유지가 모두 프레임
> 이동량에 연동됩니다. 느린 구간(도보)은 기존 동작 유지(회귀 없음), 빠른 구간(차량·고속도로)은
> 자동으로 넓혀 추적을 유지합니다. 실측: freeway 데이터에서 30m 왕복 실패 → 800m+ 전진 성공,
> 도보 데이터는 루프 30개·fitness 0.85 그대로 유지.
> 단 개방 고속도로의 전진 방향 degeneracy(자기유사 구조)는 근본적으로 어려워, 중간 헤맴 후
> 복구하는 수준입니다(완전한 highway odometry는 IMU/휠/GNSS 융합 필요).

### 루프 클로저 (GLIM식 서브맵 기반)

디스크립터로 "같은 곳"을 찾는 대신, **현재 추정 위치 근방의 과거 서브맵을 자주 정합**해
드리프트가 커지기 전에 연속적으로 닫습니다. (GLIM의 overlap 기반 접근)

```
1. SubmapBuilder: 키프레임 5개당 Submap 생성
   - 각 키프레임 센서-로컬 점 → 앵커 노드 로컬 좌표계로 변환 누적 (0.15m 다운샘플)
   - 점군은 앵커 센서 기준 상대 기하 → front-end/graph 프레임 발산과 무관하게 정확

2. LoopCloser: 새 Submap마다 — 2단 후보 검색
   [1순위: 위치 기반] 현재 추정 위치 반경 12m 내 + 노드 간격 200 이상 과거 Submap
   [2순위: ScanContext 폴백] 1순위가 루프 0건일 때만 —
     형태 디스크립터로 위치 무관 후보 검색 (드리프트 > 반경이어도 루프 복구)
     · scanContextDistance ≤ 0.5 인 top-K, bestShift로 yaw 초기추정
   - AABB 1차 필터 → SubmapRegistration
   - 채택 시 PoseGraph에 BetweenFactor(past앵커 → new앵커) 추가
     · 측정 상대 pose = inv(T_past)·T_new = registerSubmaps 결과 (역변환 없음)
     · 신뢰도(overlap×fitness) 가중 노이즈: sigma = base / confidence

3. SubmapRegistration (재사용 코어):
   - dst Submap으로 VoxelMap 구성 → coarse-to-fine GICP (1.0m → 0.5m)
   - 게이트(위치 경로): fitness ≥ 0.25, overlap ≥ 0.4, rmse ≤ 0.5
     · overlap이 진짜/가짜 루프의 주 판별자 (진짜 0.5+, 가짜 0.1-)
   - 게이트(SC 폴백 경로, 강화): fitness ≥ 0.35, overlap ≥ 0.55
     · 위치 사전정보 없이 들어오는 루프라 보수적으로. 실측 근거:
       진짜 재방문 overlap 0.6~0.93 vs perceptual alias 0.43~0.57 → 그 사이를 컷
     · 실전 검증: 16m 떨어진 alias(ov 0.43)가 기본 게이트는 통과했을 상황에서
       강화 게이트에 정확히 기각됨 (loops.log의 cand(SC) 기록)
```

보정은 PoseGraph에만 누적하고 front-end(Odometry) 내부 상태는 건드리지 않습니다.
(위치만 끌어당기면 다음 스캔이 옛 프레임 로컬맵과 매칭돼 깨지는 teleportation 방지)
Pangolin 뷰어는 매 프레임 PoseGraph 추정 궤적을 그려 루프가 닫히는 순간 화면에서 스냅되며,
채택된 루프는 노란 연결선으로 표시됩니다. 모든 후보 시도는 `output/loops.log`에 기록됩니다.

> **재사용 의도:** `SubmapRegistration`은 "여러 PLY를 겹침 기반으로 자동 병합"하는
> 오프라인 도구(`mapmerge`)에서 그대로 호출되도록 실시간 경로와 분리되어 있습니다.

### 멀티세션 맵 병합 (`mapmerge`)

여러 bag을 각각 SLAM해 세션으로 저장한 뒤, 같은 구역을 지나는 세션들을 ScanContext
장소 인식으로 정렬해 **하나의 일관된 맵**으로 병합합니다. 세션 간에는 좌표계 원점이
서로 다르므로 위치 기반 검색이 불가능 → ScanContext(형태 디스크립터)로 위치 무관 후보를
찾습니다.

```
[1단계] slam <bag> <topic> --save-session <dir>
   SLAM(세션 내 루프 포함) 후 서브맵을 저장:
     <dir>/submap_NNNN.ply  (anchor-local 점군)
     <dir>/poses.txt        (보정된 anchor pose)

[2단계] mapmerge <dir1> <dir2> [...] -o merged.ply
   A. 전역 정렬(coarseAlign): 세션 전체 점군을 yaw 스윕(10°) GICP로 통째 정합
      → 거친 T_SR(세션S 월드 → 기준 월드) 추정. (1순위)
   B. (전역 실패 시) 서브맵 ScanContext 후보 → GICP → 일관성 클러스터링. (2순위)
   C. 통합 그래프: 전 세션 anchor가 노드. 세션내 순차 + 전역정렬에 일치하는
      세션간 loop BetweenFactor를 GTSAM LM으로 최적화. loop가 0건이면 전역
      정렬값으로 강체 배치(prior 고정).
   D. 출력: 보정 pose로 점 변환·누적 → 세션별 voxel 다운샘플 + 아웃라이어 제거.
      --colored 시 세션별 색상 PLY(세션0=빨강, 1=파랑)도 저장(정렬 육안 검증용).
```

핵심 원리는 단일세션 루프 클로저와 동일하며(서브맵 점이 anchor-local이라 보정된
anchor pose로 재배치하면 됨), 차이는 **세션 간 초기 정렬**을 어떻게 잡느냐입니다.
정렬값만 정해지면 그 다음 정제는 루프 클로저와 같은 메커니즘으로 동작합니다.

#### 알려진 한계: 대칭 구조에서 자동 전역 정렬 불안정 ⚠️

ScanContext 자체는 견고합니다(실데이터에서 yaw 복원 오차 0°, 동일/상이 장소 분리 4배
마진, top-1 검색 8/8; 합성 검증에서 yaw 40°+이동을 ~2mm 오차로 복원). **그러나
실데이터(반복·대칭 건물) 두 세션 병합에서, 자동 전역 정렬이 180° 뒤집힌 가짜 골짜기에
빠지는 현상을 확인했습니다.**

- 원인: 건물이 180° 회전에 거의 대칭이라, "올바른 정렬"과 "뒤집힌 정렬"의 overlap이
  비슷(예: 0.46 vs 정답 0.63)해 자동 지표로 구분이 안 됨. yaw 스윕을 촘촘히 해도
  지표 자체가 둘을 못 가르므로 해결되지 않음.
- 이는 알고리즘 결함이 아니라 **전역 자동 정렬의 원리적 한계**이며, 같은 이유로 GLIM
  등 멀티세션 시스템은 **수동 초기 정렬 단계**를 둡니다.
- 참고: 동일 데이터에 대한 외부 검증(전체 점군 yaw 스윕 ICP)에서는 올바른 정렬
  (yaw≈-80°, t≈(5.6,16.7), overlap 0.63)이 존재함을 확인 — 즉 두 맵은 실제로 겹치며,
  문제는 "자동이 그 골짜기를 안정적으로 고르지 못함"입니다.

→ **다음 단계: 수동 초기 정렬 힌트**(`--init-yaw`, `--init-t`)를 추가해 GLIM식으로
   사용자가 대략의 방향만 주면 GICP가 정밀화하도록 할 예정. (향후 과제 참조)

### IMU / Gravity 상태

- **기본 실행은 IMU를 위치 추정에 사용하지 않습니다.** IMU 샘플은 `IMUPreintegrator`로
  들어가 **회전 deskew와 gravity 측정 보관에만** 쓰입니다. 실제 포즈는 순수 LiDAR GICP로 추정.
- `Odometry`/`ICP`의 gravity prior 훅(`_gravityScale = 0.0f`), `PoseGraph` attitude factor는 비활성.

#### `--imu` LIO 타이트커플링 (실험 종결 — 안전하게 켜지지만 순수 LiDAR가 우세)

`--imu`를 주면 IMU를 **메인 `PoseGraph`에 직접 통합**합니다 (LIO-SAM식 단일 그래프):
X/V/B 노드 + `CombinedImuFactor`가 GICP odometry(BetweenFactor) + 루프 factor와 같은
iSAM2 그래프에서 동시 최적화됩니다. 시작 정지 구간 가속도계로 nav 중력 방향을 설정하고,
IMU 노이즈는 FAST-LIVO2 avia 값(loose)을 사용합니다.

진행 경과 (시간순):
- 별도 그래프(`ImuOdometry`)의 pose override 방식 → 점군 붕괴로 **폐기**.
- 단일 그래프 전환 → 크래시 해소, 정합 정상화. 그러나 드리프트 잔존.
- **fitness 적응 odometry 노이즈** 도입: fitness 높으면 타이트(IMU 후퇴),
  낮으면/스킵 프레임이면 루즈(IMU 브릿지). + 정지 프레임 **ZUPT**(속도=0 prior).
  - 1차 시도(하한 σ×6.7 루즈)는 **중력 모델 오차가 z를 +9.3m 폭주**시키고 루프 전멸
    — 중력 오차는 world-frame 상수라 바이어스로 흡수 불가.
  - 하한을 σ×2로 강화 + **v_z 감쇠 prior**(σ 0.3m/s, 수평 비구속) → 폭주 해결.

최종 실측 (data_2, 동일 조건 비교):

| 지표 | `--imu` (최종) | IMU off |
|---|---|---|
| 궤적 z | 0~1.3m (평평) | 0~1.8m |
| 루프 | 35개+, fitness 최대 0.86 | 30개, 0.85 |
| **맵 z 두께(robust)** | **13.6m** | **6.1m** |

**결론: 폭주·크래시 없이 안전하게 동작하지만, pitch/roll 미세 흔들림으로 점군이
수직으로 퍼져 맵 품질이 순수 LiDAR의 절반입니다.** 이 격차를 넘으려면 중력을 상태로
추정(FAST-LIO식)하는 대공사가 필요하며, LiDAR가 강한 데이터에서는 보상이 없어
**실험 기능으로 봉인**합니다. 재개 조건: LiDAR가 실제로 열화되는 데이터
(빠른 모션, 좁은 FOV 복도 등)가 확보될 때.

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

# 입력 range 필터 / voxel 크기 조절
#   --max-range : 기본 80m. 0을 주면 무제한(센서 최대 거리 전부 사용) — 야외·장거리용
#   --voxel     : 기본 0.2m(실내). 야외·희박 점군은 0.4~0.5 권장
build/slam outdoor.bag /os_cloud_node/points /os_cloud_node/imu --max-range 0 --voxel 0.5

# 멀티세션 병합용 세션 저장
build/slam bag1.bag /livox/lidar /livox/imu --save-session sessions/s1
build/slam bag2.bag /livox/lidar /livox/imu --save-session sessions/s2

# 세션 병합 → 단일 맵
build/mapmerge sessions/s1 sessions/s2 -o output/merged.ply

# 병합 품질 평가 (로버스트 span + 레퍼런스 대비 RMSE)
python tools/eval_merge.py output/merged.ply reference.pcd
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

## 실행 결과

| SLAM 과정 영상 | 3D 맵핑 결과 (Pangolin 시각화 / 전역 맵) |
| :---: | :---: |
| <video src="https://github.com/user-attachments/assets/03f98008-530e-475b-a142-b6455bc9109b" width="400" controls></video> | <img src="https://github.com/user-attachments/assets/db2abb2f-7f5c-4d3e-8834-d347c3767aae" width="400" alt="3D SLAM 결과 맵"> |
| *테스트에 사용된 지하 주차장 데이터* | *Pangolin 뷰어 실시간 시각화 및 루프 클로저 연결선* |

---

## 주요 파라미터 (`main.cpp`)

| 설정 | 현재값 | 설명 |
|------|--------|------|
| `setGroundMode(false)` | false | 씬 무관 3D 추적 |
| `setMaxStepDist(3.5f)` | 3.5m | 프레임당 최대 허용 이동 거리 |
| `_stationaryStepM` | 0.03m | 정지 판정 이동 임계값 |
| `_voxelSize` | 0.2m | Voxel 크기 (0.5→0.2로 정합 정밀도·z 드리프트 대폭 개선) |
| slideWindow | 20m | 로컬맵 유지 반경 (정지/메인 경로 통일) |
| odometry 노이즈 | fitness 적응 | sigma = base/clamp(fitness/0.4, 0.5, 2.0). base 0.05rad/0.1m |
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
- [x] **멀티스레드 가속 + 실시간 처리** — GICP 대응점 탐색·공분산 추정 병렬화(std::thread),
      MapBuilder 증분 voxel 중복제거(전체 재필터 제거), 뷰어 맵복사 스로틀, -O3.
      → data_3(3021프레임) 234초 처리 = **~13fps, 녹화 시간보다 빠름**(실시간 초과)
- [x] Pangolin 실시간 뷰어 (루프 스냅 + 연결선, 종료 후 창 유지)
- [x] PLY / PPM / loops.log 출력
- [x] **순수 LiDAR front-end 안정화** (360° 라이다 전 구간 끊김 없이 추적)
- [x] **GLIM식 서브맵 루프 클로저** (PoseMath/Submap/SubmapBuilder/SubmapRegistration/LoopCloser)
  - coarse-to-fine GICP, overlap 주 판별, 신뢰도 가중 루프 노이즈, 연속 폐합
- [x] PoseGraph (GTSAM iSAM2) odometry + 루프 BetweenFactor 통합
- [~] **멀티세션 맵 병합** (`mapmerge`, Session/SessionIO/SessionMerger) — **파이프라인 완성, 자동 정렬은 한계**
  - 파이프라인: 전역 정렬(yaw 스윕 GICP) → ScanContext/클러스터링 → GTSAM 통합 그래프 → 병합
  - 합성 검증 통과: 자기병합 T_SR≈identity(mm 이하), yaw40°+이동 ~2mm 복원
  - **실데이터 한계: 대칭 건물에서 자동 전역 정렬이 180° 가짜 골짜기에 빠짐** (위 「알려진 한계」 참조)
  - 도구: `slam --save-session`, `mapmerge --colored`(육안검증), `tools/eval_merge.py`(RMSE)
- [~] LIO 단일 그래프 (`--imu`, X/V/B + CombinedImuFactor) — **실험 종결(봉인)**
  - fitness 적응 odometry 노이즈 + ZUPT + v_z 감쇠 prior로 z 폭주(+9.3m) 해결,
    루프 35개 회복 (궤적 z 0~1.3m 평평)
  - 그러나 pitch/roll 미세 흔들림으로 **맵 z 두께 13.6m vs 순수 LiDAR 6.1m** —
    순수 LiDAR 확정 우세. 재개 조건: LiDAR가 실제 열화되는 데이터 확보 시
- [x] fitness 적응 odometry 노이즈 + 스킵 프레임 IMU 브릿지 + ZUPT (PoseGraph/Odometry)
- [x] slideWindow 불일치 버그 수정 (메인 경로 50→20m 통일)
- [x] **가변 속도 대응 (도보~고속도로)** — 로컬맵·대응 반경 속도 적응, 스킵 시 예측 유지(감쇠),
      P2P 폴백 속도 비례 반경. freeway 800m+ 전진 성공, 도보 회귀 없음
- [x] **ScanContext 2차 루프 검색** — 위치 검색 실패 시 형태 디스크립터로 위치 무관 후보 복구.
      SC 경로엔 강화 게이트(ov≥0.55/fit≥0.35) — perceptual alias(실측 0.43~0.57)와
      진짜 재방문(0.6+)의 경계로 컷. 실전에서 16m alias 차단 확인, 도보 데이터 회귀 없음
- [x] **입력 range 필터** (`--max-range`, 기본 80m / 0=무제한) — 초장거리 노이즈 맵 오염 방지
- [x] **`--voxel` 플래그** — front-end voxel 실행 시 조절 (실내 0.2 / 야외 0.4~0.5)

### 진행 중 / 향후 과제
- [ ] **mapmerge 수동 초기 정렬 힌트** (`--init-yaw`, `--init-t`) — 대칭 구조 180° 모호성
      해결의 핵심. GLIM식으로 사용자가 대략 방향만 주면 GICP가 정밀화 (현재 최우선)
- [ ] mapmerge 개선: 3개+ 세션 체인 정렬(현재 추가 세션은 세션0에만 정렬)
- [ ] mapmerge용 CMake에서 Pangolin 의존 분리 (현재 configure 단계에서 Pangolin 요구)
- [ ] BagParser 압축 chunk(lz4/bz2) 지원 (현재 none만)
- [ ] (봉인 해제 시) LIO 중력 상태 추정(FAST-LIO식) — 현 데이터에선 보상 없음

> 참고: 기본 권장 구성은 **순수 LiDAR + 루프 클로저**(IMU off)입니다 — 360° 라이다에서 검증됨.
> `--imu`는 안전하게 동작하나(폭주/크래시 없음) 맵 품질이 낮아 기본값이 아닙니다.

---

## 참고

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
- [KISS-ICP](https://github.com/PRBonn/kiss-icp)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [GLIM](https://github.com/koide3/glim) — 서브맵 기반 연속 폐합/멀티세션 영감
- [Scan Context](https://github.com/irapkaist/scancontext) — 위치무관 장소 인식 디스크립터
