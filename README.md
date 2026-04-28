# Cpp_SLAM

C++로 구현하는 SLAM(Simultaneous Localization and Mapping) 학습 프로젝트입니다.  
PLY 포인트 클라우드를 기반으로 KD-Tree, ICP, Voxel Grid Filter를 단계적으로 구현합니다.

---

## 프로젝트 구조

```
Cpp_SLAM/
├── PointCloud.h        포인트 클라우드 데이터 구조체
├── PlyParser.h/cpp     PLY 파일 파서 (ASCII / Binary 모두 지원)
├── KDTree.h/cpp        KD-Tree 자료구조 (최근접 탐색)
├── VoxelGrid.h/cpp     Voxel Grid Filter (다운샘플링)
├── ICP.h/cpp           ICP 알고리즘 (두 포인트 클라우드 정렬)
├── main.cpp            테스트 진입점
└── PLY/                테스트용 PLY 파일 폴더
```

---

## 빌드 방법

### 요구사항
- macOS: Xcode Command Line Tools (`xcode-select --install`)
- Windows: Visual Studio 2019 이상
- g++ (C++17 이상)

### macOS / Linux

```bash
g++ -std=c++17 -g main.cpp PlyParser.cpp KDTree.cpp VoxelGrid.cpp ICP.cpp -o slam
```

### 실행

```bash
./slam <PLY파일경로>
```

### VS Code

`.vscode/tasks.json`이 포함되어 있어 `Terminal → Run Build Task`로 빌드할 수 있습니다.  
`launch.json`이 설정되어 있어 F5로 디버그 실행이 가능합니다. (CodeLLDB 확장 필요)

---

## 구현 내용

### 1. PLY 파서 (`PlyParser`)
- ASCII / Binary 포맷 자동 감지
- `element vertex`, `property float`, `property uchar` 파싱
- Windows `\r\n` 줄바꿈 처리

### 2. KD-Tree (`KDTree`)
- 3D 포인트 클라우드를 공간적으로 분할하는 이진 트리
- X → Y → Z 축을 순환하며 재귀적으로 분할
- 최근접 탐색(Nearest Neighbor Search) 구현
- 브루트포스 대비 약 845배 빠른 탐색 속도

```
브루트포스 : 3,381 μs
KD-Tree   :     4 μs  (246,654개 점 기준)
```

### 3. Voxel Grid Filter (`VoxelGrid`)
- 공간을 격자(복셀)로 나눠 각 격자당 점 하나(평균)만 남김
- 복셀 크기로 다운샘플링 비율 조절
- `unordered_map` 기반으로 빠른 분류

```
원본    : 246,654개
다운샘플링 후 : 17,417개 (92.9% 축소, 16ms)
```

### 4. ICP (`ICP`)
- Iterative Closest Point 알고리즘
- KD-Tree로 대응점 탐색 → SVD(야코비 반복법)로 회전행렬 계산 → 반복 수렴
- 최대 반복 횟수 / 수렴 허용 오차 설정 가능

```
다운샘플링 적용 후 ICP 소요시간 : 168ms (17,417개 점 기준)
```

---

## 테스트 결과

`points3D.ply` (246,654개 점) 기준

| 항목 | 결과 |
|------|------|
| PLY 로드 | Binary 포맷, 15 bytes stride |
| KD-Tree 구축 | 513ms |
| KD-Tree 탐색 | 4μs (브루트포스 3,310μs) |
| Voxel Grid (0.3m) | 246,654 → 17,417개, 16ms |
| ICP (1도 회전, 0.05m 이동) | 7회 반복, 오차 0.129, 168ms |

---

## SLAM 로드맵

```
✅ 1단계 : KD-Tree         최근접 탐색
✅ 2단계 : Voxel Grid      다운샘플링
✅ 3단계 : ICP             두 프레임 정렬
⬜ 4단계 : LiDAR Odometry  연속 프레임 이동 추적
⬜ 5단계 : SLAM            지도 생성 + 루프 감지
```
