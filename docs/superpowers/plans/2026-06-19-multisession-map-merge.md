# 멀티세션 맵 병합 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 여러 bag을 각각 SLAM해 세션으로 저장하고, ScanContext 기반 정렬로 하나의 맵으로 병합하는 오프라인 파이프라인 구축.

**Architecture:** 2단계. (1) `slam --save-session <dir>`가 서브맵을 anchor-local PLY + poses.txt로 저장. (2) `mapmerge`가 세션들을 로드해 ScanContext 후보검색 → GICP → 일관성 클러스터링 → GTSAM 통합 그래프 최적화 → 병합 PLY. SessionMerger가 GTSAM 그래프를 직접 구성(PoseGraph는 멀티체인 토폴로지에 부적합하여 미사용).

**Tech Stack:** C++17, GTSAM, 기존 ScanContext/SubmapRegistration/PlyParser/PoseMath/VoxelGrid 재사용. 테스트는 assert 기반 독립 실행파일 + Python RMSE 하니스.

---

## File Structure

- `src/Session.h` (신규) — `Session` 구조체(서브맵 목록 + 메타).
- `src/SessionIO.h/.cpp` (신규) — 세션 dir 저장/로드.
- `src/SessionMerger.h/.cpp` (신규) — 후보검색·클러스터링·GTSAM 그래프·병합.
- `src/mapmerge_main.cpp` (신규) — 병합 CLI.
- `src/ScanContext.h/.cpp` (수정) — z-센터링 변형 오버로드 추가.
- `src/main.cpp` (수정) — `--save-session` 플래그 + 종료 시 SessionIO 호출.
- `src/SubmapBuilder.h` (확인) — 완성 서브맵 목록 접근자(없으면 추가).
- `tests/test_merge.cpp` (신규) — SessionIO 왕복 + 클러스터링 단위 테스트.
- `CMakeLists.txt` (수정) — mapmerge, test_merge 타깃 + 공유 소스.
- `tools/eval_merge.py` (신규) — split-merge 통합 검증(RMSE 하니스 정리본).

---

## Task 1: CMake 공유 라이브러리화 + 빈 타깃

**Files:**
- Modify: `CMakeLists.txt`

기존 `slam`의 소스 목록을 변수로 묶어 `mapmerge`/`test_merge`가 재사용하게 한다.
Pangolin 의존(PangolinViewer)은 slam 전용으로 분리해 mapmerge/test가 GUI 없이 빌드되게 한다.

- [ ] **Step 1: CMakeLists에 코어 소스 변수 도입**

```cmake
set(CORE_SOURCES
    src/PlyParser.cpp src/KDTree.cpp src/ICP.cpp src/PoseGraph.cpp
    src/ScanContext.cpp src/VoxelGrid.cpp src/SubmapRegistration.cpp
    src/SubmapBuilder.cpp src/LoopCloser.cpp src/MapBuilder.cpp
    src/IMUPreintegrator.cpp
    src/SessionIO.cpp src/SessionMerger.cpp
)
```

`slam` 타깃은 `${CORE_SOURCES}` + `src/main.cpp src/Odometry.cpp src/ImuOdometry.cpp src/BagParser.cpp src/PangolinViewer.cpp`로 구성하도록 수정.

- [ ] **Step 2: mapmerge / test_merge 타깃 추가**

```cmake
add_executable(mapmerge src/mapmerge_main.cpp ${CORE_SOURCES})
target_include_directories(mapmerge PRIVATE src)
target_compile_options(mapmerge PRIVATE -O2 -Wall)
target_link_libraries(mapmerge gtsam)

add_executable(test_merge tests/test_merge.cpp ${CORE_SOURCES})
target_include_directories(test_merge PRIVATE src)
target_link_libraries(test_merge gtsam)
enable_testing()
add_test(NAME merge_unit COMMAND test_merge)
```

- [ ] **Step 3: 빈 파일 스텁 생성 후 빌드 확인**

`src/SessionIO.{h,cpp}`, `src/SessionMerger.{h,cpp}`, `src/mapmerge_main.cpp`,
`tests/test_merge.cpp`(빈 main) 스텁 작성 → `cmake -S . -B build && cmake --build build`
→ 4개 타깃 모두 링크 성공(빈 동작).

- [ ] **Step 4: 커밋** (사용자가 직접 — 자동 커밋 금지)

---

## Task 2: Session 구조체 + SessionIO 왕복

**Files:**
- Create: `src/Session.h`, `src/SessionIO.h`, `src/SessionIO.cpp`
- Test: `tests/test_merge.cpp`

세션 = 서브맵 목록 + 메타. 저장: `submap_NNNN.ply`(anchor-local 점) + `poses.txt`
(행: `submapIndex anchorId R0..R8 tx ty tz`) + `meta.txt`(name, count, voxel).

- [ ] **Step 1: 실패 테스트 — SessionIO 왕복**

```cpp
// tests/test_merge.cpp
#include "Session.h"
#include "SessionIO.h"
#include <cassert>
#include <cstdio>
int main() {
    Session s; s.name = "t";
    Submap m; m.id=0; m.anchorId=5;
    m.refPose = { {1,0,0, 0,1,0, 0,0,1}, {1.0f,2.0f,3.0f} };
    m.points = {{0.1f,0.2f,0.3f},{1.0f,1.0f,1.0f}};
    s.submaps.push_back(m);
    saveSession("/tmp/_sess_test", s);
    Session r = loadSession("/tmp/_sess_test");
    assert(r.submaps.size()==1);
    assert(r.submaps[0].anchorId==5);
    assert(r.submaps[0].points.size()==2);
    float dx = r.submaps[0].refPose.t[0]-1.0f;
    assert(dx*dx < 1e-6f);
    float pz = r.submaps[0].points[0][2]-0.3f;
    assert(pz*pz < 1e-6f);
    printf("SessionIO roundtrip OK\n");
    return 0;
}
```

- [ ] **Step 2: 빌드 → 링크 실패(saveSession 미정의) 확인**

Run: `cmake --build build --target test_merge`
Expected: undefined reference to saveSession/loadSession.

- [ ] **Step 3: Session.h 작성**

```cpp
#pragma once
#include <string>
#include <vector>
#include "Submap.h"
struct Session {
    std::string name;
    float voxelSize = 0.15f;
    std::vector<Submap> submaps;  // points는 anchor-local, refPose는 local→world
};
```

- [ ] **Step 4: SessionIO.h 작성**

```cpp
#pragma once
#include <string>
#include "Session.h"
// dir에 submap_NNNN.ply + poses.txt + meta.txt 저장.
void saveSession(const std::string& dir, const Session& session);
// dir에서 세션 로드 (poses.txt 순서대로 submap_NNNN.ply 읽음).
Session loadSession(const std::string& dir);
```

- [ ] **Step 5: SessionIO.cpp 작성**

```cpp
#include "SessionIO.h"
#include "PlyParser.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace fs = std::filesystem;

static std::string submapFile(int i) {
    std::ostringstream o; o<<"submap_"<<std::setfill('0')<<std::setw(4)<<i<<".ply";
    return o.str();
}

void saveSession(const std::string& dir, const Session& session) {
    fs::create_directories(dir);
    std::ofstream poses(dir + "/poses.txt");
    for (size_t i = 0; i < session.submaps.size(); ++i) {
        const Submap& m = session.submaps[i];
        savePly(dir + "/" + submapFile((int)i), m.points);  // 기존 PlyParser API
        poses << i << " " << m.anchorId;
        for (int k=0;k<9;++k) poses << " " << m.refPose.R[k];
        poses << " " << m.refPose.t[0] << " " << m.refPose.t[1]
              << " " << m.refPose.t[2] << "\n";
    }
    std::ofstream meta(dir + "/meta.txt");
    meta << "name " << session.name << "\n"
         << "count " << session.submaps.size() << "\n"
         << "voxel " << session.voxelSize << "\n";
}

Session loadSession(const std::string& dir) {
    Session s;
    std::ifstream meta(dir + "/meta.txt");
    std::string key;
    while (meta >> key) {
        if (key=="name") meta >> s.name;
        else if (key=="voxel") meta >> s.voxelSize;
        else { std::string skip; std::getline(meta, skip); }
    }
    std::ifstream poses(dir + "/poses.txt");
    if (!poses) throw std::runtime_error("poses.txt 없음: " + dir);
    std::string line;
    while (std::getline(poses, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        int idx, anchorId; ss >> idx >> anchorId;
        Submap m; m.id = idx; m.anchorId = anchorId;
        for (int k=0;k<9;++k) ss >> m.refPose.R[k];
        ss >> m.refPose.t[0] >> m.refPose.t[1] >> m.refPose.t[2];
        m.points = loadPly(dir + "/" + submapFile(idx));  // 기존 PlyParser API
        // AABB 재계산 (로컬)
        if (!m.points.empty()) {
            std::array<float,6> b = {m.points[0][0],m.points[0][1],m.points[0][2],
                                     m.points[0][0],m.points[0][1],m.points[0][2]};
            for (auto&p:m.points){for(int d=0;d<3;++d){b[d]=std::min(b[d],p[d]);b[d+3]=std::max(b[d+3],p[d]);}}
            m.aabb = b;
        }
        s.submaps.push_back(std::move(m));
    }
    return s;
}
```

> 주: `savePly`/`loadPly` 실제 시그니처는 `src/PlyParser.h` 확인 후 맞춘다.
> 다르면 (예: 반환형/인자 순서) 이 호출부만 조정.

- [ ] **Step 6: 빌드 + 실행 → "SessionIO roundtrip OK"**

Run: `cmake --build build --target test_merge && ./build/test_merge`
Expected: `SessionIO roundtrip OK`

- [ ] **Step 7: 커밋** (사용자)

---

## Task 3: slam --save-session 연동

**Files:**
- Modify: `src/main.cpp`, `src/SubmapBuilder.h` (서브맵 접근자 확인/추가)

SLAM 종료 시(최종 finalPoses 보정 후) 완성된 서브맵들을 모아 Session으로 저장.
서브맵 점은 anchor-local, refPose는 finalPoses[anchorId]로 채운다.

- [ ] **Step 1: 완성 서브맵 출처 확인**

`LoopCloser`가 `_submaps`(전 서브맵, anchor-local)를 보유. 접근자가 없으면
`const std::vector<Submap>& submaps() const { return _submaps; }`를 LoopCloser에 추가.
(SubmapBuilder가 진행 중 서브맵을 들고 있으면 그쪽도 flush 고려.)

- [ ] **Step 2: main.cpp 플래그 파싱 추가**

`--save-session <dir>` 파싱 → `std::string sessionDir;` 설정 (Task에서 인자 1개 소비).

- [ ] **Step 3: 종료부에서 세션 구성·저장**

`bagMap.rebuildFromPoses(finalPoses)` 이후:

```cpp
if (!sessionDir.empty()) {
    Session sess; sess.name = "session";
    const auto& subs = bagLoopCloser.submaps();
    for (size_t i=0;i<subs.size();++i) {
        Submap m = subs[i];
        if (m.anchorId>=0 && m.anchorId<(int)finalPoses.size())
            m.refPose = finalPoses[m.anchorId];   // 보정된 anchor 포즈
        sess.submaps.push_back(std::move(m));
    }
    saveSession(sessionDir, sess);
    std::cout << "[SLAM] 세션 저장: " << sessionDir
              << " (" << sess.submaps.size() << " 서브맵)" << std::endl;
}
```

- [ ] **Step 4: 빌드 + 실데이터 1개로 세션 저장 → poses.txt/서브맵 PLY 생성 확인**

Run: `./build/slam <bag> <topic> --save-session /tmp/sess1`
Expected: `/tmp/sess1/poses.txt` 행 수 == 서브맵 수, `submap_0000.ply` 존재.

- [ ] **Step 5: 커밋** (사용자)

---

## Task 4: ScanContext z-센터링 + 후보 검색

**Files:**
- Modify: `src/ScanContext.h/.cpp`
- Create: `src/SessionMerger.h/.cpp` (후보 검색 부분)

세션 간 z datum 차 보정을 위해 점군 z를 중앙값으로 센터링 후 디스크립터 생성.

- [ ] **Step 1: ScanContext에 센터링 오버로드 추가**

```cpp
// ScanContext.h
ScanContextDesc computeScanContextCentered(const std::vector<std::array<float,3>>& points);
```

```cpp
// ScanContext.cpp
ScanContextDesc computeScanContextCentered(const std::vector<std::array<float,3>>& pts){
    if (pts.empty()) return computeScanContext(pts);
    std::vector<float> zs; zs.reserve(pts.size());
    for (auto&p:pts) zs.push_back(p[2]);
    std::nth_element(zs.begin(), zs.begin()+zs.size()/2, zs.end());
    float zmed = zs[zs.size()/2];
    std::vector<std::array<float,3>> c; c.reserve(pts.size());
    for (auto&p:pts) c.push_back({p[0],p[1],p[2]-zmed});
    return computeScanContext(c);
}
```

- [ ] **Step 2: SessionMerger.h 골격**

```cpp
#pragma once
#include <vector>
#include "Session.h"
#include "SubmapRegistration.h"
struct MergeParams {
    int   topK = 5;             // 서브맵당 ScanContext 후보 수
    float maxScDist = 0.5f;     // ScanContext 거리 상한(후보 컷)
    int   minClusterSize = 3;   // 세션 채택 최소 일관 매칭 수
    float clusterTransTol = 1.5f; // T_SR 군집 이동 허용오차(m)
    float clusterRotTol = 0.15f;  // 회전 허용오차(rad)
    RegistrationParams reg;     // GICP 게이트
    float outputVoxel = 0.1f;
};
struct InterMatch { int refIdx, srcIdx; Pose3D T_SR; float fitness, overlap; };
class SessionMerger {
public:
    explicit SessionMerger(MergeParams p={}) : _p(p) {}
    // 기준 세션(0)에 추가 세션 S를 정렬할 매칭들 반환(클러스터링 전 raw).
    std::vector<InterMatch> findMatches(const Session& ref, const Session& src) const;
private:
    MergeParams _p;
};
```

- [ ] **Step 3: findMatches 구현 (ScanContext 후보 → GICP → T_SR)**

```cpp
#include "SessionMerger.h"
#include "ScanContext.h"
#include "PoseMath.h"
#include <algorithm>
std::vector<InterMatch> SessionMerger::findMatches(const Session& ref,
                                                   const Session& src) const {
    std::vector<InterMatch> out;
    std::vector<ScanContextDesc> refDesc(ref.submaps.size());
    for (size_t i=0;i<ref.submaps.size();++i)
        refDesc[i]=computeScanContextCentered(ref.submaps[i].points);
    for (size_t j=0;j<src.submaps.size();++j) {
        ScanContextDesc d = computeScanContextCentered(src.submaps[j].points);
        std::vector<std::pair<float,int>> cand; // (scDist, refIdx)
        for (size_t i=0;i<ref.submaps.size();++i)
            cand.push_back({scanContextDistance(d, refDesc[i]).first,(int)i});
        std::sort(cand.begin(),cand.end());
        for (int t=0; t<_p.topK && t<(int)cand.size(); ++t) {
            if (cand[t].first > _p.maxScDist) break;
            int i = cand[t].second;
            int shift = scanContextDistance(d, refDesc[i]).second;
            float yaw = shift * (2.0f*(float)M_PI / SC_SECTORS);
            // 초기 추정: src 서브맵 로컬 → ref 서브맵 로컬 (yaw 회전만, t=0에서 시작)
            float c=std::cos(yaw), s=std::sin(yaw);
            Pose3D guess = {{c,-s,0, s,c,0, 0,0,1},{0,0,0}};
            RegistrationResult r = registerSubmaps(src.submaps[j], ref.submaps[i],
                                                   guess, _p.reg);
            if (!r.success) continue;
            // T_SR = T_anchorRef · rel · inv(T_anchorS)
            Pose3D T_SR = posemath::compose(ref.submaps[i].refPose,
                          posemath::compose(r.relativePose,
                          posemath::invert(src.submaps[j].refPose)));
            out.push_back({i,(int)j,T_SR,r.fitness,r.overlap});
        }
    }
    return out;
}
```

- [ ] **Step 4: 빌드 확인**

Run: `cmake --build build --target mapmerge`
Expected: 컴파일 성공.

- [ ] **Step 5: 커밋** (사용자)

---

## Task 5: 일관성 클러스터링

**Files:**
- Modify: `src/SessionMerger.h/.cpp`
- Test: `tests/test_merge.cpp`

raw 매칭들의 T_SR 중 서로 일치(이동+회전 허용오차)하는 최대 군집을 찾는다.

- [ ] **Step 1: 실패 테스트 — 다수 일치 + 이상치 1개**

```cpp
// tests/test_merge.cpp 에 함수 추가 후 main에서 호출
void test_cluster() {
    SessionMerger mg;
    std::vector<InterMatch> in;
    Pose3D good = {{1,0,0,0,1,0,0,0,1},{5,0,0}};
    Pose3D bad  = {{1,0,0,0,1,0,0,0,1},{50,0,0}};
    for(int i=0;i<4;++i) in.push_back({i,i,good,0.7f,0.7f});
    in.push_back({9,9,bad,0.6f,0.6f});
    Pose3D T; std::vector<InterMatch> inl;
    bool ok = mg.bestCluster(in, T, inl);
    assert(ok); assert(inl.size()==4);
    assert((T.t[0]-5.0f)*(T.t[0]-5.0f) < 1e-3f);
    printf("cluster OK\n");
}
```

- [ ] **Step 2: bestCluster 선언 추가 (SessionMerger.h)**

```cpp
// 최대 일관 군집 탐색. 성공 시 대표 T_SR(인라이어 평균 이동/첫 회전)와 인라이어 반환.
bool bestCluster(const std::vector<InterMatch>& matches,
                 Pose3D& outT, std::vector<InterMatch>& inliers) const;
```

- [ ] **Step 3: bestCluster 구현 (SessionMerger.cpp)**

```cpp
static float transDist(const Pose3D&a,const Pose3D&b){
    float dx=a.t[0]-b.t[0],dy=a.t[1]-b.t[1],dz=a.t[2]-b.t[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}
static float rotDist(const Pose3D&a,const Pose3D&b){
    // trace(Ra^T Rb) → 각도
    float tr=0; for(int i=0;i<3;++i)for(int k=0;k<3;++k) tr+=a.R[i*3+k]*b.R[i*3+k];
    float cosang=(tr-1.0f)*0.5f; cosang=std::max(-1.0f,std::min(1.0f,cosang));
    return std::acos(cosang);
}
bool SessionMerger::bestCluster(const std::vector<InterMatch>& m,
                                Pose3D& outT, std::vector<InterMatch>& inliers) const {
    int best=-1; std::vector<InterMatch> bestSet;
    for (size_t i=0;i<m.size();++i) {
        std::vector<InterMatch> set;
        for (size_t j=0;j<m.size();++j)
            if (transDist(m[i].T_SR,m[j].T_SR)<_p.clusterTransTol &&
                rotDist(m[i].T_SR,m[j].T_SR)<_p.clusterRotTol)
                set.push_back(m[j]);
        if ((int)set.size()>best){best=(int)set.size();bestSet=set;}
    }
    if (best < _p.minClusterSize) return false;
    // 대표 T: 인라이어 이동 평균 + 첫 회전(소회전이라 평균 생략)
    Pose3D T = bestSet[0].T_SR; float tx=0,ty=0,tz=0;
    for (auto&e:bestSet){tx+=e.T_SR.t[0];ty+=e.T_SR.t[1];tz+=e.T_SR.t[2];}
    T.t={tx/bestSet.size(),ty/bestSet.size(),tz/bestSet.size()};
    outT=T; inliers=bestSet; return true;
}
```

- [ ] **Step 4: 빌드 + 실행 → "cluster OK"**

Run: `cmake --build build --target test_merge && ./build/test_merge`
Expected: `SessionIO roundtrip OK` + `cluster OK`

- [ ] **Step 5: 커밋** (사용자)

---

## Task 6: 통합 GTSAM 그래프 최적화

**Files:**
- Modify: `src/SessionMerger.h/.cpp`

전 세션 anchor를 노드로 하는 GTSAM 그래프. 세션내 인접 anchor BetweenFactor +
세션간 인라이어 loop BetweenFactor. 세션0 첫 노드에 prior(게이지). 최적화 후
보정된 anchor 포즈 반환.

- [ ] **Step 1: merge() 선언 (SessionMerger.h)**

```cpp
// 세션들을 병합해 (세션idx,서브맵idx)->보정 world pose 맵과 병합 점군 생성.
struct MergeResult {
    bool success=false;
    std::vector<std::vector<Pose3D>> poses; // poses[sess][submap] = 보정 world pose
    std::vector<std::array<float,3>> cloud; // 병합·다운샘플·아웃라이어제거된 점군
};
MergeResult merge(const std::vector<Session>& sessions) const;
```

- [ ] **Step 2: merge() 구현 — 정렬 + 그래프 빌드 + 최적화**

GTSAM 직접 사용. 노드 키: `gtsam::Symbol('x', globalId)`. globalId = 세션별
오프셋 누적. 세션0 첫 노드 prior. 세션내 엣지: `inv(refPose[k])·refPose[k+1]`.
세션간 엣지: 인라이어의 `relativePose`를 ref/src anchor 글로벌 id 사이에 추가.
초기값: 세션0=저장 refPose, 세션S=`T_SR·refPose`. `LevenbergMarquardtOptimizer`로
최적화. (의사코드 — 구현 시 GTSAM 헤더 include: Pose3, NonlinearFactorGraph,
BetweenFactor, PriorFactor, Values, LevenbergMarquardtOptimizer, Symbol.)

```cpp
// 핵심 흐름
// 1) sessions[0] 기준. 각 S>=1: findMatches→bestCluster. 실패 시 경고 후 제외.
// 2) globalId 부여, 초기 Values 채움.
// 3) graph: prior(첫노드, 1e-6), 세션내 between(odomNoise 0.05/0.1),
//    세션간 between(loopNoise, confidence=overlap*fitness 가중).
// 4) LM 최적화 → result. poses[sess][k] = result.at<Pose3>(...).
// 5) 각 서브맵 점을 보정 pose로 변환·누적 → voxelGridFilter(outputVoxel)
//    → 통계적 아웃라이어 제거(반경 내 이웃 수 기준).
```

- [ ] **Step 3: 통계적 아웃라이어 제거 헬퍼**

```cpp
// KDTree 재사용: 각 점의 k번째 최근접 거리가 평균+3σ 초과면 제거.
static std::vector<std::array<float,3>> removeOutliers(
    const std::vector<std::array<float,3>>& pts, int k=8, float stdMul=3.0f);
```
(구현: KDTree로 각 점 k-NN 평균거리 → 전체 평균 μ, 표준편차 σ → > μ+stdMul·σ 제거.)

- [ ] **Step 4: 빌드 확인**

Run: `cmake --build build --target mapmerge`
Expected: 성공.

- [ ] **Step 5: 커밋** (사용자)

---

## Task 7: mapmerge CLI + split-merge 통합 검증

**Files:**
- Modify: `src/mapmerge_main.cpp`
- Create: `tools/eval_merge.py`

- [ ] **Step 1: mapmerge_main 구현**

```cpp
#include "SessionIO.h"
#include "SessionMerger.h"
#include "PlyParser.h"
#include <iostream>
int main(int argc, char** argv){
    std::vector<std::string> dirs; std::string out="merged.ply";
    for(int i=1;i<argc;++i){ std::string a=argv[i];
        if(a=="-o"&&i+1<argc) out=argv[++i]; else dirs.push_back(a); }
    if(dirs.size()<2){ std::cerr<<"usage: mapmerge <s1> <s2> [...] -o out.ply\n"; return 1; }
    std::vector<Session> sess; for(auto&d:dirs) sess.push_back(loadSession(d));
    SessionMerger mg; auto r = mg.merge(sess);
    if(!r.success){ std::cerr<<"병합 실패: 세션 간 일관 정렬 없음\n"; return 2; }
    savePly(out, r.cloud);
    std::cout<<"병합 완료: "<<out<<" ("<<r.cloud.size()<<" 점)\n"; return 0;
}
```

- [ ] **Step 2: eval_merge.py 작성 (RMSE 하니스 정리)**

`/tmp/align2.py`의 yaw-sweep ICP RMSE 로직을 `tools/eval_merge.py`로 옮기고
인자화: `python tools/eval_merge.py <merged.ply> <reference.ply/pcd>` → robust span +
정렬 후 RMSE/inlier% 출력.

- [ ] **Step 3: split-merge 통합 검증 (수동 실행, 사용자)**

같은 bag을 두 구간으로 나눠 두 세션 저장 후 병합, 단일세션 맵과 비교:
```bash
# (a) 단일 세션 전체
./build/slam <bag> <topic> --save-session /tmp/full
# (b) 두 세션으로 — bag 분할 도구가 없으면 동일 bag 2회로 자기정합 스모크 테스트
./build/slam <bag> <topic> --save-session /tmp/sA
./build/mapmerge /tmp/sA /tmp/sA -o /tmp/merged.ply   # 자기병합: T_SR≈identity 기대
python tools/eval_merge.py /tmp/merged.ply output/slam_map.ply
```
Expected: 자기병합 시 T_SR≈identity, 병합 맵이 원본과 RMSE 수 cm.

- [ ] **Step 4: 커밋** (사용자)

---

## Self-Review 메모

- 스펙 커버리지: 세션저장(T2,3) / ScanContext z센터링(T4) / 후보검색(T4) /
  클러스터링(T5) / 통합그래프(T6) / 출력·아웃라이어(T6) / CLI(T7) / 통합검증(T7) 전부 매핑됨.
- PoseGraph 재사용 → SessionMerger 직접 GTSAM으로 변경(멀티체인 토폴로지 사유). 스펙 갱신함.
- PlyParser 시그니처(savePly/loadPly)는 Task2 Step5 구현 시 실제 헤더로 확정.
- 클러스터 대표 회전은 소회전 가정으로 첫 인라이어 사용(평균 회전은 YAGNI).
```
