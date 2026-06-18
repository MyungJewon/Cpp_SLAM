#pragma once
// LoopCloser (재설계): 서브맵 기반 연속 루프 클로저.
// GLIM식 — 디스크립터로 "같은 곳"을 찾는 대신, 현재 추정 기준으로 가까운 과거
// 서브맵을 찾아 GICP 정합하고, overlap/fitness 게이트를 통과한 것만 루프로 채택한다.
// 드리프트가 커지기 전에 자주 닫으므로 위치 기반 후보 탐색도 유효하게 동작한다.
#include <array>
#include <functional>
#include <ostream>
#include <vector>
#include "Submap.h"
#include "SubmapRegistration.h"

// 검출된 루프 한 건 (PoseGraph에 넣을 BetweenFactor 측정값)
struct LoopResult {
    int    fromId;        // dst 서브맵 앵커 노드 id
    int    toId;          // src 서브맵 앵커 노드 id
    Pose3D relativePose;  // inv(T_from) * T_to. 즉 src→dst 로컬 변환
    float  fitness;
    float  overlap;
};

class LoopCloser {
public:
    LoopCloser() = default;

    void setSearchRadius(float r)  { _searchRadius = r; }
    void setMinNodeGap(int g)      { _minNodeGap = g; }
    void setMaxCandidates(int n)   { _maxCandidates = n; }
    void setRegistrationParams(const RegistrationParams& p) { _params = p; }

    // 과거 서브맵 앵커의 "현재" world pose 조회 콜백 (poseGraph.getPose)
    using PoseLookup = std::function<Pose3D(int /*anchorId*/)>;

    // 새 서브맵을 등록하고 과거 서브맵들과 정합을 시도한다.
    //   currentRefPose : poseGraph가 추정한 newSubmap.anchorId의 "현재" world pose
    //   lookup         : 과거 서브맵 앵커의 현재 world pose 조회
    // 반환: 채택된 루프들 (여러 개일 수 있음).
    // log: nullptr이 아니면 모든 후보 시도(앵커, 거리, fitness/overlap/rmse, 채택여부)를 기록.
    std::vector<LoopResult> add(const Submap& newSubmap,
                                const Pose3D& currentRefPose,
                                const PoseLookup& lookup,
                                std::ostream* log = nullptr);

private:
    std::vector<Submap> _submaps;       // 등록된 모든 서브맵 (로컬 점군 보관)
    float _searchRadius = 20.0f;        // 후보 검색 반경 (현재 추정 위치 기준)
    int   _minNodeGap   = 30;           // 앵커 노드 id 차이 최소 (인접 서브맵 제외)
    int   _maxCandidates = 5;
    RegistrationParams _params;
};
