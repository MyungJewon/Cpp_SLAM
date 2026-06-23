#pragma once
// SessionMerger: 여러 세션을 ScanContext 후보검색 → GICP → 일관성 클러스터링
// → GTSAM 통합 포즈그래프 최적화로 하나의 맵으로 병합한다.
#include <vector>
#include "Session.h"
#include "SubmapRegistration.h"

struct MergeParams {
    int   topK            = 5;     // 서브맵당 ScanContext 후보 수
    float maxScDist       = 0.6f;  // ScanContext 거리 상한 (후보 컷)
    int   minClusterSize  = 3;     // 세션 채택 최소 일관 매칭 수
    float clusterTransTol = 1.5f;  // T_SR 군집 이동 허용오차 (m)
    float clusterRotTol   = 0.15f; // T_SR 군집 회전 허용오차 (rad)
    RegistrationParams reg;        // GICP 게이트
    float outputVoxel     = 0.1f;  // 병합 출력 다운샘플
    int   outlierK        = 8;     // 통계적 아웃라이어 제거 k-NN
    float outlierStdMul   = 3.0f;  // 평균 + stdMul·σ 초과 제거

    // 거친 전역 정렬 (반복구조 aliasing 방어): 세션 전체 점군을 yaw 스윕 GICP로 먼저 맞춘다.
    bool  coarseAlign     = true;  // 서브맵 클러스터링 대신 전역 정렬 우선
    float coarseVoxel     = 0.2f;  // 전역 정렬용 다운샘플 (finest voxel보다 충분히 작게)
    int   coarseYawStep   = 10;    // yaw 스윕 간격(deg) — 촘촘해야 대칭 함정 회피
    float coarseMinOverlap= 0.3f;  // 전역 정렬 성공 판정 overlap 하한
    float coarseMinFitness= 0.2f;  // 전역 정렬 fitness 하한 (세션 간은 낮게 정상)
};

// 세션 간 서브맵 매칭 한 건. T_SR: 세션 src 월드 → 기준(ref) 월드 변환.
struct InterMatch {
    int    refIdx;   // 기준 세션 서브맵 인덱스
    int    srcIdx;   // src 세션 서브맵 인덱스
    Pose3D relativePose;  // src서브맵 로컬 → ref서브맵 로컬 (registerSubmaps 결과)
    Pose3D T_SR;     // 세션 레벨 변환
    float  fitness;
    float  overlap;
};

struct MergeResult {
    bool success = false;
    std::vector<std::vector<Pose3D>>  poses;    // poses[sess][submap] = 보정 world pose
    std::vector<std::array<float,3>>  cloud;    // 병합·다운샘플·아웃라이어제거 점군
    std::vector<int>                  sessionId; // cloud와 같은 길이, 각 점의 출처 세션
};

class SessionMerger {
public:
    explicit SessionMerger(MergeParams p = {}) : _p(p) {}

    // 기준 세션 ref에 src를 정렬하는 raw 매칭들 반환 (클러스터링 전).
    std::vector<InterMatch> findMatches(const Session& ref, const Session& src) const;

    // raw 매칭에서 최대 일관 군집 탐색. 성공 시 대표 T_SR과 인라이어 반환.
    bool bestCluster(const std::vector<InterMatch>& matches,
                     Pose3D& outT, std::vector<InterMatch>& inliers) const;

    // 세션 전체 점군(월드 좌표)을 yaw 스윕 GICP로 거칠게 정렬. 성공 시 T_SR 반환.
    bool coarseAlign(const std::vector<std::array<float,3>>& refWorld,
                     const std::vector<std::array<float,3>>& srcWorld,
                     Pose3D& T_SR) const;

    // 세션들을 병합. sessions[0]이 기준 좌표계.
    MergeResult merge(const std::vector<Session>& sessions) const;

private:
    MergeParams _p;
};
