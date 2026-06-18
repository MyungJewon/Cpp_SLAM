#pragma once
// 두 Submap을 정합하는 재사용 코어.
// 실시간 루프클로저(LoopCloser)와 오프라인 PLY 병합(MapMerger)이 공통으로 호출한다.
#include "Submap.h"

struct RegistrationParams {
    // coarse-to-fine 다해상도 GICP voxel 크기 (큰 것부터 작은 것 순).
    // GLIM식: 거친 voxel로 먼저 정렬해 local minimum을 피하고 점진적으로 미세화한다.
    // 주의: 우리 ICP는 voxel당 점 6개 이상을 요구하므로 가장 작은 해상도는
    //       서브맵 다운샘플 간격의 ~3배 이상이어야 한다 (0.15m 다운샘플 → 0.5m 최소).
    std::vector<float> resolutions = {1.0f, 0.5f};
    float eigenFloor    = 1e-3f;  // 공분산 고유값 상대 플로어
    int   maxIterations = 30;
    float minFitness    = 0.5f;   // GICP inlier(대응점) 비율 하한 (GLIM inlier_fraction=0.5)
    float minOverlap    = 0.3f;   // 정렬 후 겹침 비율 하한 (false loop 차단의 핵심)
    float maxRmse       = 0.5f;   // 최종 오차 상한

    float finest() const { return resolutions.empty() ? 0.5f : resolutions.back(); }
};

// src(로컬)를 dst(로컬)에 맞춘다. initialGuess는 src→dst 초기 변환.
RegistrationResult registerSubmaps(const Submap& src,
                                   const Submap& dst,
                                   const Pose3D& initialGuess,
                                   const RegistrationParams& params = {});

// 두 서브맵 AABB가 (현재 추정 상대 변환 기준으로) 겹치는지 빠른 1차 필터.
// guess: src→dst 변환. margin만큼 여유를 둔다.
bool aabbOverlap(const Submap& src, const Submap& dst,
                 const Pose3D& guess, float margin = 0.0f);
