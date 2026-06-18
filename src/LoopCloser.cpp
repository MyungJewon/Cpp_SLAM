#include "LoopCloser.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include "PoseMath.h"

std::vector<LoopResult> LoopCloser::add(const Submap& newSubmap,
                                        const Pose3D& currentRefPose,
                                        const PoseLookup& lookup,
                                        std::ostream* log)
{
    std::vector<LoopResult> results;

    // 후보 수집: 현재 추정 위치 기준 반경 내 + 노드 간격 충분한 과거 서브맵.
    struct Cand { int idx; float dist; };
    std::vector<Cand> cands;
    const auto& cp = currentRefPose.t;
    for (size_t i = 0; i < _submaps.size(); ++i) {
        const Submap& past = _submaps[i];
        if (std::abs(newSubmap.anchorId - past.anchorId) < _minNodeGap)
            continue;
        Pose3D pastPose = lookup(past.anchorId);  // 현재 추정치 사용 (드리프트 보정 반영)
        float dx = cp[0] - pastPose.t[0];
        float dy = cp[1] - pastPose.t[1];
        float dz = cp[2] - pastPose.t[2];
        float d = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d <= _searchRadius)
            cands.push_back({(int)i, d});
    }

    if (log)
        *log << "# submap anchor " << newSubmap.anchorId
             << " : 반경내 후보 " << cands.size() << "개" << std::endl;

    if (!cands.empty()) {
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b){ return a.dist < b.dist; });
        int toCheck = std::min(_maxCandidates, (int)cands.size());

        for (int c = 0; c < toCheck; ++c) {
            const Submap& past = _submaps[cands[c].idx];
            Pose3D pastPose = lookup(past.anchorId);

            // 초기 추정: src(new) 로컬 → dst(past) 로컬 = inv(T_past) * T_new
            Pose3D guess = posemath::relativePoseOf(pastPose, currentRefPose);

            // AABB 1차 필터 (여유 = voxel 2칸)
            if (!aabbOverlap(newSubmap, past, guess, _params.finest() * 2.0f)) {
                if (log)
                    *log << "  cand 앵커 " << past.anchorId << " dist=" << cands[c].dist
                         << "  -> AABB 미겹침(skip)" << std::endl;
                continue;
            }

            RegistrationResult reg =
                registerSubmaps(newSubmap, past, guess, _params);

            std::cout << "[LoopCloser] 후보 앵커 " << past.anchorId
                      << " fitness=" << reg.fitness
                      << " overlap=" << reg.overlap
                      << " rmse=" << reg.rmse
                      << (reg.success ? "  -> 채택" : "") << std::endl;

            if (log)
                *log << "  cand 앵커 " << past.anchorId << " dist=" << cands[c].dist
                     << " fitness=" << reg.fitness << " overlap=" << reg.overlap
                     << " rmse=" << reg.rmse
                     << (reg.success ? "  -> 채택" : "  -> 게이트탈락") << std::endl;

            if (reg.success) {
                // BetweenFactor(from=past앵커, to=new앵커) 측정값 = src→dst 로컬 변환
                results.push_back({past.anchorId, newSubmap.anchorId,
                                   reg.relativePose, reg.fitness, reg.overlap});
            }
        }
    }

    _submaps.push_back(newSubmap);
    return results;
}
