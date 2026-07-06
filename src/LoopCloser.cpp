#include "LoopCloser.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include "PoseMath.h"

namespace {
// 한 후보(past)와 newSubmap을 주어진 초기추정으로 정합 시도. 성공 시 LoopResult 반환(있음).
bool tryRegister(const Submap& newSubmap, const Submap& past,
                 const Pose3D& guess, const RegistrationParams& params,
                 float finestMargin, LoopResult& out, RegistrationResult& reg)
{
    if (!aabbOverlap(newSubmap, past, guess, finestMargin))
        return false;
    reg = registerSubmaps(newSubmap, past, guess, params);
    if (!reg.success)
        return false;
    out = {past.anchorId, newSubmap.anchorId, reg.relativePose, reg.fitness, reg.overlap};
    return true;
}
} // namespace

std::vector<LoopResult> LoopCloser::add(const Submap& newSubmap,
                                        const Pose3D& currentRefPose,
                                        const PoseLookup& lookup,
                                        std::ostream* log)
{
    std::vector<LoopResult> results;
    const float margin = _params.finest() * 2.0f;

    // ── 1순위: 위치 기반 후보 (드리프트가 작을 때 정확·저렴) ──
    struct Cand { int idx; float dist; };
    std::vector<Cand> cands;
    const auto& cp = currentRefPose.t;
    for (size_t i = 0; i < _submaps.size(); ++i) {
        const Submap& past = _submaps[i];
        if (std::abs(newSubmap.anchorId - past.anchorId) < _minNodeGap)
            continue;
        Pose3D pastPose = lookup(past.anchorId);
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
            Pose3D guess = posemath::relativePoseOf(lookup(past.anchorId), currentRefPose);
            LoopResult lr; RegistrationResult reg;
            bool ok = tryRegister(newSubmap, past, guess, _params, margin, lr, reg);
            if (log)
                *log << "  cand 앵커 " << past.anchorId << " dist=" << cands[c].dist
                     << " fitness=" << reg.fitness << " overlap=" << reg.overlap
                     << " rmse=" << reg.rmse
                     << (ok ? "  -> 채택" : "  -> 게이트탈락") << std::endl;
            if (ok) results.push_back(lr);
        }
    }

    // ── 2순위: ScanContext 형태 검색 (위치 기반이 루프를 못 찾았을 때만) ──
    // 드리프트가 searchRadius를 넘으면 위치 후보가 비거나 다 어긋난다. 이때 형태
    // 디스크립터로 위치 무관하게 후보를 찾아 큰 드리프트 상황의 루프를 복구한다.
    ScanContextDesc newDesc = computeScanContextCentered(newSubmap.points);
    if (_scEnabled && results.empty() && !_descs.empty()) {
        struct SC { int idx; float dist; int shift; };
        std::vector<SC> scCands;
        for (size_t i = 0; i < _submaps.size(); ++i) {
            if (std::abs(newSubmap.anchorId - _submaps[i].anchorId) < _minNodeGap)
                continue;
            auto ds = scanContextDistance(newDesc, _descs[i]);
            if (ds.first <= _scMaxDist)
                scCands.push_back({(int)i, ds.first, ds.second});
        }
        std::sort(scCands.begin(), scCands.end(),
                  [](const SC& a, const SC& b){ return a.dist < b.dist; });
        int toCheck = std::min(_maxCandidates, (int)scCands.size());
        if (log && toCheck > 0)
            *log << "  [ScanContext 폴백] 후보 " << scCands.size() << "개" << std::endl;

        // SC 경로 강화 게이트: 위치 사전정보 없이 들어오는 루프라 보수적으로.
        // 실측 근거 — 진짜 재방문 overlap 0.6~0.93, perceptual alias 0.43~0.57
        // (멀티세션 실험에서 가짜가 기본 게이트 0.4/0.25를 통과함을 확인).
        RegistrationParams scParams = _params;
        scParams.minOverlap = std::max(scParams.minOverlap, 0.55f);
        scParams.minFitness = std::max(scParams.minFitness, 0.35f);

        for (int c = 0; c < toCheck; ++c) {
            const Submap& past = _submaps[scCands[c].idx];
            // yaw 초기추정 = bestShift × 섹터각. 이동은 0(양쪽 anchor-local이라 원점 근방).
            float yaw = scCands[c].shift * (2.0f * (float)M_PI / SC_SECTORS);
            float cs = std::cos(yaw), sn = std::sin(yaw);
            Pose3D guess = {{cs,-sn,0, sn,cs,0, 0,0,1}, {0,0,0}};
            LoopResult lr; RegistrationResult reg;
            bool ok = tryRegister(newSubmap, past, guess, scParams, margin, lr, reg);
            if (log)
                *log << "  cand(SC) 앵커 " << past.anchorId << " scDist=" << scCands[c].dist
                     << " fitness=" << reg.fitness << " overlap=" << reg.overlap
                     << " rmse=" << reg.rmse
                     << (ok ? "  -> 채택" : "  -> 게이트탈락") << std::endl;
            if (ok) results.push_back(lr);
        }
    }

    _submaps.push_back(newSubmap);
    _descs.push_back(newDesc);
    return results;
}
