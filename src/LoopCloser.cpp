#include "LoopCloser.h"
#include <algorithm>
#include <cmath>
#include <iostream>

LoopCloser::LoopCloser(float searchRadius, float icpThreshold, int minFrameGap)
    : _searchRadius(searchRadius)
    , _icpThreshold(icpThreshold)
    , _minFrameGap(minFrameGap)
{
}

void LoopCloser::addKeyFrame(int index,
                              const std::array<float, 3>& position,
                              const Matrix3x3& rotation,
                              const std::vector<std::array<float, 3>>& points)
{
    KeyFrame kf;
    kf.index    = index;
    kf.position = position;
    kf.rotation = rotation;
    kf.points   = points;
    _keyFrames.push_back(kf);
}

bool LoopCloser::detect(const std::array<float, 3>& currentPos,
                         const Matrix3x3& currentRot,
                         const std::vector<std::array<float, 3>>& currentPoints)
{
    return detect((int)_keyFrames.size(), currentPos, currentRot, currentPoints);
}

bool LoopCloser::detect(int currentIndex,
                         const std::array<float, 3>& currentPos,
                         const Matrix3x3& currentRot,
                         const std::vector<std::array<float, 3>>& currentPoints)
{
    _lastLoopFromId = -1;
    _lastLoopToId = -1;

    if (currentIndex - _lastLoopFrame < _loopCooldown)
        return false;

    struct Candidate
    {
        const KeyFrame* kf;
        float dist;
    };
    std::vector<Candidate> scored;
    for (const auto& kf : _keyFrames)
    {
        if (currentIndex - kf.index < _minFrameGap) continue;
        bool alreadyDetected = false;
        for (int id : _detectedFromIds)
            if (id == kf.index) { alreadyDetected = true; break; }
        if (alreadyDetected) continue;

        float dx = currentPos[0] - kf.position[0];
        float dy = currentPos[1] - kf.position[1];
        float dz = currentPos[2] - kf.position[2];
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < _searchRadius)
            scored.push_back({&kf, dist});
    }

    if (scored.empty())
    {
        std::cout << "[LoopCloser] 후보 없음" << std::endl;
        return false;
    }

    std::sort(scored.begin(), scored.end(),
              [](const Candidate& a, const Candidate& b){ return a.dist < b.dist; });

    int toCheck = std::min(_topNCandidates, (int)scored.size());
    std::vector<Candidate> candidates;
    for (int i = 0; i < toCheck; ++i)
        candidates.push_back(scored[i]);

    std::cout << "[LoopCloser] 후보 " << candidates.size() << "개 발견, ICP 검증 중..." << std::endl;

    for (const auto& cand : candidates)
    {
        const KeyFrame* kf = cand.kf;

        // GTSAM이 이미 추정한 두 키프레임의 절대 pose로 상대 변환을 직접 계산
        // (GLIM의 T_target^-1 * T_source 방식) — 무작위 각도 추측보다 신뢰도 높은 초기값
        Matrix3x3 invKfR = {
            kf->rotation[0], kf->rotation[3], kf->rotation[6],
            kf->rotation[1], kf->rotation[4], kf->rotation[7],
            kf->rotation[2], kf->rotation[5], kf->rotation[8]
        };
        std::array<float,3> dPos = {
            currentPos[0] - kf->position[0],
            currentPos[1] - kf->position[1],
            currentPos[2] - kf->position[2]
        };
        std::array<float,3> initT = {
            invKfR[0]*dPos[0] + invKfR[1]*dPos[1] + invKfR[2]*dPos[2],
            invKfR[3]*dPos[0] + invKfR[4]*dPos[1] + invKfR[5]*dPos[2],
            invKfR[6]*dPos[0] + invKfR[7]*dPos[1] + invKfR[8]*dPos[2]
        };
        Matrix3x3 initR = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    initR[i*3+j] += invKfR[i*3+k] * currentRot[k*3+j];

        ICPResult bestIcp = runICP(currentPoints, kf->points, 20, 1e-4f, &initR, &initT, true);

        std::cout << "[LoopCloser] 프레임 " << kf->index
                  << " ICP 오차: " << bestIcp.error
                  << " | fitness: " << bestIcp.fitness << std::endl;

        if (bestIcp.error < _icpThreshold && bestIcp.fitness > _minFitness)
        {
            std::cout << "[LoopCloser] 루프 감지! 프레임 " << kf->index << std::endl;

            Matrix3x3 invR = {
                bestIcp.R[0], bestIcp.R[3], bestIcp.R[6],
                bestIcp.R[1], bestIcp.R[4], bestIcp.R[7],
                bestIcp.R[2], bestIcp.R[5], bestIcp.R[8]
            };
            std::array<float,3> invT = {
                -(invR[0]*bestIcp.t[0] + invR[1]*bestIcp.t[1] + invR[2]*bestIcp.t[2]),
                -(invR[3]*bestIcp.t[0] + invR[4]*bestIcp.t[1] + invR[5]*bestIcp.t[2]),
                -(invR[6]*bestIcp.t[0] + invR[7]*bestIcp.t[1] + invR[8]*bestIcp.t[2])
            };

            _lastLoopFromId = kf->index;
            _lastLoopToId = currentIndex;
            _lastLoopRelativePose = {invR, invT};
            _lastLoopFrame = currentIndex;
            _detectedFromIds.push_back(kf->index);

            return true;
        }
    }

    std::cout << "[LoopCloser] 루프 없음 (오차 임계값 초과)" << std::endl;
    return false;
}
