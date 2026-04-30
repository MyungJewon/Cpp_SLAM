// Loop Closure 구현: 과거 프레임과 현재 프레임을 비교해 누적 오차를 보정합니다.
#include "LoopCloser.h"
#include <cmath>
#include <iostream>

LoopCloser::LoopCloser(float searchRadius, float icpThreshold, int minFrameGap)
    : _searchRadius(searchRadius)
    , _icpThreshold(icpThreshold)
    , _minFrameGap(minFrameGap)
{
}

void LoopCloser::addKeyFrame(int index,
                              const std::array<float, 3>&              position,
                              const std::vector<std::array<float, 3>>& points)
{
    KeyFrame kf;
    kf.index    = index;
    kf.position = position;
    kf.points   = points;
    _keyFrames.push_back(kf);
}

bool LoopCloser::detect(const std::array<float, 3>&              currentPos,
                         const std::vector<std::array<float, 3>>& currentPoints,
                         std::vector<std::array<float, 3>>&       trajectory)
{
    int currentIndex = (int)_keyFrames.size();

    // 1. 반경 안의 과거 프레임을 후보로 추림
    std::vector<const KeyFrame*> candidates;
    for (const auto& kf : _keyFrames)
    {
        // 최근 프레임은 제외
        if (currentIndex - kf.index < _minFrameGap) continue;

        // 현재 위치와 키프레임 위치 사이의 거리 계산
        float dx = currentPos[0] - kf.position[0];
        float dy = currentPos[1] - kf.position[1];
        float dz = currentPos[2] - kf.position[2];
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < _searchRadius)
            candidates.push_back(&kf);
    }

    if (candidates.empty())
    {
        std::cout << "[LoopCloser] 후보 없음" << std::endl;
        return false;
    }

    std::cout << "[LoopCloser] 후보 " << candidates.size() << "개 발견, ICP 검증 중..." << std::endl;

    // 2. 후보마다 ICP 실행해서 루프 확인
    for (const auto* kf : candidates)
    {
        ICPResult icp = runICP(currentPoints, kf->points);

        std::cout << "[LoopCloser] 프레임 " << kf->index
                  << " ICP 오차: " << icp.error << std::endl;

        if (icp.error < _icpThreshold)
        {
            // 3. 루프 감지 — 경로 보정
            // Odometry가 추정한 현재 위치와 키프레임 위치의 차이가 누적 오차
            // icp.t는 두 포인트 클라우드가 얼마나 다른지를 나타내므로 제외하고
            // 순수하게 위치 차이만으로 drift를 계산해요
            std::array<float, 3> drift = {
                currentPos[0] - kf->position[0],
                currentPos[1] - kf->position[1],
                currentPos[2] - kf->position[2]
            };

            std::cout << "[LoopCloser] 루프 감지! 프레임 " << kf->index
                      << " | 누적 오차: ("
                      << drift[0] << ", " << drift[1] << ", " << drift[2] << ")"
                      << std::endl;

            // 4. 루프 감지 이후 프레임들을 선형 보정
            // 루프 발생 시점(kf->index) 이후 프레임부터 현재까지
            // 오차를 균등하게 나눠서 줄여나감
            int   loopStart  = kf->index;
            int   loopEnd    = (int)trajectory.size() - 1;
            int   loopLength = loopEnd - loopStart;

            if (loopLength > 0)
            {
                for (int i = loopStart; i <= loopEnd; ++i)
                {
                    float ratio = (float)(i - loopStart) / (float)loopLength;
                    trajectory[i][0] -= drift[0] * ratio;
                    trajectory[i][1] -= drift[1] * ratio;
                    trajectory[i][2] -= drift[2] * ratio;
                }
            }

            return true;
        }
    }

    std::cout << "[LoopCloser] 루프 없음 (오차 임계값 초과)" << std::endl;
    return false;
}