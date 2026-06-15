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
        // 키프레임 위치 기준으로 초기 translation 설정
        // 두 스캔 모두 로컬 좌표이므로 위치 차이를 초기값으로 넘겨 수렴 가능성을 높임
        Matrix3x3 initR = {1,0,0, 0,1,0, 0,0,1};
        std::array<float,3> initT = {
            kf->position[0] - currentPos[0],
            kf->position[1] - currentPos[1],
            kf->position[2] - currentPos[2]
        };
        ICPResult icp = runICP(currentPoints, kf->points, 30, 1e-4f, &initR, &initT, true);

        std::cout << "[LoopCloser] 프레임 " << kf->index
                  << " ICP 오차: " << icp.error << std::endl;

        if (icp.error < _icpThreshold)
        {
            // 3. 루프 감지 — 경로 보정
            // ICP가 현재 포인트 클라우드를 키프레임 클라우드로 정렬하면서
            // 구한 이동량(icp.t)이 두 위치 사이의 실제 기하 오프셋입니다.
            // 이전 구현은 icp.t를 버리고 위치 차이만 썼는데,
            // icp.t를 반영해야 Odometry 드리프트가 정확히 제거됩니다.
            std::array<float, 3> drift = {
                currentPos[0] - kf->position[0] - icp.t[0],
                currentPos[1] - kf->position[1] - icp.t[1],
                currentPos[2] - kf->position[2] - icp.t[2]
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