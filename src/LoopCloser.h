// Loop Closure 선언: 과거 프레임과 현재 프레임을 비교해 누적 오차를 보정합니다.
#pragma once
#include <array>
#include <vector>
#include "ICP.h"
#include "VoxelGrid.h"

// 과거 프레임 하나를 저장하는 구조체
struct KeyFrame
{
    int                                  index;     // 프레임 번호
    std::array<float, 3>                 position;  // 당시 추정 위치
    std::vector<std::array<float, 3>>    points;    // 다운샘플링된 포인트
};

class LoopCloser
{
public:
    // searchRadius : 루프 후보를 탐색할 반경 (meter)
    // icpThreshold : 이 오차 이하면 루프로 판단
    // minFrameGap  : 최근 N프레임은 루프 후보에서 제외 (바로 직전 프레임과 매칭 방지)
    LoopCloser(float searchRadius  = 1.0f,
               float icpThreshold  = 0.05f,
               int   minFrameGap   = 10);

    // 새 키프레임 등록
    void addKeyFrame(int index,
                     const std::array<float, 3>&              position,
                     const std::vector<std::array<float, 3>>& points);

    // 루프 감지 시도
    // 감지되면 true 반환, correctedTrajectory에 보정된 경로를 채워줌
    bool detect(const std::array<float, 3>&              currentPos,
                const std::vector<std::array<float, 3>>& currentPoints,
                std::vector<std::array<float, 3>>&       trajectory);

private:
    float _searchRadius;
    float _icpThreshold;
    int   _minFrameGap;

    std::vector<KeyFrame> _keyFrames;
};