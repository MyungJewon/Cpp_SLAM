#pragma once
#include <array>
#include <vector>
#include "ICP.h"
#include "PoseGraph.h"
#include "VoxelGrid.h"

struct KeyFrame
{
    int                                  index;
    std::array<float, 3>                 position;
    Matrix3x3                            rotation;
    std::vector<std::array<float, 3>>    points;
};

class LoopCloser
{
public:
    LoopCloser(float searchRadius  = 1.0f,
               float icpThreshold  = 0.05f,
               int   minFrameGap   = 10);

    void addKeyFrame(int index,
                     const std::array<float, 3>& position,
                     const Matrix3x3& rotation,
                     const std::vector<std::array<float, 3>>& points);

    bool detect(const std::array<float, 3>& currentPos,
                const Matrix3x3& currentRot,
                const std::vector<std::array<float, 3>>& currentPoints);

    bool detect(int currentIndex,
                const std::array<float, 3>& currentPos,
                const Matrix3x3& currentRot,
                const std::vector<std::array<float, 3>>& currentPoints);

    void setTopNCandidates(int n) { _topNCandidates = n; }
    void setLoopCooldown(int frames) { _loopCooldown = frames; }

    int getLastLoopFromId() const { return _lastLoopFromId; }
    int getLastLoopToId() const { return _lastLoopToId; }
    Pose3D getLastLoopRelativePose() const { return _lastLoopRelativePose; }

private:
    float _searchRadius;
    float _icpThreshold;
    int   _minFrameGap;
    float _minFitness = 0.6f;  // 매칭된 점 비율이 이 이하면 우연히 비슷한 구조물로 간주
    int   _topNCandidates = 5;
    int   _loopCooldown   = 100;

    std::vector<KeyFrame> _keyFrames;
    std::vector<int> _detectedFromIds;
    int _lastLoopFrame = -9999;
    int _lastLoopFromId = -1;
    int _lastLoopToId = -1;
    Pose3D _lastLoopRelativePose = {
        {1,0,0, 0,1,0, 0,0,1},
        {0,0,0}
    };
};
