#pragma once
#include <vector>
#include <array>
#include <utility>
#include <mutex>
#include <atomic>
#include "ICP.h"  // for Matrix3x3

class PangolinViewer {
public:
    PangolinViewer();
    ~PangolinViewer();

    void runBlocking();
    bool shouldQuit() const;
    void requestStop();

    // Called from SLAM background thread each frame
    void update(const std::vector<std::array<float,3>>& mapPoints,
                const std::vector<std::array<float,3>>& trajectory,
                const Matrix3x3& rotation,
                const std::array<float,3>& position,
                int frameIdx,
                float icpError);

    // 루프 클로저 연결선 (각 원소 = {fromAnchor 위치, toAnchor 위치}).
    // 어느 지점끼리 루프로 묶였는지 시각화한다.
    void setLoopEdges(const std::vector<std::pair<std::array<float,3>,
                                                  std::array<float,3>>>& edges);

private:
    std::mutex  _mutex;
    std::atomic<bool> _running{true};

    // Shared buffers (protected by _mutex)
    std::vector<std::array<float,3>> _mapPoints;
    std::vector<std::array<float,3>> _trajectory;
    std::vector<std::pair<std::array<float,3>, std::array<float,3>>> _loopEdges;
    Matrix3x3            _rotation;
    std::array<float,3>  _position;
    int   _frameIdx  = 0;
    float _icpError  = 0.0f;
    bool  _dirty     = false;
};
