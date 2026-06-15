#pragma once
#include <vector>
#include <array>
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

private:
    std::mutex  _mutex;
    std::atomic<bool> _running{true};

    // Shared buffers (protected by _mutex)
    std::vector<std::array<float,3>> _mapPoints;
    std::vector<std::array<float,3>> _trajectory;
    Matrix3x3            _rotation;
    std::array<float,3>  _position;
    int   _frameIdx  = 0;
    float _icpError  = 0.0f;
    bool  _dirty     = false;
};
