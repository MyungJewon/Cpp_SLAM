#pragma once
#include <vector>
#include <array>
#include "ICP.h"  // for Matrix3x3

// Pose = rotation (Matrix3x3 row-major) + translation
struct Pose3D {
    Matrix3x3            R;
    std::array<float, 3> t;
};

class PoseGraph {
public:
    PoseGraph();
    ~PoseGraph();

    void init(const Pose3D& firstPose);
    // deltaPose: 직전 노드 대비 상대 변환.
    // gravityBodyUp: 이 노드의 IMU 중력(바디 좌표). nullptr이면 자세 factor 미추가.
    Pose3D addOdometry(const Pose3D& deltaPose,
                       const std::array<float, 3>* gravityBodyUp = nullptr);
    // confidence: 정합 신뢰도(0~1, 보통 overlap×fitness). 높을수록 루프를 강하게 구속한다.
    // GLIM의 Hessian 기반 정보행렬 가중을 근사 — 약한 루프는 영향이 작아진다.
    std::vector<Pose3D> addLoopClosure(int fromId, int toId, const Pose3D& relativePose,
                                       float confidence = 1.0f);

    // 자세 factor 노이즈 (rad). 작을수록 중력 정렬을 강하게 강제.
    void setAttitudeSigma(float rad) { _attitudeSigma = rad; }
    int getCurrentId() const { return _currentId; }
    Pose3D getPose(int id) const;
    std::vector<Pose3D> getAllPoses() const;

private:
    struct Impl;
    Impl* _impl;
    int _currentId = 0;
    float _attitudeSigma = 0.1f;  // 자세 factor 노이즈 (rad, ~5.7°)
};
