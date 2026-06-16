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
    Pose3D addOdometry(const Pose3D& deltaPose);
    std::vector<Pose3D> addLoopClosure(int fromId, int toId, const Pose3D& relativePose);
    int getCurrentId() const { return _currentId; }
    Pose3D getPose(int id) const;
    std::vector<Pose3D> getAllPoses() const;

private:
    struct Impl;
    Impl* _impl;
    int _currentId = 0;
};
