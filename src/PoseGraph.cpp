#include "PoseGraph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <stdexcept>

namespace
{
gtsam::Key key(int id)
{
    return gtsam::Symbol('x', id);
}

gtsam::Pose3 toGtsamPose(const Pose3D& pose)
{
    const auto& R = pose.R;
    gtsam::Rot3 rot(R[0], R[1], R[2],
                    R[3], R[4], R[5],
                    R[6], R[7], R[8]);
    return gtsam::Pose3(rot, gtsam::Point3(pose.t[0], pose.t[1], pose.t[2]));
}

Pose3D fromGtsamPose(const gtsam::Pose3& pose)
{
    const gtsam::Matrix3 R = pose.rotation().matrix();
    const gtsam::Point3 t = pose.translation();
    return {
        {
            static_cast<float>(R(0, 0)), static_cast<float>(R(0, 1)), static_cast<float>(R(0, 2)),
            static_cast<float>(R(1, 0)), static_cast<float>(R(1, 1)), static_cast<float>(R(1, 2)),
            static_cast<float>(R(2, 0)), static_cast<float>(R(2, 1)), static_cast<float>(R(2, 2))
        },
        {
            static_cast<float>(t.x()),
            static_cast<float>(t.y()),
            static_cast<float>(t.z())
        }
    };
}

gtsam::SharedNoiseModel diagonalNoise(double s0, double s1, double s2,
                                      double s3, double s4, double s5)
{
    gtsam::Vector6 sigmas;
    sigmas << s0, s1, s2, s3, s4, s5;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}
} // namespace

struct PoseGraph::Impl
{
    gtsam::ISAM2 isam;
    gtsam::Values estimate;

    gtsam::SharedNoiseModel odomNoise =
        diagonalNoise(0.05, 0.05, 0.05, 0.1, 0.1, 0.1);
    // 잘못된 루프 클로저 하나가 전체 그래프를 망치는 것을 막기 위해
    // Huber robust kernel로 감싼다 — 기존 추정과 크게 벗어나는 제약은
    // 영향력이 점진적으로 줄어들고, 완전히 무시되지는 않되 폭주하지 않음.
    gtsam::SharedNoiseModel loopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.0),
        diagonalNoise(0.1, 0.1, 0.1, 0.2, 0.2, 0.2));
    gtsam::SharedNoiseModel priorNoise =
        diagonalNoise(1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6);
};

PoseGraph::PoseGraph()
    : _impl(new Impl())
{
}

PoseGraph::~PoseGraph()
{
    delete _impl;
}

void PoseGraph::init(const Pose3D& firstPose)
{
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    const gtsam::Pose3 pose = toGtsamPose(firstPose);

    _currentId = 0;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(key(0), pose, _impl->priorNoise));
    initial.insert(key(0), pose);
    _impl->isam.update(graph, initial);
    _impl->estimate = _impl->isam.calculateEstimate();
}

Pose3D PoseGraph::addOdometry(const Pose3D& deltaPose)
{
    const int fromId = _currentId;
    const int toId = _currentId + 1;
    const gtsam::Pose3 delta = toGtsamPose(deltaPose);
    const gtsam::Pose3 previous = _impl->estimate.at<gtsam::Pose3>(key(fromId));
    const gtsam::Pose3 predicted = previous.compose(delta);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key(fromId), key(toId), delta, _impl->odomNoise));
    initial.insert(key(toId), predicted);

    _impl->isam.update(graph, initial);
    _impl->estimate = _impl->isam.calculateEstimate();
    _currentId = toId;
    return getPose(_currentId);
}

std::vector<Pose3D> PoseGraph::addLoopClosure(int fromId, int toId, const Pose3D& relativePose)
{
    if (fromId < 0 || toId < 0 || fromId > _currentId || toId > _currentId)
        return getAllPoses();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key(fromId), key(toId), toGtsamPose(relativePose), _impl->loopNoise));

    _impl->isam.update(graph, initial);
    _impl->isam.update();
    _impl->isam.update();
    _impl->isam.update();
    _impl->estimate = _impl->isam.calculateEstimate();
    return getAllPoses();
}

Pose3D PoseGraph::getPose(int id) const
{
    if (id < 0 || id > _currentId)
        throw std::out_of_range("PoseGraph pose id out of range");
    return fromGtsamPose(_impl->estimate.at<gtsam::Pose3>(key(id)));
}

std::vector<Pose3D> PoseGraph::getAllPoses() const
{
    std::vector<Pose3D> poses;
    poses.reserve(static_cast<size_t>(_currentId) + 1);
    for (int id = 0; id <= _currentId; ++id)
        poses.push_back(getPose(id));
    return poses;
}
