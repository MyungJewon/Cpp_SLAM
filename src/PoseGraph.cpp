#include "PoseGraph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>

#include <stdexcept>
#include <cmath>

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

    // 자세 factor 기준 "위"(월드 좌표). 첫 중력 측정으로 1회 설정.
    // (0,0,1)을 강제하지 않고 시작 자세 대비 drift만 막는다 — 센서 비수평 장착 대응.
    gtsam::Unit3 worldUpRef;
    bool worldUpSet = false;
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

Pose3D PoseGraph::addOdometry(const Pose3D& deltaPose,
                             const std::array<float, 3>* gravityBodyUp)
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

    // IMU 중력 자세 factor: 바디 "위"(bRef)가 월드 up(nZ=(0,0,1))에 정렬되도록.
    // pitch/roll 2-DOF만 구속(yaw 자유). loose noise라 LiDAR가 국소적으로 지배하되
    // 전역 pitch 드리프트는 중력으로 묶인다.
    if (gravityBodyUp)
    {
        const auto& g = *gravityBodyUp;
        float n = std::sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
        if (n > 1e-6f)
        {
            gtsam::Unit3 bRef(g[0]/n, g[1]/n, g[2]/n);

            // 첫 측정에서 월드 기준 up을 캡처: nZ = R_node * g_body (현재 추정 자세로
            // 바디 중력을 월드로 회전). 이후 모든 factor가 이 기준에 맞춰 자세 drift만 억제.
            if (!_impl->worldUpSet)
            {
                gtsam::Point3 gw = predicted.rotation().rotate(gtsam::Point3(g[0]/n, g[1]/n, g[2]/n));
                _impl->worldUpRef = gtsam::Unit3(gw);
                _impl->worldUpSet = true;
            }

            auto attNoise = gtsam::noiseModel::Isotropic::Sigma(2, _attitudeSigma);
            graph.add(gtsam::AttitudeFactor<gtsam::Pose3>(
                key(toId), _impl->worldUpRef, attNoise, bRef));
        }
    }

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
