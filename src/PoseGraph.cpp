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
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <iostream>

namespace
{
gtsam::Key key(int id)
{
    return gtsam::Symbol('x', id);
}

gtsam::Key vKey(int id) { return gtsam::Symbol('v', id); }
gtsam::Key bKey(int id) { return gtsam::Symbol('b', id); }

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

    // ── IMU 타이트커플링 (LIO-SAM식 단일 그래프) ──
    bool imuEnabled = false;
    std::shared_ptr<gtsam::PreintegrationCombinedParams> imuParams;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preint;
    gtsam::imuBias::ConstantBias initialBias;
    double lastImuTs = -1.0;

    gtsam::SharedNoiseModel velPriorNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
    gtsam::SharedNoiseModel biasPriorNoise2 =
        diagonalNoise(0.3, 0.3, 0.3, 1e-3, 1e-3, 1e-3);  // [acc(3), gyro(3)]

    gtsam::imuBias::ConstantBias biasAt(int id) const
    {
        if (estimate.exists(bKey(id)))
            return estimate.at<gtsam::imuBias::ConstantBias>(bKey(id));
        return initialBias;
    }

    void ensurePreint(int curId)
    {
        if (!preint && imuParams)
            preint = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
                imuParams, biasAt(curId));
    }
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

    // IMU 활성화 시 velocity/bias 노드도 prior와 함께 생성
    if (_impl->imuEnabled)
    {
        const gtsam::Vector3 v0(0.0, 0.0, 0.0);
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(vKey(0), v0, _impl->velPriorNoise));
        graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
            bKey(0), _impl->initialBias, _impl->biasPriorNoise2));
        initial.insert(vKey(0), v0);
        initial.insert(bKey(0), _impl->initialBias);
        _impl->preint.reset();
        _impl->lastImuTs = -1.0;
    }

    _impl->isam.update(graph, initial);
    _impl->estimate = _impl->isam.calculateEstimate();
}

void PoseGraph::enableImu(const std::array<double, 3>& gyroBias, double gravityMag,
                          const std::array<float, 3>& navGravity)
{
    auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(gravityMag);
    // FAST-LIVO2 avia.yaml 노이즈 (loose IMU — LiDAR를 더 신뢰)
    params->setAccelerometerCovariance(gtsam::Matrix3::Identity() * 0.5);
    params->setGyroscopeCovariance(gtsam::Matrix3::Identity() * 0.3);
    params->setBiasAccCovariance(gtsam::Matrix3::Identity() * 1e-4);
    params->setBiasOmegaCovariance(gtsam::Matrix3::Identity() * 1e-4);
    params->setIntegrationCovariance(gtsam::Matrix3::Identity() * 1e-8);
    // 측정한 nav 중력 방향 설정 (시작 자세 안 기울이고 중력 방향만 알려줌)
    params->n_gravity = gtsam::Vector3(navGravity[0], navGravity[1], navGravity[2]);

    _impl->imuParams = params;
    _impl->initialBias = gtsam::imuBias::ConstantBias(
        gtsam::Vector3(0.0, 0.0, 0.0),
        gtsam::Vector3(gyroBias[0], gyroBias[1], gyroBias[2]));
    _impl->imuEnabled = true;
}

void PoseGraph::integrateImu(const std::vector<ImuSample>& samples)
{
    if (!_impl->imuEnabled) return;
    _impl->ensurePreint(_currentId);
    for (const auto& s : samples)
    {
        if (_impl->lastImuTs < 0.0) { _impl->lastImuTs = s.timestamp; continue; }
        double dt = s.timestamp - _impl->lastImuTs;
        _impl->lastImuTs = s.timestamp;
        if (dt <= 0.0 || dt > 0.5) continue;
        _impl->preint->integrateMeasurement(
            gtsam::Vector3(s.linearAcceleration[0], s.linearAcceleration[1], s.linearAcceleration[2]),
            gtsam::Vector3(s.angularVelocity[0], s.angularVelocity[1], s.angularVelocity[2]),
            dt);
    }
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

    // IMU 타이트커플링: 누적된 preintegration으로 CombinedImuFactor 추가.
    // GICP delta(BetweenFactor)와 IMU가 같은 그래프에서 X,V,B를 동시에 최적화한다.
    if (_impl->imuEnabled)
    {
        const gtsam::Vector3 prevVel = _impl->estimate.exists(vKey(fromId))
            ? _impl->estimate.at<gtsam::Vector3>(vKey(fromId)) : gtsam::Vector3(0,0,0);
        const gtsam::imuBias::ConstantBias prevBias = _impl->biasAt(fromId);

        gtsam::Vector3 predVel = prevVel;
        if (_impl->preint && _impl->preint->deltaTij() > 0.0)
        {
            graph.add(gtsam::CombinedImuFactor(
                key(fromId), vKey(fromId),
                key(toId),   vKey(toId),
                bKey(fromId), bKey(toId),
                *_impl->preint));
            gtsam::NavState pred = _impl->preint->predict(
                gtsam::NavState(previous, prevVel), prevBias);
            predVel = pred.velocity();
        }
        else
        {
            // IMU 공백: v/b를 직전에 묶어 underconstrained 방지
            graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                bKey(fromId), bKey(toId), gtsam::imuBias::ConstantBias(),
                diagonalNoise(1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4)));
            graph.add(gtsam::BetweenFactor<gtsam::Vector3>(
                vKey(fromId), vKey(toId), gtsam::Vector3(0,0,0),
                gtsam::noiseModel::Isotropic::Sigma(3, 2.0)));
        }
        initial.insert(vKey(toId), predVel);
        initial.insert(bKey(toId), prevBias);
    }

    try
    {
        _impl->isam.update(graph, initial);
        _impl->estimate = _impl->isam.calculateEstimate();
        _currentId = toId;
    }
    catch (const std::exception& e)
    {
        // 발산/특이행렬 — 이번 노드를 버리고 직전 상태 유지 (크래시 방지)
        std::cerr << "[PoseGraph] iSAM2 update 실패 (" << e.what()
                  << ") — 노드 " << toId << " 건너뜀" << std::endl;
        return getPose(_currentId);
    }

    // 다음 프레임을 위해 preintegration 리셋 (새 bias 추정으로)
    if (_impl->imuEnabled)
    {
        _impl->preint.reset();
        _impl->lastImuTs = -1.0;
    }
    return getPose(_currentId);
}

std::vector<Pose3D> PoseGraph::addLoopClosure(int fromId, int toId, const Pose3D& relativePose,
                                              float confidence)
{
    if (fromId < 0 || toId < 0 || fromId > _currentId || toId > _currentId)
        return getAllPoses();

    // 게이트(fitness/overlap)를 이미 통과한 루프는 신뢰할 만하므로 타이트한 base를 쓴다.
    // sigma = base / confidence: 신뢰도가 높을수록 더 강하게 그래프를 당긴다 (GLIM Hessian 근사).
    // base가 너무 크면(이전 0.1rad/0.2m) 누적 drift를 못 이겨 두 바퀴가 안 붙는다.
    // Huber robust로 감싸 잘못된 루프 하나가 그래프를 폭주시키는 것을 막는다.
    float c = std::min(std::max(confidence, 0.1f), 1.0f);
    double sr = 0.02 / c;  // 회전 sigma (rad) — base 1.15°
    double st = 0.05 / c;  // 이동 sigma (m)  — base 5cm
    auto loopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.0),
        diagonalNoise(sr, sr, sr, st, st, st));

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key(fromId), key(toId), toGtsamPose(relativePose), loopNoise));

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
