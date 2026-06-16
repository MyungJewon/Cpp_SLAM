#include "ImuOdometry.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
gtsam::Key xKey(int id)
{
    return gtsam::Symbol('x', id);
}

gtsam::Key vKey(int id)
{
    return gtsam::Symbol('v', id);
}

gtsam::Key bKey(int id)
{
    return gtsam::Symbol('b', id);
}

gtsam::Pose3 toGtsamPose(const Matrix3x3& R, const std::array<float, 3>& t)
{
    gtsam::Rot3 rot(R[0], R[1], R[2],
                    R[3], R[4], R[5],
                    R[6], R[7], R[8]);
    return gtsam::Pose3(rot, gtsam::Point3(t[0], t[1], t[2]));
}

Matrix3x3 fromGtsamRotation(const gtsam::Pose3& pose)
{
    const gtsam::Matrix3 R = pose.rotation().matrix();
    return {
        static_cast<float>(R(0, 0)), static_cast<float>(R(0, 1)), static_cast<float>(R(0, 2)),
        static_cast<float>(R(1, 0)), static_cast<float>(R(1, 1)), static_cast<float>(R(1, 2)),
        static_cast<float>(R(2, 0)), static_cast<float>(R(2, 1)), static_cast<float>(R(2, 2))
    };
}

std::array<float, 3> fromGtsamTranslation(const gtsam::Pose3& pose)
{
    const gtsam::Point3 t = pose.translation();
    return {
        static_cast<float>(t.x()),
        static_cast<float>(t.y()),
        static_cast<float>(t.z())
    };
}

std::array<float, 3> fromGtsamVector3(const gtsam::Vector3& v)
{
    return {
        static_cast<float>(v.x()),
        static_cast<float>(v.y()),
        static_cast<float>(v.z())
    };
}

gtsam::SharedNoiseModel diagonalNoise(double s0, double s1, double s2,
                                      double s3, double s4, double s5)
{
    gtsam::Vector6 sigmas;
    sigmas << s0, s1, s2, s3, s4, s5;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

ImuOdometry::Prediction makePrediction(const gtsam::Pose3& pose,
                                       const gtsam::Vector3& velocity)
{
    return {
        fromGtsamRotation(pose),
        fromGtsamTranslation(pose),
        fromGtsamVector3(velocity)
    };
}
} // namespace

struct ImuOdometry::Impl
{
    explicit Impl(const std::array<double, 3>& gyroBias, double gravityMagnitude)
        : preintParams(gtsam::PreintegrationCombinedParams::MakeSharedU(gravityMagnitude))
        , initialBias(gtsam::Vector3(0.0, 0.0, 0.0),
                      gtsam::Vector3(gyroBias[0], gyroBias[1], gyroBias[2]))
    {
        const double gyroNoiseSigma = 1e-3;
        const double accelNoiseSigma = 1e-2;
        const double gyroBiasRwSigma = 1e-5;
        // 가속도 바이어스는 캘리브레이션을 안 해서 0으로 시작하는데,
        // random walk가 너무 작으면(이전 1e-4) factor graph가 실측값을 보고도
        // 바이어스를 거의 못 움직여서 z가 그대로 적분 오차로 누적됨.
        // 충분히 키워서 그래프가 실제로 바이어스를 추정/교정하게 함.
        const double accelBiasRwSigma = 3e-3;
        const double integrationSigma = 1e-8;

        preintParams->setGyroscopeCovariance(
            gtsam::Matrix3::Identity() * gyroNoiseSigma * gyroNoiseSigma);
        preintParams->setAccelerometerCovariance(
            gtsam::Matrix3::Identity() * accelNoiseSigma * accelNoiseSigma);
        preintParams->setBiasOmegaCovariance(
            gtsam::Matrix3::Identity() * gyroBiasRwSigma * gyroBiasRwSigma);
        preintParams->setBiasAccCovariance(
            gtsam::Matrix3::Identity() * accelBiasRwSigma * accelBiasRwSigma);
        preintParams->setIntegrationCovariance(
            gtsam::Matrix3::Identity() * integrationSigma * integrationSigma);
    }

    gtsam::ISAM2 isam;
    std::shared_ptr<gtsam::PreintegrationCombinedParams> preintParams;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preint;
    gtsam::Values currentEstimate;
    gtsam::imuBias::ConstantBias initialBias;
    double lastTimestamp = -1.0;
    int currentId = 0;

    gtsam::SharedNoiseModel posePriorNoise =
        diagonalNoise(1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6);
    gtsam::SharedNoiseModel velocityPriorNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
    // ConstantBias 벡터 순서는 [accBias(3), gyroBias(3)].
    // 자이로 바이어스는 정지 구간에서 캘리브레이션했으니 prior를 좁게 유지하고,
    // 가속도 바이어스는 0으로 추측만 한 값이라 prior를 넓게 풀어줘서
    // factor graph가 실측으로 자유롭게 교정할 수 있게 함.
    gtsam::SharedNoiseModel biasPriorNoise =
        diagonalNoise(0.3, 0.3, 0.3, 1e-3, 1e-3, 1e-3);

    gtsam::imuBias::ConstantBias currentBias() const
    {
        if (currentEstimate.exists(bKey(currentId)))
            return currentEstimate.at<gtsam::imuBias::ConstantBias>(bKey(currentId));
        return initialBias;
    }

    void ensurePreintegrator()
    {
        if (!preint)
            preint = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
                preintParams, currentBias());
    }
};

ImuOdometry::ImuOdometry(const std::array<double,3>& gyroBias, double gravityMagnitude)
    : _impl(new Impl(gyroBias, gravityMagnitude))
{
}

ImuOdometry::~ImuOdometry()
{
    delete _impl;
}

void ImuOdometry::init(const Matrix3x3& R0, const std::array<float,3>& t0)
{
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    const gtsam::Pose3 pose = toGtsamPose(R0, t0);
    const gtsam::Vector3 velocity(0.0, 0.0, 0.0);

    _impl->currentId = 0;
    _impl->lastTimestamp = -1.0;
    _impl->preint.reset();

    graph.add(gtsam::PriorFactor<gtsam::Pose3>(xKey(0), pose, _impl->posePriorNoise));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(vKey(0), velocity, _impl->velocityPriorNoise));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bKey(0), _impl->initialBias, _impl->biasPriorNoise));

    initial.insert(xKey(0), pose);
    initial.insert(vKey(0), velocity);
    initial.insert(bKey(0), _impl->initialBias);

    _impl->isam.update(graph, initial);
    _impl->currentEstimate = _impl->isam.calculateEstimate();
}

void ImuOdometry::integrateImu(const std::vector<ImuSample>& samples)
{
    for (const auto& sample : samples)
    {
        if (_impl->lastTimestamp < 0.0)
        {
            _impl->lastTimestamp = sample.timestamp;
            continue;
        }

        const double dt = sample.timestamp - _impl->lastTimestamp;
        _impl->lastTimestamp = sample.timestamp;

        if (dt <= 0.0 || dt > 0.5)
            continue;

        _impl->ensurePreintegrator();
        _impl->preint->integrateMeasurement(
            gtsam::Vector3(sample.linearAcceleration[0],
                           sample.linearAcceleration[1],
                           sample.linearAcceleration[2]),
            gtsam::Vector3(sample.angularVelocity[0],
                           sample.angularVelocity[1],
                           sample.angularVelocity[2]),
            dt);
    }
}

ImuOdometry::Prediction ImuOdometry::predict() const
{
    const gtsam::Pose3 pose =
        _impl->currentEstimate.at<gtsam::Pose3>(xKey(_impl->currentId));
    const gtsam::Vector3 velocity =
        _impl->currentEstimate.at<gtsam::Vector3>(vKey(_impl->currentId));

    if (!_impl->preint)
        return makePrediction(pose, velocity);

    const gtsam::imuBias::ConstantBias bias =
        _impl->currentEstimate.at<gtsam::imuBias::ConstantBias>(bKey(_impl->currentId));
    const gtsam::NavState prevState(pose, velocity);
    const gtsam::NavState predicted = _impl->preint->predict(prevState, bias);
    return makePrediction(predicted.pose(), predicted.velocity());
}

ImuOdometry::Prediction ImuOdometry::update(const Matrix3x3& icpR,
                                            const std::array<float,3>& icpT,
                                            float icpFitness)
{
    const int prevId = _impl->currentId;
    const int newId = prevId + 1;

    const gtsam::Pose3 previousPose =
        _impl->currentEstimate.at<gtsam::Pose3>(xKey(prevId));
    const gtsam::Vector3 previousVelocity =
        _impl->currentEstimate.at<gtsam::Vector3>(vKey(prevId));
    const gtsam::imuBias::ConstantBias previousBias =
        _impl->currentEstimate.at<gtsam::imuBias::ConstantBias>(bKey(prevId));

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    if (_impl->preint)
    {
        graph.add(gtsam::CombinedImuFactor(
            xKey(prevId), vKey(prevId),
            xKey(newId), vKey(newId),
            bKey(prevId), bKey(newId),
            *_impl->preint));
    }

    const gtsam::Pose3 absoluteIcpPose = toGtsamPose(icpR, icpT);
    const gtsam::Pose3 relativePose = previousPose.between(absoluteIcpPose);
    const double fitness = std::max(0.0, std::min(1.0, static_cast<double>(icpFitness)));
    const double posSigma = 0.05 + (1.0 - fitness) * 0.5;
    const double rotSigma = 0.02 + (1.0 - fitness) * 0.3;

    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        xKey(prevId), xKey(newId), relativePose,
        diagonalNoise(rotSigma, rotSigma, rotSigma, posSigma, posSigma, posSigma)));

    const Prediction predicted = _impl->preint
        ? predict()
        : makePrediction(absoluteIcpPose, previousVelocity);
    const gtsam::Pose3 initialPose = _impl->preint
        ? toGtsamPose(predicted.R, predicted.t)
        : absoluteIcpPose;
    const gtsam::Vector3 initialVelocity(
        predicted.velocity[0], predicted.velocity[1], predicted.velocity[2]);

    initial.insert(xKey(newId), initialPose);
    initial.insert(vKey(newId), initialVelocity);
    initial.insert(bKey(newId), previousBias);

    _impl->isam.update(graph, initial);
    _impl->currentEstimate = _impl->isam.calculateEstimate();
    _impl->currentId = newId;

    return makePrediction(
        _impl->currentEstimate.at<gtsam::Pose3>(xKey(newId)),
        _impl->currentEstimate.at<gtsam::Vector3>(vKey(newId)));
}

void ImuOdometry::resetIntegration()
{
    _impl->preint.reset();
}
