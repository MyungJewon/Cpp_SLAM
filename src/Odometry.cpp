#include "Odometry.h"
#include <iostream>
#include <cmath>
#include <algorithm>

// ICP 회전 누적 시 발생하는 부동소수점 오차로 행렬이 직교성을 잃는 것을 방지
// (groundMode가 꺼져 있으면 이 오차가 보정되지 않고 누적되어 GTSAM 최적화가 발산함)
static Matrix3x3 orthonormalize(const Matrix3x3& R)
{
    float c0[3] = {R[0], R[3], R[6]};
    float c1[3] = {R[1], R[4], R[7]};

    float n0 = std::sqrt(c0[0]*c0[0] + c0[1]*c0[1] + c0[2]*c0[2]);
    for (int i = 0; i < 3; ++i) c0[i] /= n0;

    float dot01 = c0[0]*c1[0] + c0[1]*c1[1] + c0[2]*c1[2];
    for (int i = 0; i < 3; ++i) c1[i] -= dot01 * c0[i];
    float n1 = std::sqrt(c1[0]*c1[0] + c1[1]*c1[1] + c1[2]*c1[2]);
    for (int i = 0; i < 3; ++i) c1[i] /= n1;

    float c2[3] = {
        c0[1]*c1[2] - c0[2]*c1[1],
        c0[2]*c1[0] - c0[0]*c1[2],
        c0[0]*c1[1] - c0[1]*c1[0]
    };

    return {
        c0[0], c1[0], c2[0],
        c0[1], c1[1], c2[1],
        c0[2], c1[2], c2[2]
    };
}

static Matrix3x3 multiplyMat(const Matrix3x3& A, const Matrix3x3& B)
{
    Matrix3x3 C = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C[i*3+j] += A[i*3+k] * B[k*3+j];
    return C;
}

static Matrix3x3 transposeMat(const Matrix3x3& R)
{
    return {
        R[0], R[3], R[6],
        R[1], R[4], R[7],
        R[2], R[5], R[8]
    };
}

static std::array<float, 3> multiplyVec(const Matrix3x3& R,
                                        const std::array<float, 3>& v)
{
    return {
        R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
        R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
        R[6]*v[0] + R[7]*v[1] + R[8]*v[2]
    };
}

Odometry::Odometry(float voxelSize)
    : _voxelSize(voxelSize)
    , _isFirst(true)
    , _rotation({1,0,0, 0,1,0, 0,0,1})
    , _position({0, 0, 0})
    , _localMap(_voxelSize)
{
}

void Odometry::addFrame(const std::vector<std::array<float, 3>>& rawPoints,
                        const std::vector<float>* pointTimes)
{
    const std::vector<std::array<float, 3>>* filterInput = &rawPoints;
    std::vector<std::array<float, 3>> deskewed;

    if (_imu && pointTimes && pointTimes->size() == rawPoints.size() && !pointTimes->empty())
    {
        deskewed = rawPoints;
        double scanDuration = (double)*std::max_element(pointTimes->begin(), pointTimes->end());
        Matrix3x3 REnd = _imu->getRotationAt(scanDuration);
        Matrix3x3 REndInv = transposeMat(REnd);
        std::array<float, 3> bodyVelocity = {0.0f, 0.0f, 0.0f};

        if (_imuOdom)
        {
            auto pred = _imuOdom->predict();
            bodyVelocity = multiplyVec(transposeMat(pred.R), pred.velocity);
        }

        for (size_t i = 0; i < rawPoints.size(); ++i)
        {
            Matrix3x3 RAtTi = _imu->getRotationAt((double)(*pointTimes)[i]);
            // 점이 찍힌 시점의 자세를 스캔 종료 시점 자세로 맞춰 회전 왜곡을 줄입니다.
            Matrix3x3 RRel = multiplyMat(REndInv, RAtTi);
            deskewed[i] = multiplyVec(RRel, rawPoints[i]);
            if (_imuOdom)
            {
                const float dtToScanEnd = static_cast<float>(scanDuration - (double)(*pointTimes)[i]);
                deskewed[i][0] -= bodyVelocity[0] * dtToScanEnd;
                deskewed[i][1] -= bodyVelocity[1] * dtToScanEnd;
                deskewed[i][2] -= bodyVelocity[2] * dtToScanEnd;
            }
        }
        filterInput = &deskewed;
    }

    auto currentFrame = voxelGridFilter(*filterInput, _voxelSize);

    if (_isFirst)
    {
        _lastFrame = currentFrame;
        _localMap.insert(currentFrame);
        _trajectory.push_back(_position);
        _isFirst = false;
        std::cout << "[Odometry] 첫 프레임 저장 (" << currentFrame.size() << "개 점)" << std::endl;
        return;
    }

    // IMU 상태를 먼저 꺼내고 즉시 reset — ICP 성공·실패에 무관하게 누출 없음
    Matrix3x3 initR = _rotation;
    std::array<float, 3> initT = _position;

    std::array<float, 3> gravityUp = {0.0f, 0.0f, 1.0f};
    bool hasGravity = false;

    if (_imuOdom)
    {
        auto pred = _imuOdom->predict();
        initR = pred.R;
        initT = pred.t;

        if (_imu)
        {
            hasGravity = _imu->getGravityUp(gravityUp);
            _imu->reset();
        }
    }
    else if (_imu)
    {
        bool hasSamples = (_imu->sampleCount() > 0);
        Matrix3x3 imuR  = _imu->getRotation();
        hasGravity = _imu->getGravityUp(gravityUp);
        _imu->reset();

        if (hasSamples)
        {
            initR = multiplyMat(imuR, _rotation);
        }
    }

    auto localMapVec = _localMap.toVector();
    ICPResult icp = runICP(currentFrame, localMapVec, 20, 1e-4f, &initR, &initT, true);

    float dx = icp.t[0] - _position[0];
    float dy = icp.t[1] - _position[1];
    float dz = icp.t[2] - _position[2];
    float stepDist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (stepDist > _maxStepDist)
    {
        std::cout << "[Odometry] 경고: 프레임 " << _trajectory.size()+1
                  << " ICP 점프 무시 (step=" << stepDist << "m > " << _maxStepDist << "m)" << std::endl;
        // update()를 호출하지 않더라도 프리인테그레이션 버퍼는 비워야 함.
        // 안 비우면 다음 프레임에서도 계속 누적되어 예측이 갈수록 더 길게
        // 외삽되고, 그 오차가 기하급수적으로 커져 영원히 복구되지 않음.
        if (_imuOdom) _imuOdom->resetIntegration();
        _trajectory.push_back(_position);
        _lastFrame = currentFrame;
        return;
    }

    // 포즈와 로컬 맵을 갱신하지 않아 노이즈 누적을 방지합니다.
    float angle = std::acos(std::max(-1.0f, std::min(1.0f,
        (icp.R[0] + icp.R[4] + icp.R[8] - 1.0f) * 0.5f))) * (180.0f / 3.14159265f);
    if (stepDist < _stationaryStepM && angle < _stationaryAngleDeg)
    {
        if (_imuOdom) _imuOdom->resetIntegration();
        _trajectory.push_back(_position);
        _lastFrame = currentFrame;
        return;
    }

    if (_imuOdom)
    {
        auto opt = _imuOdom->update(icp.R, icp.t, icp.fitness);
        _position = opt.t;
        _rotation = orthonormalize(opt.R);
        _imuOdom->resetIntegration();
    }
    else
    {
        _position = icp.t;
        _rotation = orthonormalize(icp.R);
    }
    (void)hasGravity;
    (void)gravityUp;

    // 평지 주행 데이터에서 ICP의 z축 오차 누적을 방지합니다.
    if (_groundMode)
    {
        _position[2] = 0.0f;

        float yaw = std::atan2(_rotation[3], _rotation[0]);
        float c = std::cos(yaw), s = std::sin(yaw);
        _rotation = { c, -s, 0.0f,
                      s,  c, 0.0f,
                      0.0f, 0.0f, 1.0f };
    }

    for (const auto& p : currentFrame) {
        std::array<float, 3> pWorld = {
            _rotation[0]*p[0]+_rotation[1]*p[1]+_rotation[2]*p[2]+_position[0],
            _rotation[3]*p[0]+_rotation[4]*p[1]+_rotation[5]*p[2]+_position[1],
            _rotation[6]*p[0]+_rotation[7]*p[1]+_rotation[8]*p[2]+_position[2]
        };
        _localMap.insert(pWorld);
    }
    _localMap.trimToMax(_localMapMaxPts);

    _lastFrame = currentFrame;
    _trajectory.push_back(_position);

    std::cout << "[Odometry] 프레임 " << _trajectory.size()
              << " | 위치: (" << _position[0] << ", "
                              << _position[1] << ", "
                              << _position[2] << ")"
              << " | ICP 반복: " << icp.iterations
              << " | 오차: "     << icp.error
              << std::endl;
}

std::array<float, 3> Odometry::getPosition() const
{
    return _position;
}

Matrix3x3 Odometry::getRotation() const
{
    return _rotation;
}

const std::vector<std::array<float, 3>>& Odometry::getTrajectory() const
{
    return _trajectory;
}

void Odometry::setTrajectory(const std::vector<std::array<float, 3>>& traj)
{
    _trajectory = traj;
}

void Odometry::setPosition(const std::array<float, 3>& pos)
{
    _position = pos;
}
