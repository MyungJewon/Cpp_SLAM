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

static float dotVec(const std::array<float, 3>& a,
                    const std::array<float, 3>& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static std::array<float, 3> crossVec(const std::array<float, 3>& a,
                                     const std::array<float, 3>& b)
{
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

static bool normalizeVec(std::array<float, 3>& v)
{
    float n = std::sqrt(dotVec(v, v));
    if (n < 1e-6f) return false;
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return true;
}

static Matrix3x3 rodriguesAxisAngle(const std::array<float, 3>& axis,
                                    float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);
    float oneMinusC = 1.0f - c;
    float x = axis[0], y = axis[1], z = axis[2];

    return {
        c + x*x*oneMinusC,     x*y*oneMinusC - z*s, x*z*oneMinusC + y*s,
        y*x*oneMinusC + z*s,   c + y*y*oneMinusC,   y*z*oneMinusC - x*s,
        z*x*oneMinusC - y*s,   z*y*oneMinusC + x*s, c + z*z*oneMinusC
    };
}

[[maybe_unused]] static Matrix3x3 limitedAlignmentRotation(std::array<float, 3> from,
                                          std::array<float, 3> to,
                                          float maxAngleRad)
{
    Matrix3x3 I = {1,0,0, 0,1,0, 0,0,1};
    if (!normalizeVec(from) || !normalizeVec(to)) return I;

    float c = std::max(-1.0f, std::min(1.0f, dotVec(from, to)));
    float angle = std::acos(c);
    if (angle < 1e-5f) return I;

    std::array<float, 3> axis = crossVec(from, to);
    if (!normalizeVec(axis)) return I;

    angle = std::min(angle, maxAngleRad);
    return rodriguesAxisAngle(axis, angle);
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
    // 프레임 상태 초기화 — 조기 return(품질부족/게이팅) 경로는 accepted=false 유지.
    // back-end가 이 상태로 odometry factor 노이즈를 적응시킨다 (스킵 프레임 = IMU 브릿지).
    _lastFitness = 0.0f;
    _lastAccepted = false;
    _lastStationary = false;

    const std::vector<std::array<float, 3>>* filterInput = &rawPoints;
    std::vector<std::array<float, 3>> deskewed;

    if (_imu && pointTimes && pointTimes->size() == rawPoints.size() && !pointTimes->empty())
    {
        deskewed = rawPoints;
        double scanDuration = (double)*std::max_element(pointTimes->begin(), pointTimes->end());
        Matrix3x3 REnd = _imu->getRotationAt(scanDuration);
        Matrix3x3 REndInv = transposeMat(REnd);
        for (size_t i = 0; i < rawPoints.size(); ++i)
        {
            Matrix3x3 RAtTi = _imu->getRotationAt((double)(*pointTimes)[i]);
            // 점이 찍힌 시점의 자세를 스캔 종료 시점 자세로 맞춰 회전 왜곡을 줄입니다.
            // 회전 deskew는 자이로 기반이라 신뢰 가능. translational deskew는 IMU velocity가
            // 발산하면 점을 수 미터씩 밀어 점군을 망가뜨리므로(정합 fitness 붕괴) 사용하지 않는다.
            Matrix3x3 RRel = multiplyMat(REndInv, RAtTi);
            deskewed[i] = multiplyVec(RRel, rawPoints[i]);
        }
        filterInput = &deskewed;
    }

    auto currentFrame = voxelGridFilter(*filterInput, _voxelSize);

    if (_isFirst)
    {
        _lastFrame = currentFrame;
        _localMap.insertAndUpdate(*filterInput, _position);
        _trajectory.push_back(_position);
        _isFirst = false;
        std::cout << "[Odometry] 첫 프레임 저장 (" << currentFrame.size() << "개 점)" << std::endl;
        return;
    }

    const std::array<float, 3> prevPosition = _position;
    const Matrix3x3 prevRotation = _rotation;

    Matrix3x3 predictedR = {};
    std::array<float, 3> predictedT = {};
    const Matrix3x3* initialR = nullptr;
    const std::array<float, 3>* initialT = nullptr;

    if (_hasPrevDelta)
    {
        predictedT = {
            _position[0] + _lastDeltaT[0],
            _position[1] + _lastDeltaT[1],
            _position[2] + _lastDeltaT[2]
        };
        predictedR = multiplyMat(_lastDeltaR, _rotation);
    }
    else
    {
        predictedT = _position;
        predictedR = _rotation;
    }
    initialR = &predictedR;
    initialT = &predictedT;

    // IMU 상태를 먼저 꺼내고 즉시 reset — ICP 성공·실패에 무관하게 누출 없음
    std::array<float, 3> gravityUp = {0.0f, 0.0f, 1.0f};
    bool hasGravity = false;

    if (_imuOdom)
    {
        if (_imu)
        {
            hasGravity = _imu->getGravityUp(gravityUp);
            _imu->reset();
        }
    }
    else if (_imu)
    {
        hasGravity = _imu->getGravityUp(gravityUp);
        _imu->reset();
    }

    // back-end 자세 factor용으로 이번 프레임 중력을 보존 (조기 return에도 유효)
    _lastGravityUp = gravityUp;
    _lastHasGravity = hasGravity;

    std::vector<std::array<float, 3>> localMapVec;
    localMapVec.reserve(_localMap.cells().size());
    for (const auto& kv : _localMap.cells())
        localMapVec.push_back(kv.second.center);
    const std::array<float, 3>* gravityArg =
        (hasGravity && _gravityScale > 0.0f) ? &gravityUp : nullptr;
    ICPResult icp = runICP(currentFrame, localMapVec, 20, 1e-4f,
                           initialR, initialT, false, true,
                           &_localMap.cells(), _voxelSize,
                           gravityArg, _gravityScale);
    if (icp.fitness <= 0.0f && !localMapVec.empty())
    {
        icp = runICP(currentFrame, localMapVec, 20, 1e-4f,
                     initialR, initialT, false, false);
    }

    if (!std::isfinite(icp.error) || icp.fitness < 0.08f ||
        (icp.error > _voxelSize * 0.6f && icp.fitness < 0.35f))
    {
        std::cout << "[Odometry] 경고: 프레임 " << _trajectory.size()+1
                  << " ICP 품질 부족 (fitness=" << icp.fitness
                  << ", error=" << icp.error << ") 포즈 유지" << std::endl;
        if (_imuOdom) _imuOdom->resetIntegration();
        _trajectory.push_back(_position);
        _lastFrame = currentFrame;
        _hasPrevDelta = false;
        return;
    }

    // NaN/Inf 결과 거부 — degenerate 프레임에서 ICP가 발산하면 포즈그래프(GTSAM)가
    // 터지므로, 비유한값이면 이번 프레임을 버리고 직전 포즈 유지.
    bool icpFinite = std::isfinite(icp.t[0]) && std::isfinite(icp.t[1]) && std::isfinite(icp.t[2]);
    for (int i = 0; i < 9 && icpFinite; ++i)
        icpFinite = std::isfinite(icp.R[i]);
    if (!icpFinite)
    {
        std::cout << "[Odometry] 경고: 프레임 " << _trajectory.size()+1
                  << " ICP 결과 NaN/Inf — 프레임 무시" << std::endl;
        if (_imuOdom) _imuOdom->resetIntegration();
        _trajectory.push_back(_position);
        _lastFrame = currentFrame;
        _hasPrevDelta = false;
        return;
    }

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
        _hasPrevDelta = false;  // 잘못된 예측 delta 리셋
        return;
    }

    // Constant velocity 예측 대비 ICP 결과 괴리 검증 (innovation gating)
    // 정지 상태(predStepLen < 0.1m)에서는 재출발 첫 이동을 막지 않도록 게이팅 생략
    if (_hasPrevDelta)
    {
        float gx = icp.t[0] - predictedT[0];
        float gy = icp.t[1] - predictedT[1];
        float gz = icp.t[2] - predictedT[2];
        float gatingDist = std::sqrt(gx*gx + gy*gy + gz*gz);
        float predStepLen = std::sqrt(_lastDeltaT[0]*_lastDeltaT[0]
                                    + _lastDeltaT[1]*_lastDeltaT[1]
                                    + _lastDeltaT[2]*_lastDeltaT[2]);
        // 실제로 이동 중일 때만 게이팅 적용 (정지→재출발 구간 제외)
        if (predStepLen > 0.1f)
        {
            if (gatingDist > std::max(2.0f, predStepLen * 3.0f) && icp.fitness < 0.15f)
            {
                std::cout << "[Odometry] 경고: 프레임 " << _trajectory.size()+1
                          << " ICP 결과 예측 괴리 (gating=" << gatingDist
                          << "m, predStep=" << predStepLen
                          << "m, fitness=" << icp.fitness << ") delta 리셋" << std::endl;
                if (_imuOdom) _imuOdom->resetIntegration();
                _trajectory.push_back(_position);
                _lastFrame = currentFrame;
                _hasPrevDelta = false;
                return;
            }
        }
    }

    _lastDeltaT = {
        icp.t[0] - prevPosition[0],
        icp.t[1] - prevPosition[1],
        icp.t[2] - prevPosition[2]
    };
    _lastDeltaR = multiplyMat(icp.R, transposeMat(prevRotation));
    _hasPrevDelta = true;

    // 포즈와 로컬 맵을 갱신하지 않아 노이즈 누적을 방지합니다.
    float angle = std::acos(std::max(-1.0f, std::min(1.0f,
        (icp.R[0] + icp.R[4] + icp.R[8] - 1.0f) * 0.5f))) * (180.0f / 3.14159265f);
    if (stepDist < _stationaryStepM && angle < _stationaryAngleDeg)
    {
        if (_imuOdom) _imuOdom->resetIntegration();
        std::vector<std::array<float, 3>> worldFrameStationary;
        worldFrameStationary.reserve(filterInput->size());
        for (const auto& p : *filterInput) {
            worldFrameStationary.push_back({
                _rotation[0]*p[0]+_rotation[1]*p[1]+_rotation[2]*p[2]+_position[0],
                _rotation[3]*p[0]+_rotation[4]*p[1]+_rotation[5]*p[2]+_position[1],
                _rotation[6]*p[0]+_rotation[7]*p[1]+_rotation[8]*p[2]+_position[2]
            });
        }
        _localMap.insertAndUpdate(worldFrameStationary, _position);
        _localMap.slideWindow(_position, 20.0f);
        _trajectory.push_back(_position);
        _lastFrame = currentFrame;
        _hasPrevDelta = false;  // 정지 후 재출발 시 stale velocity 예측 방지
        _lastFitness = icp.fitness;
        _lastAccepted = true;
        _lastStationary = true;  // ZUPT(정지 시 속도=0 factor)용
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

    // 중력 보정은 이제 GICP 내부 soft constraint(L2)로 처리한다.
    // 후처리 snap은 LiDAR 신뢰도를 무시하고 매 프레임 강제로 당겨 이중 보정/충돌을
    // 일으키므로 제거했다. (in-solver 방식이 LiDAR가 약한 축에서만 중력이 지배)

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

    std::vector<std::array<float, 3>> worldFrame;
    worldFrame.reserve(filterInput->size());
    for (const auto& p : *filterInput) {
        std::array<float, 3> pWorld = {
            _rotation[0]*p[0]+_rotation[1]*p[1]+_rotation[2]*p[2]+_position[0],
            _rotation[3]*p[0]+_rotation[4]*p[1]+_rotation[5]*p[2]+_position[1],
            _rotation[6]*p[0]+_rotation[7]*p[1]+_rotation[8]*p[2]+_position[2]
        };
        worldFrame.push_back(pWorld);
    }
    _localMap.insertAndUpdate(worldFrame, _position);
    _localMap.slideWindow(_position, 20.0f);

    _lastFrame = currentFrame;
    _trajectory.push_back(_position);
    _lastFitness = icp.fitness;
    _lastAccepted = true;
    _lastStationary = false;

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
