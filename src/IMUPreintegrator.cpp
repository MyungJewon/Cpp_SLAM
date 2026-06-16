#include "IMUPreintegrator.h"
#include <cmath>

static constexpr double CALIBRATION_DURATION_SEC = 1.0;

IMUPreintegrator::IMUPreintegrator()
{
    reset();
}

void IMUPreintegrator::reset()
{
    _R                = {1,0,0, 0,1,0, 0,0,1};
    _lastTime         = -1.0;
    _windowStartTime  = -1.0;
    _count            = 0;
    _accSum           = {0, 0, 0};
    _accCount         = 0;
    _history.clear();
    _history.push_back({0.0, _R});
}

void IMUPreintegrator::calibrate()
{
    _calibrating = true;
    _calibrated = false;
    _gyroBias = {0, 0, 0};
    _calibAccumSum = {0, 0, 0};
    _calibAccumCount = 0;
    _calibStartTime = -1.0;
    reset();
}

void IMUPreintegrator::addSample(const ImuSample& s)
{
    if (_calibrating)
    {
        if (_calibStartTime < 0.0)
            _calibStartTime = s.timestamp;

        _calibAccumSum[0] += s.angularVelocity[0];
        _calibAccumSum[1] += s.angularVelocity[1];
        _calibAccumSum[2] += s.angularVelocity[2];
        ++_calibAccumCount;

        if (s.timestamp - _calibStartTime >= CALIBRATION_DURATION_SEC &&
            _calibAccumCount > 0)
        {
            _gyroBias = {
                _calibAccumSum[0] / _calibAccumCount,
                _calibAccumSum[1] / _calibAccumCount,
                _calibAccumSum[2] / _calibAccumCount
            };
            _calibrating = false;
            _calibrated = true;
            reset();
        }
        return;
    }

    _accSum[0] += s.linearAcceleration[0];
    _accSum[1] += s.linearAcceleration[1];
    _accSum[2] += s.linearAcceleration[2];
    ++_accCount;

    if (_lastTime < 0.0)
    {
        _lastTime = s.timestamp;
        _windowStartTime = s.timestamp;
        return;
    }

    double dt = s.timestamp - _lastTime;
    _lastTime = s.timestamp;

    // dt가 비정상이면 스킵 (센서 초기화 구간, 타임스탬프 역전 등)
    if (dt <= 0.0 || dt > 0.5) return;

    // 바이어스를 빼지 않으면 작은 오프셋도 적분되며 자세 드리프트로 누적됩니다.
    double wx = s.angularVelocity[0] - (_calibrated ? _gyroBias[0] : 0.0);
    double wy = s.angularVelocity[1] - (_calibrated ? _gyroBias[1] : 0.0);
    double wz = s.angularVelocity[2] - (_calibrated ? _gyroBias[2] : 0.0);
    double theta = std::sqrt(wx*wx + wy*wy + wz*wz) * dt;

    double relativeTime = (_windowStartTime >= 0.0)
                            ? s.timestamp - _windowStartTime
                            : 0.0;

    if (theta < 1e-10)
    {
        _history.push_back({relativeTime, _R});
        ++_count;
        return;
    }  // 사실상 정지

    // Rodrigues 공식으로 증분 회전행렬 계산
    // R_delta = I + sin(theta)/theta * [w*dt]x + (1-cos(theta))/theta^2 * [w*dt]x^2
    double ax = wx * dt / theta;
    double ay = wy * dt / theta;
    double az = wz * dt / theta;

    double s_t = std::sin(theta);
    double c_t = std::cos(theta);
    double one_c = 1.0 - c_t;

    // 회전축 (ax, ay, az) 기준 Rodrigues
    Matrix3x3 dR = {
        (float)(c_t + ax*ax*one_c),       (float)(ax*ay*one_c - az*s_t), (float)(ax*az*one_c + ay*s_t),
        (float)(ay*ax*one_c + az*s_t),    (float)(c_t + ay*ay*one_c),    (float)(ay*az*one_c - ax*s_t),
        (float)(az*ax*one_c - ay*s_t),    (float)(az*ay*one_c + ax*s_t), (float)(c_t + az*az*one_c)
    };

    // 누적: R = dR * R
    Matrix3x3 newR = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                newR[i*3+j] += dR[i*3+k] * _R[k*3+j];
    _R = newR;
    _history.push_back({relativeTime, _R});

    ++_count;
}

Matrix3x3 IMUPreintegrator::getRotation() const
{
    return _R;
}

Matrix3x3 IMUPreintegrator::getRotationAt(double relativeTimeSec) const
{
    Matrix3x3 identity = {1,0,0, 0,1,0, 0,0,1};
    if (_history.empty()) return identity;

    size_t bestIdx = 0;
    double bestDt = std::abs(_history[0].t - relativeTimeSec);
    for (size_t i = 1; i < _history.size(); ++i)
    {
        double dt = std::abs(_history[i].t - relativeTimeSec);
        if (dt < bestDt)
        {
            bestDt = dt;
            bestIdx = i;
        }
    }

    // 스캔당 IMU 샘플이 적어 MVP에서는 최근접 회전만으로도 디스큐잉 오차가 작습니다.
    return _history[bestIdx].R;
}

bool IMUPreintegrator::getGravityUp(std::array<float, 3>& outUp) const
{
    if (_accCount == 0) return false;

    double ax = _accSum[0] / _accCount;
    double ay = _accSum[1] / _accCount;
    double az = _accSum[2] / _accCount;
    double norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6) return false;

    // 가속도계는 정지 시 중력 반대 방향("위")으로 1g를 측정
    outUp = { (float)(ax / norm), (float)(ay / norm), (float)(az / norm) };
    return true;
}
