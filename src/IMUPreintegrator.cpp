#include "IMUPreintegrator.h"
#include <cmath>

IMUPreintegrator::IMUPreintegrator()
{
    reset();
}

void IMUPreintegrator::reset()
{
    _R        = {1,0,0, 0,1,0, 0,0,1};
    _lastTime = -1.0;
    _count    = 0;
}

void IMUPreintegrator::addSample(const ImuSample& s)
{
    if (_lastTime < 0.0)
    {
        _lastTime = s.timestamp;
        return;
    }

    double dt = s.timestamp - _lastTime;
    _lastTime = s.timestamp;

    // dt가 비정상이면 스킵 (센서 초기화 구간, 타임스탬프 역전 등)
    if (dt <= 0.0 || dt > 0.5) return;

    // 각속도 벡터
    double wx = s.angularVelocity[0];
    double wy = s.angularVelocity[1];
    double wz = s.angularVelocity[2];
    double theta = std::sqrt(wx*wx + wy*wy + wz*wz) * dt;

    if (theta < 1e-10) { ++_count; return; }  // 사실상 정지

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

    ++_count;
}

Matrix3x3 IMUPreintegrator::getRotation() const
{
    return _R;
}
