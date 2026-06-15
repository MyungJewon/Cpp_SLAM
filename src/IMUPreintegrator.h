#pragma once
#include <array>
#include <vector>
#include "ICP.h"

struct ImuSample
{
    double timestamp;                        // 초 단위
    std::array<double, 3> angularVelocity;   // rad/s  (x, y, z)
    std::array<double, 3> linearAcceleration;// m/s^2  (x, y, z)
};

// 두 라이다 프레임 사이 IMU 샘플을 적분해 회전을 예측합니다.
// 가속도 적분은 중력 제거가 필요해 오차가 크므로 회전만 제공합니다.
// 위치(translation)는 ICP가 담당합니다.
class IMUPreintegrator
{
public:
    IMUPreintegrator();

    // 이전 프레임 처리 후 상태를 초기화합니다.
    void reset();

    // IMU 샘플 하나를 추가합니다.
    void addSample(const ImuSample& sample);

    // 적분된 회전행렬을 반환합니다. (ICP 초기값으로 사용)
    Matrix3x3 getRotation() const;

    int sampleCount() const { return _count; }

private:
    Matrix3x3 _R;          // 누적 회전
    double    _lastTime;   // 직전 샘플 타임스탬프 (-1 이면 미초기화)
    int       _count;
};
