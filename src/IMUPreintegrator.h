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

    // 보정 윈도우(기본 1초) 동안 자이로 바이어스를 추정합니다.
    // SLAM 시작 시 센서가 정지해 있다고 가정 — 측정된 평균 각속도가 곧 바이어스.
    void calibrate();
    bool isCalibrating() const { return _calibrating; }
    bool isCalibrated() const { return _calibrated; }
    std::array<double,3> getGyroBias() const { return _gyroBias; }

    // 적분된 회전행렬을 반환합니다. (ICP 초기값으로 사용)
    Matrix3x3 getRotation() const;

    // 적분 윈도우 내 (상대시간, 그 시점까지의 누적 회전) 기록 — 디스큐잉용
    Matrix3x3 getRotationAt(double relativeTimeSec) const;

    // 이번 프레임 구간의 평균 가속도 방향(중력 기준 "위" 방향, 바디 좌표계)을 반환합니다.
    // 정지/저가속 구간일수록 정확하며, 샘플이 없으면 false를 반환합니다.
    bool getGravityUp(std::array<float, 3>& outUp) const;

    int sampleCount() const { return _count; }

private:
    struct TimedRotation
    {
        double t;
        Matrix3x3 R;
    };

    Matrix3x3 _R;          // 누적 회전
    double    _lastTime;   // 직전 샘플 타임스탬프 (-1 이면 미초기화)
    double    _windowStartTime = -1.0;
    int       _count;
    std::array<double, 3> _accSum = {0, 0, 0};
    int       _accCount = 0;
    std::vector<TimedRotation> _history;

    bool _calibrating = false;
    bool _calibrated = false;
    std::array<double,3> _gyroBias = {0, 0, 0};
    std::array<double,3> _calibAccumSum = {0, 0, 0};
    int _calibAccumCount = 0;
    double _calibStartTime = -1.0;
};
