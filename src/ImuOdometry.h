#pragma once
#include <array>
#include <vector>
#include <memory>
#include "ICP.h"
#include "IMUPreintegrator.h"  // for ImuSample struct

// IMU 프리인테그레이션 + ICP를 같은 factor graph에서 동시에 최적화하는
// 타이트 커플링 odometry. 기존 PoseGraph(루프클로저용)와는 별개 — 이건
// 프레임 간 odometry 자체의 정확도를 높이기 위한 모듈입니다.
class ImuOdometry
{
public:
    // gyroBias: IMUPreintegrator::calibrate()로 추정한 자이로 바이어스
    // gravityMagnitude: 보통 9.81 (m/s^2)
    ImuOdometry(const std::array<double,3>& gyroBias, double gravityMagnitude = 9.81);
    ~ImuOdometry();

    // nav(월드) 좌표계에서의 중력 벡터를 설정 (m/s^2, 보통 -9.81·up).
    // 정지 구간 가속도계로 측정한 중력 방향을 넣으면 preintegration이 중력을
    // 올바르게 제거해 속도 발산을 막는다. init() 호출 전에 설정해야 한다.
    void setNavGravity(const std::array<float,3>& gNav);

    // 첫 프레임의 pose로 초기화 (velocity=0, 추정 바이어스로 시작)
    void init(const Matrix3x3& R0, const std::array<float,3>& t0);

    // 이전 프레임~현재 프레임 사이 IMU 샘플을 프리인테그레이션에 누적
    void integrateImu(const std::vector<ImuSample>& samples);

    // 누적된 IMU 프리인테그레이션으로 다음 pose/velocity를 예측
    // (ICP 초기값 + 디스큐잉용 velocity로 사용)
    struct Prediction {
        Matrix3x3 R;
        std::array<float,3> t;
        std::array<float,3> velocity;  // 월드 좌표계 m/s
    };
    Prediction predict() const;

    // ICP가 추정한 이번 프레임의 absolute pose(scan-to-map ICP 결과, world frame)를
    // LiDAR 측정 제약으로 추가하고, IMU 프리인테그레이션 제약과 함께
    // ISAM2로 최적화. icpFitness가 낮으면 LiDAR 제약의 신뢰도를 낮춤.
    // 반환값: 최적화된 현재 프레임의 pose/velocity
    Prediction update(const Matrix3x3& icpR, const std::array<float,3>& icpT,
                       float icpFitness);

    // 다음 프레임을 위해 프리인테그레이션 버퍼를 리셋 (update 이후 호출)
    void resetIntegration();

private:
    struct Impl;
    Impl* _impl;
};
