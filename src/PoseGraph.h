#pragma once
#include <vector>
#include <array>
#include "ICP.h"  // for Matrix3x3
#include "IMUPreintegrator.h"  // for ImuSample

// Pose = rotation (Matrix3x3 row-major) + translation
struct Pose3D {
    Matrix3x3            R;
    std::array<float, 3> t;
};

class PoseGraph {
public:
    PoseGraph();
    ~PoseGraph();

    void init(const Pose3D& firstPose);

    // IMU 타이트커플링 활성화 (init 전에 호출). LIO-SAM식 단일 그래프로
    // X,V,B 노드 + CombinedImuFactor를 GICP odometry/루프와 같은 그래프에서 최적화한다.
    //   gyroBias    : 정지 캘리브레이션 자이로 바이어스
    //   gravityMag  : 중력 크기 (보통 9.81)
    //   navGravity  : nav(월드) 좌표 중력 벡터 (보통 -9.81·up). 시작 정지 가속도계로 측정.
    void enableImu(const std::array<double, 3>& gyroBias, double gravityMag,
                   const std::array<float, 3>& navGravity);

    // 직전 노드~현재 프레임 사이 IMU 샘플을 preintegration에 누적 (addOdometry 전에 호출)
    void integrateImu(const std::vector<ImuSample>& samples);

    // deltaPose: 직전 노드 대비 상대 변환 (GICP scan-to-map 측정값).
    // gravityBodyUp: 이 노드의 IMU 중력(바디 좌표). nullptr이면 자세 factor 미추가.
    // lidarFitness: GICP fitness(0~1). >=0이면 odometry 노이즈를 적응 조절 —
    //   fitness 높음 → 타이트(LiDAR 신뢰, IMU 물러남), 낮음/스킵(0) → 루즈(IMU 브릿지).
    //   -1이면 기존 고정 노이즈.
    // stationary: true면(정지 감지) IMU 활성 시 속도=0 prior(ZUPT) 추가 —
    //   handheld 정지 구간에서 속도/바이어스 드리프트를 리셋.
    // IMU가 활성화되어 있으면 누적된 preintegration으로 CombinedImuFactor도 함께 추가한다.
    Pose3D addOdometry(const Pose3D& deltaPose,
                       const std::array<float, 3>* gravityBodyUp = nullptr,
                       float lidarFitness = -1.0f,
                       bool stationary = false);
    // confidence: 정합 신뢰도(0~1, 보통 overlap×fitness). 높을수록 루프를 강하게 구속한다.
    // GLIM의 Hessian 기반 정보행렬 가중을 근사 — 약한 루프는 영향이 작아진다.
    std::vector<Pose3D> addLoopClosure(int fromId, int toId, const Pose3D& relativePose,
                                       float confidence = 1.0f);

    // 자세 factor 노이즈 (rad). 작을수록 중력 정렬을 강하게 강제.
    void setAttitudeSigma(float rad) { _attitudeSigma = rad; }
    int getCurrentId() const { return _currentId; }
    Pose3D getPose(int id) const;
    std::vector<Pose3D> getAllPoses() const;

private:
    struct Impl;
    Impl* _impl;
    int _currentId = 0;
    float _attitudeSigma = 0.1f;  // 자세 factor 노이즈 (rad, ~5.7°)
};
