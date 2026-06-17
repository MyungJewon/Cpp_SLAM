// LiDAR Odometry 선언: 연속 프레임 간 ICP로 누적 위치를 추적합니다.
#pragma once
#include <array>
#include <vector>
#include "ICP.h"
#include "VoxelGrid.h"
#include "VoxelMap.h"
#include "IMUPreintegrator.h"
#include "ImuOdometry.h"

class Odometry
{
public:
    Odometry(float voxelSize = 0.3f);

    // 새 프레임을 입력받아 이전 프레임과 ICP 실행 후 누적 위치 갱신
    // 첫 프레임은 기준으로 저장만 하고 위치는 갱신하지 않음
    void addFrame(const std::vector<std::array<float, 3>>& rawPoints,
                  const std::vector<float>* pointTimes = nullptr);

    // 현재 누적 위치 반환 (x, y, z)
    std::array<float, 3> getPosition() const;

    // 현재 누적 회전행렬 반환
    Matrix3x3 getRotation() const;

    // 지금까지 누적된 경로 반환 (프레임마다 위치 하나)
    const std::vector<std::array<float, 3>>& getTrajectory() const;

    // 루프 클로저 보정 후 경로/위치를 외부에서 교체할 때 사용
    void setTrajectory(const std::vector<std::array<float, 3>>& traj);
    void setPosition(const std::array<float, 3>& pos);

    // addFrame()에서 다운샘플링한 프레임 반환 — 외부에서 재활용해 중복 필터링 방지
    const std::vector<std::array<float, 3>>& getLastFrame() const { return _lastFrame; }

    // 마지막 프레임에서 추정한 중력 "위"(바디 좌표) 반환 — back-end 자세 factor용.
    // 유효하면 true. (IMU 샘플 있을 때만 유효)
    bool getLastGravityUp(std::array<float, 3>& out) const
    {
        out = _lastGravityUp;
        return _lastHasGravity;
    }

    // 지면 구속 모드: z=0 고정, Yaw(수평 회전)만 추적
    void setGroundMode(bool enabled) { _groundMode = enabled; }

    // 프레임당 최대 허용 이동 거리 (기본 2.0m) — 초과 시 ICP 실패로 무시
    void setMaxStepDist(float m) { _maxStepDist = m; }

    // ICP 대상 로컬 맵 최대 점 개수 (기본 5000) — 클수록 정확하지만 느림
    void setLocalMapMaxPts(int n) { _localMapMaxPts = n; }

    // 정지 감지 임계값 설정
    // stepM  : 이동량(m) 이하면 정지로 판단 (기본 0.03m)
    // angleDeg: 회전량(도) 이하면 정지로 판단 (기본 0.5도)
    void setStationaryThresh(float stepM, float angleDeg)
    {
        _stationaryStepM   = stepM;
        _stationaryAngleDeg = angleDeg;
    }

    // IMU 프리인테그레이터 연결 (nullptr이면 IMU 미사용)
    // addFrame() 내부에서 ICP 초기값으로 활용하고 자동으로 reset() 호출
    void setImuPreintegrator(IMUPreintegrator* imu) { _imu = imu; }
    void setImuOdometry(ImuOdometry* imuOdom) { _imuOdom = imuOdom; }

    // 중력 prior(L2) 가중치. GICP 정규방정식의 회전 블록 평균 대각 대비 비율.
    // 0이면 비활성. 중력 방향(IMU 가속도)으로 roll/pitch를 soft constraint.
    void setGravityScale(float s) { _gravityScale = s; }

private:
    float     _voxelSize;
    bool      _isFirst;
    bool      _groundMode    = false;
    float     _maxStepDist   = 5.0f;   // 프레임당 최대 이동 거리 (m) — 초과 시 ICP 실패로 간주
    IMUPreintegrator* _imu   = nullptr;
    ImuOdometry* _imuOdom    = nullptr;

    Matrix3x3                            _rotation;    // 누적 회전
    std::array<float, 3>                 _position;    // 누적 위치
    Matrix3x3                            _lastDeltaR;
    std::array<float, 3>                 _lastDeltaT = {0.f, 0.f, 0.f};
    bool                                 _hasPrevDelta = false;
    std::vector<std::array<float, 3>>    _trajectory;  // 경로 기록
    std::vector<std::array<float, 3>>    _lastFrame;   // 마지막 다운샘플링 프레임
    VoxelMap                             _localMap;          // 월드 좌표계 로컬 맵
    int                                  _localMapMaxPts     = 5000; // KD-Tree 대상 상한

    float _stationaryStepM    = 0.03f;  // 정지 판단 이동 임계값 (m)
    float _stationaryAngleDeg = 0.5f;   // 정지 판단 회전 임계값 (도)
    float _gravityScale       = 0.0f;   // 중력 prior 가중치 (0=비활성). 게이팅 재설계 전까지 끔.
    std::array<float, 3> _lastGravityUp = {0.0f, 0.0f, 1.0f};  // 마지막 프레임 중력(바디)
    bool _lastHasGravity = false;
};
