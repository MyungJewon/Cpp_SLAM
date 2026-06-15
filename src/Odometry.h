// LiDAR Odometry 선언: 연속 프레임 간 ICP로 누적 위치를 추적합니다.
#pragma once
#include <array>
#include <vector>
#include "ICP.h"
#include "VoxelGrid.h"
#include "IMUPreintegrator.h"

class Odometry
{
public:
    Odometry(float voxelSize = 0.3f);

    // 새 프레임을 입력받아 이전 프레임과 ICP 실행 후 누적 위치 갱신
    // 첫 프레임은 기준으로 저장만 하고 위치는 갱신하지 않음
    void addFrame(const std::vector<std::array<float, 3>>& rawPoints);

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

    // 지면 구속 모드: z=0 고정, Yaw(수평 회전)만 추적
    void setGroundMode(bool enabled) { _groundMode = enabled; }

    // 프레임당 최대 허용 이동 거리 (기본 2.0m) — 초과 시 ICP 실패로 무시
    void setMaxStepDist(float m) { _maxStepDist = m; }

    // ICP 대상 로컬 맵 최대 점 개수 (기본 5000) — 클수록 정확하지만 느림
    void setLocalMapMaxPts(int n) { _localMapMaxPts = n; }

    // IMU 프리인테그레이터 연결 (nullptr이면 IMU 미사용)
    // addFrame() 내부에서 ICP 초기값으로 활용하고 자동으로 reset() 호출
    void setImuPreintegrator(IMUPreintegrator* imu) { _imu = imu; }

private:
    float     _voxelSize;
    bool      _isFirst;
    bool      _groundMode    = false;
    float     _maxStepDist   = 2.0f;   // 프레임당 최대 이동 거리 (m) — 초과 시 ICP 실패로 간주
    IMUPreintegrator* _imu   = nullptr;

    Matrix3x3                            _rotation;    // 누적 회전
    std::array<float, 3>                 _position;    // 누적 위치
    std::vector<std::array<float, 3>>    _trajectory;  // 경로 기록
    std::vector<std::array<float, 3>>    _lastFrame;   // 마지막 다운샘플링 프레임
    std::vector<std::array<float, 3>>    _localMap;          // 월드 좌표계 로컬 맵
    int                                  _localMapFrameCount = 0;
    int                                  _localMapMaxPts     = 5000; // KD-Tree 대상 상한
};
