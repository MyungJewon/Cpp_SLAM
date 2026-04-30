// LiDAR Odometry 선언: 연속 프레임 간 ICP로 누적 위치를 추적합니다.
#pragma once
#include <array>
#include <vector>
#include "ICP.h"
#include "VoxelGrid.h"

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

    // 지면 구속 모드: z=0 고정, Yaw(수평 회전)만 추적
    // 평지 주행 데이터에서 z 드리프트를 제거합니다
    void setGroundMode(bool enabled) { _groundMode = enabled; }

private:
    float     _voxelSize;
    bool      _isFirst;
    bool      _groundMode = false;

    Matrix3x3                            _rotation;    // 누적 회전
    std::array<float, 3>                 _position;    // 누적 위치
    std::vector<std::array<float, 3>>    _trajectory;  // 경로 기록
    std::vector<std::array<float, 3>>    _prevFrame;   // 직전 프레임
};