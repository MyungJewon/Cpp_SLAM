// LiDAR Odometry 구현: 연속 프레임 간 ICP로 누적 위치를 추적합니다.
#include "Odometry.h"
#include <iostream>
#include <cmath>

Odometry::Odometry(float voxelSize)
    : _voxelSize(voxelSize)
    , _isFirst(true)
    , _rotation({1,0,0, 0,1,0, 0,0,1})  // 단위행렬 (아무 회전 없음)
    , _position({0, 0, 0})               // 원점에서 시작
{
}

void Odometry::addFrame(const std::vector<std::array<float, 3>>& rawPoints)
{
    // 1. 새 프레임 다운샘플링
    auto frame = voxelGridFilter(rawPoints, _voxelSize);

    // 2. 첫 프레임이면 기준으로 저장만 하고 종료
    if (_isFirst)
    {
        _prevFrame = frame;
        _trajectory.push_back(_position);
        _isFirst = false;
        std::cout << "[Odometry] 첫 프레임 저장 (" << frame.size() << "개 점)" << std::endl;
        return;
    }

    // 3. 직전 프레임과 ICP 실행
    // 현재 프레임(src)을 직전 프레임(dst)에 맞추면
    // 그 변환이 곧 이번 프레임에서의 이동량
    ICPResult icp = runICP(frame, _prevFrame);

    // 4. 누적 위치 갱신
    // ICP는 "현재 프레임을 이전 프레임으로 되돌리는 변환"을 반환해요.
    // 실제 이동 방향은 그 반대이므로 부호를 반전(-=)해서 누적해요.
    _position[0] -= _rotation[0]*icp.t[0] + _rotation[1]*icp.t[1] + _rotation[2]*icp.t[2];
    _position[1] -= _rotation[3]*icp.t[0] + _rotation[4]*icp.t[1] + _rotation[5]*icp.t[2];
    _position[2] -= _rotation[6]*icp.t[0] + _rotation[7]*icp.t[1] + _rotation[8]*icp.t[2];

    // 5. 누적 회전 갱신 (이번 회전을 기존 회전에 합산)
    Matrix3x3 newR = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                newR[i*3+j] += icp.R[i*3+k] * _rotation[k*3+j];
    _rotation = newR;

    // 5-b. 지면 구속 모드: z 고정 + Roll/Pitch 제거
    // 평지 주행 데이터에서 ICP의 z축 오차 누적을 방지합니다.
    // 회전행렬에서 Yaw(수평 회전각)만 남기고 재구성해요.
    if (_groundMode)
    {
        _position[2] = 0.0f;

        // Yaw 추출: R[1][0] / R[0][0] = sin(yaw) / cos(yaw)
        float yaw = std::atan2(_rotation[3], _rotation[0]);
        float c = std::cos(yaw), s = std::sin(yaw);
        _rotation = { c, -s, 0.0f,
                      s,  c, 0.0f,
                      0.0f, 0.0f, 1.0f };
    }

    // 6. 경로 기록
    _trajectory.push_back(_position);

    // 7. 직전 프레임 교체
    _prevFrame = frame;

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