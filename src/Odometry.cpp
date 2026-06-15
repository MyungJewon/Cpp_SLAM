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
    auto currentFrame = voxelGridFilter(rawPoints, _voxelSize);

    // 2. 첫 프레임이면 월드 좌표계 로컬 맵의 기준으로 저장만 하고 종료
    if (_isFirst)
    {
        _lastFrame = currentFrame;
        _localMap.insert(_localMap.end(), currentFrame.begin(), currentFrame.end());
        _localMapFrameCount = 1;
        _trajectory.push_back(_position);
        _isFirst = false;
        std::cout << "[Odometry] 첫 프레임 저장 (" << currentFrame.size() << "개 점)" << std::endl;
        return;
    }

    // 3. 누적 로컬 맵과 ICP 실행 (Scan-to-Map)
    //
    // src = currentFrame (로컬/바디 좌표)
    // dst = _localMap    (월드 좌표)
    //
    // 반드시 현재 pose(_rotation, _position)로 초기화해야 합니다.
    // 그래야 로컬 포인트가 월드 좌표로 미리 변환되어 대응점 탐색이 가능합니다.
    // ICP 결과 R, t는 "로컬 → 월드" 절대 변환입니다.
    //
    // IMU 상태를 먼저 꺼내고 즉시 reset — ICP 성공·실패에 무관하게 누출 없음
    Matrix3x3 initR = _rotation;
    std::array<float, 3> initT = _position;

    if (_imu)
    {
        bool hasSamples = (_imu->sampleCount() > 0);
        Matrix3x3 imuR  = _imu->getRotation();
        _imu->reset();

        if (hasSamples)
        {
            // IMU 회전을 현재 누적 회전에 합성해 초기값으로 사용
            Matrix3x3 composedR = {};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    for (int k = 0; k < 3; ++k)
                        composedR[i*3+j] += imuR[i*3+k] * _rotation[k*3+j];
            initR = composedR;
        }
    }

    ICPResult icp = runICP(currentFrame, _localMap, 20, 1e-4f, &initR, &initT, true);

    // 4. 포즈 갱신
    // Scan-to-Map ICP 결과는 "로컬 → 월드" 절대 변환이므로 직접 교체합니다.
    //
    // 비정상 점프 판정: 이전 위치와 새 위치 사이의 거리로 계산합니다.
    float dx = icp.t[0] - _position[0];
    float dy = icp.t[1] - _position[1];
    float dz = icp.t[2] - _position[2];
    float stepDist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (stepDist > _maxStepDist)
    {
        std::cout << "[Odometry] 경고: 프레임 " << _trajectory.size()+1
                  << " ICP 점프 무시 (step=" << stepDist << "m > " << _maxStepDist << "m)" << std::endl;
        _trajectory.push_back(_position);
        _lastFrame = currentFrame;
        return;
    }

    _position = icp.t;
    _rotation = icp.R;

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

    // 6. 현재 프레임을 월드 좌표계로 변환해 로컬 맵에 누적
    _localMap.reserve(_localMap.size() + currentFrame.size());
    for (const auto& p : currentFrame)
    {
        std::array<float, 3> pWorld = {
            _rotation[0]*p[0] + _rotation[1]*p[1] + _rotation[2]*p[2] + _position[0],
            _rotation[3]*p[0] + _rotation[4]*p[1] + _rotation[5]*p[2] + _position[1],
            _rotation[6]*p[0] + _rotation[7]*p[1] + _rotation[8]*p[2] + _position[2]
        };
        _localMap.push_back(pWorld);
    }
    ++_localMapFrameCount;

    // 로컬 맵 크기를 항상 _localMapMaxPts 이하로 유지
    // KD-Tree 빌드 비용이 점 개수에 비례하므로 상한을 타이트하게 잡습니다.
    if (_localMap.size() > (size_t)_localMapMaxPts)
    {
        _localMap = voxelGridFilter(_localMap, _voxelSize);
        _localMapFrameCount = 0;
        // 필터 후에도 초과 시 강제 절삭 (voxelSize가 너무 작은 경우 대비)
        if (_localMap.size() > (size_t)_localMapMaxPts)
            _localMap.resize(_localMapMaxPts);
    }

    // 7. 마지막 다운샘플링 프레임 교체
    _lastFrame = currentFrame;

    // 8. 경로 기록
    _trajectory.push_back(_position);

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
