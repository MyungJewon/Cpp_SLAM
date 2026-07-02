// Map Builder 구현: 각 프레임의 포인트를 전역 좌표로 변환해 누적 맵을 생성합니다.
#include "MapBuilder.h"
#include <cmath>
#include <fstream>
#include <iostream>

MapBuilder::MapBuilder(float voxelSize, int cleanupInterval)
    : _voxelSize(voxelSize)
    , _cleanupInterval(cleanupInterval)
    , _frameCount(0)
{
}

void MapBuilder::addFrame(const std::vector<std::array<float, 3>>& points,
                           const Matrix3x3&                         rotation,
                           const std::array<float, 3>&              position)
{
    // 각 점을 전역 좌표로 변환하며 증분 voxel 중복제거.
    // 이미 점이 있는 voxel은 스킵 → 프레임당 O(신규 점).
    // (기존: N프레임마다 전체 맵 voxelGridFilter = O(맵 크기), 맵이 클수록 급격히 느려짐)
    for (const auto& p : points)
    {
        std::array<float, 3> global = {
            rotation[0]*p[0] + rotation[1]*p[1] + rotation[2]*p[2] + position[0],
            rotation[3]*p[0] + rotation[4]*p[1] + rotation[5]*p[2] + position[1],
            rotation[6]*p[0] + rotation[7]*p[1] + rotation[8]*p[2] + position[2]
        };
        VoxelKey key{(int)std::floor(global[0] / _voxelSize),
                     (int)std::floor(global[1] / _voxelSize),
                     (int)std::floor(global[2] / _voxelSize)};
        if (_occupied.insert(key).second)
            _map.push_back(global);
    }

    ++_frameCount;
}

void MapBuilder::rebuildOccupied()
{
    _occupied.clear();
    _occupied.reserve(_map.size() * 2);
    for (const auto& p : _map)
        _occupied.insert(VoxelKey{(int)std::floor(p[0] / _voxelSize),
                                  (int)std::floor(p[1] / _voxelSize),
                                  (int)std::floor(p[2] / _voxelSize)});
}

int MapBuilder::getPointCount() const
{
    return (int)_map.size();
}

const std::vector<std::array<float, 3>>& MapBuilder::getMap() const
{
    return _map;
}

void MapBuilder::storeKeyFramePoints(int keyFrameId,
                                     const std::vector<std::array<float, 3>>& localPoints,
                                     const Pose3D& pose)
{
    if (keyFrameId < 0)
        return;

    if ((int)_keyFrameData.size() <= keyFrameId)
        _keyFrameData.resize((size_t)keyFrameId + 1);

    _keyFrameData[(size_t)keyFrameId] = {localPoints, pose};
}

void MapBuilder::rebuildFromPoses(const std::vector<Pose3D>& optimizedPoses)
{
    _map.clear();

    for (size_t id = 0; id < _keyFrameData.size(); ++id)
    {
        const auto& keyFrame = _keyFrameData[id];
        if (keyFrame.localPoints.empty())
            continue;

        const Pose3D& pose = (id < optimizedPoses.size()) ? optimizedPoses[id] : keyFrame.pose;
        const auto& R = pose.R;
        const auto& t = pose.t;

        _map.reserve(_map.size() + keyFrame.localPoints.size());
        for (const auto& p : keyFrame.localPoints)
        {
            _map.push_back({
                R[0]*p[0] + R[1]*p[1] + R[2]*p[2] + t[0],
                R[3]*p[0] + R[4]*p[1] + R[5]*p[2] + t[1],
                R[6]*p[0] + R[7]*p[1] + R[8]*p[2] + t[2]
            });
        }
    }

    _map = voxelGridFilter(_map, _voxelSize);
    rebuildOccupied();  // 재구성 후 증분 중복제거 상태 동기화
    _frameCount = (int)_keyFrameData.size();
    std::cout << "[MapBuilder] 포즈 그래프 결과로 맵 재구성: "
              << _map.size() << "개 점" << std::endl;
}

bool MapBuilder::saveToPly(const std::string& filePath) const
{
    std::ofstream file(filePath);
    if (!file.is_open())
    {
        std::cout << "[MapBuilder] 파일 저장 실패: " << filePath << std::endl;
        return false;
    }

    // PLY 헤더 작성 (ASCII 포맷)
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << _map.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // 각 점 기록
    for (const auto& p : _map)
        file << p[0] << " " << p[1] << " " << p[2] << "\n";

    file.close();
    std::cout << "[MapBuilder] 맵 저장 완료: " << filePath
              << " (" << _map.size() << "개 점)" << std::endl;
    return true;
}
