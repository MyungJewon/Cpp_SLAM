// Map Builder 구현: 각 프레임의 포인트를 전역 좌표로 변환해 누적 맵을 생성합니다.
#include "MapBuilder.h"
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
    // 1. 각 점을 전역 좌표로 변환
    // 로컬 좌표 → 회전 적용 → 위치 이동 → 전역 좌표
    for (const auto& p : points)
    {
        std::array<float, 3> global = {
            rotation[0]*p[0] + rotation[1]*p[1] + rotation[2]*p[2] + position[0],
            rotation[3]*p[0] + rotation[4]*p[1] + rotation[5]*p[2] + position[1],
            rotation[6]*p[0] + rotation[7]*p[1] + rotation[8]*p[2] + position[2]
        };
        _map.push_back(global);
    }

    ++_frameCount;

    // 2. 일정 프레임마다 Voxel Grid로 맵 정리
    if (_frameCount % _cleanupInterval == 0)
    {
        int before = (int)_map.size();
        _map = voxelGridFilter(_map, _voxelSize);
        std::cout << "[MapBuilder] 맵 정리: " << before
                  << " → " << _map.size() << "개 점" << std::endl;
    }
}

int MapBuilder::getPointCount() const
{
    return (int)_map.size();
}

const std::vector<std::array<float, 3>>& MapBuilder::getMap() const
{
    return _map;
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