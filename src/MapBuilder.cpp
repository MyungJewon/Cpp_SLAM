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

    // 진단: 키프레임별 회전(rpy)·위치·점군 spread를 output/keyframes.log에 기록.
    // "위치는 멀쩡한데 맵만 폭발"의 원인(회전 오염 vs 로컬점 폭주)을 특정하기 위함.
    std::ofstream kfLog("output/keyframes.log");
    kfLog << "# id  roll pitch yaw(deg)  tx ty tz  localMaxR  worldSpread\n";

    for (size_t id = 0; id < _keyFrameData.size(); ++id)
    {
        const auto& keyFrame = _keyFrameData[id];
        if (keyFrame.localPoints.empty())
            continue;

        const Pose3D& pose = (id < optimizedPoses.size()) ? optimizedPoses[id] : keyFrame.pose;
        const auto& R = pose.R;
        const auto& t = pose.t;

        // 회전을 roll/pitch/yaw(deg)로 (R = Rz*Ry*Rx 가정)
        float pitch = std::asin(std::max(-1.0f, std::min(1.0f, -R[6])));
        float roll  = std::atan2(R[7], R[8]);
        float yaw   = std::atan2(R[3], R[0]);
        const float RAD = 180.0f / 3.14159265f;

        // 로컬 점군 최대 반경(원본 스캔 크기) + 변환 후 월드 spread(퍼짐)
        float localMaxR = 0.0f;
        float wminx=1e30f,wmaxx=-1e30f,wminy=1e30f,wmaxy=-1e30f,wminz=1e30f,wmaxz=-1e30f;

        _map.reserve(_map.size() + keyFrame.localPoints.size());
        for (const auto& p : keyFrame.localPoints)
        {
            float lr = std::sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
            if (lr > localMaxR) localMaxR = lr;
            std::array<float,3> w = {
                R[0]*p[0] + R[1]*p[1] + R[2]*p[2] + t[0],
                R[3]*p[0] + R[4]*p[1] + R[5]*p[2] + t[1],
                R[6]*p[0] + R[7]*p[1] + R[8]*p[2] + t[2]
            };
            wminx=std::min(wminx,w[0]);wmaxx=std::max(wmaxx,w[0]);
            wminy=std::min(wminy,w[1]);wmaxy=std::max(wmaxy,w[1]);
            wminz=std::min(wminz,w[2]);wmaxz=std::max(wmaxz,w[2]);
            _map.push_back(w);
        }
        float spread = std::max(wmaxx-wminx, std::max(wmaxy-wminy, wmaxz-wminz));
        kfLog << id << "  " << roll*RAD << " " << pitch*RAD << " " << yaw*RAD
              << "  " << t[0] << " " << t[1] << " " << t[2]
              << "  " << localMaxR << "  " << spread << "\n";
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
