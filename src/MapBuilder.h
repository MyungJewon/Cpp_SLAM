// Map Builder 선언: 각 프레임의 포인트를 전역 좌표로 변환해 누적 맵을 생성합니다.
#pragma once
#include <array>
#include <vector>
#include <string>
#include <unordered_set>
#include "ICP.h"
#include "VoxelGrid.h"
#include "VoxelMap.h"  // VoxelKey/VoxelKeyHash (증분 중복제거용)
#include "PoseGraph.h"

struct KeyFrameData
{
    std::vector<std::array<float, 3>> localPoints;
    Pose3D pose;
};

class MapBuilder
{
public:
    // voxelSize      : 맵 정리에 쓸 복셀 크기
    // cleanupInterval: 이 프레임 수마다 맵을 Voxel Grid로 정리 (메모리 관리)
    MapBuilder(float voxelSize = 0.3f, int cleanupInterval = 10);

    // 새 프레임 추가
    // points   : 현재 프레임의 포인트 (로컬 좌표)
    // rotation : 현재 누적 회전행렬 (Odometry에서 가져옴)
    // position : 현재 누적 위치     (Odometry에서 가져옴)
    void addFrame(const std::vector<std::array<float, 3>>& points,
                  const Matrix3x3&                         rotation,
                  const std::array<float, 3>&              position);

    // 현재 맵의 점 개수 반환
    int getPointCount() const;

    // 맵 전체 반환
    const std::vector<std::array<float, 3>>& getMap() const;

    void storeKeyFramePoints(int keyFrameId,
                             const std::vector<std::array<float, 3>>& localPoints,
                             const Pose3D& pose);

    void rebuildFromPoses(const std::vector<Pose3D>& optimizedPoses);

    // 맵을 PLY 파일로 저장 (ASCII 포맷)
    bool saveToPly(const std::string& filePath) const;

private:
    float _voxelSize;
    int   _cleanupInterval;
    int   _frameCount;

    std::vector<std::array<float, 3>> _map;
    std::vector<KeyFrameData> _keyFrameData;

    // 증분 voxel 중복제거: 이미 점이 있는 voxel은 스킵 → 프레임당 O(신규 점).
    // (기존: 주기적으로 전체 맵 voxelGridFilter = O(맵 크기), 맵이 클수록 느려짐)
    std::unordered_set<VoxelKey, VoxelKeyHash> _occupied;
    void rebuildOccupied();  // rebuildFromPoses 후 _map 기준으로 재구축
};
