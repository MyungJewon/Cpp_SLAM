// Voxel Grid Filter 구현: 공간을 격자로 나눠 점을 균일하게 줄입니다.
#include "VoxelGrid.h"
#include <unordered_map>
#include <cmath>

// 격자 인덱스를 해시맵의 키로 쓰기 위한 구조체
struct VoxelKey
{
    int ix, iy, iz;

    bool operator==(const VoxelKey& other) const
    {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

// VoxelKey를 해시맵에서 쓸 수 있도록 해시 함수 정의
struct VoxelKeyHash
{
    size_t operator()(const VoxelKey& k) const
    {
        // 세 정수를 하나의 해시값으로 합침
        size_t h1 = std::hash<int>{}(k.ix);
        size_t h2 = std::hash<int>{}(k.iy);
        size_t h3 = std::hash<int>{}(k.iz);
        return h1 ^ (h2 << 16) ^ (h3 << 32);
    }
};

// 격자 하나에 속한 점들의 합계와 개수를 저장
struct VoxelAccum
{
    float sumX = 0, sumY = 0, sumZ = 0;
    int   count = 0;
};

std::vector<std::array<float, 3>> voxelGridFilter(
    const std::vector<std::array<float, 3>>& points,
    float voxelSize)
{
    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxelMap;

    // 각 점을 격자에 분류하고 합산
    for (const auto& p : points)
    {
        VoxelKey key;
        key.ix = static_cast<int>(std::floor(p[0] / voxelSize));
        key.iy = static_cast<int>(std::floor(p[1] / voxelSize));
        key.iz = static_cast<int>(std::floor(p[2] / voxelSize));

        auto& accum = voxelMap[key];
        accum.sumX += p[0];
        accum.sumY += p[1];
        accum.sumZ += p[2];
        accum.count++;
    }

    // 각 격자의 평균점을 결과로 반환
    std::vector<std::array<float, 3>> result;
    result.reserve(voxelMap.size());

    for (const auto& [key, accum] : voxelMap)
    {
        result.push_back({
            accum.sumX / accum.count,
            accum.sumY / accum.count,
            accum.sumZ / accum.count
        });
    }

    return result;
}