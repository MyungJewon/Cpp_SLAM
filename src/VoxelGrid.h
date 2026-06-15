// Voxel Grid Filter 선언: 공간을 격자로 나눠 점을 균일하게 줄입니다.
#pragma once
#include <array>
#include <vector>

// points: 입력 점들
// voxelSize: 격자 한 칸의 크기 (단위: meter)
std::vector<std::array<float, 3>> voxelGridFilter(
    const std::vector<std::array<float, 3>>& points,
    float voxelSize);