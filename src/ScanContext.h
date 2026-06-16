#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <utility>

static constexpr int SC_RINGS   = 20;
static constexpr int SC_SECTORS = 60;
static constexpr float SC_MAX_RADIUS = 80.0f;

using ScanContextDesc = std::array<float, SC_RINGS * SC_SECTORS>;

ScanContextDesc computeScanContext(const std::vector<std::array<float,3>>& points);
std::pair<float,int> scanContextDistance(const ScanContextDesc& a, const ScanContextDesc& b);
