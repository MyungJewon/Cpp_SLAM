#pragma once
#include <array>
#include <vector>
#include <unordered_map>
#include <cmath>

class VoxelMap {
public:
    explicit VoxelMap(float voxelSize = 0.3f) : _voxelSize(voxelSize) {}

    void insert(const std::array<float,3>& p) {
        size_t key = hash(p);
        _cells.emplace(key, p);
    }

    void insert(const std::vector<std::array<float,3>>& pts) {
        for (const auto& p : pts) insert(p);
    }

    std::vector<std::array<float,3>> toVector() const {
        std::vector<std::array<float,3>> out;
        out.reserve(_cells.size());
        for (const auto& [k, v] : _cells) out.push_back(v);
        return out;
    }

    size_t size() const { return _cells.size(); }
    void clear() { _cells.clear(); }

    void trimToMax(int maxCells) {
        if ((int)_cells.size() <= maxCells) return;
        std::unordered_map<size_t, std::array<float,3>> trimmed;
        trimmed.reserve(maxCells);
        int i = 0;
        for (auto& [k, v] : _cells) {
            if (i++ % 2 == 0) trimmed.emplace(k, v);
            if ((int)trimmed.size() >= maxCells) break;
        }
        _cells = std::move(trimmed);
    }

private:
    float _voxelSize;
    std::unordered_map<size_t, std::array<float,3>> _cells;

    size_t hash(const std::array<float,3>& p) const {
        int ix = (int)std::floor(p[0] / _voxelSize);
        int iy = (int)std::floor(p[1] / _voxelSize);
        int iz = (int)std::floor(p[2] / _voxelSize);
        size_t hx = (size_t)(ix * 73856093);
        size_t hy = (size_t)(iy * 19349663);
        size_t hz = (size_t)(iz * 83492791);
        return hx ^ hy ^ hz;
    }
};
