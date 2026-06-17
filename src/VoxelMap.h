#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct VoxelCell {
    std::array<float, 3> center;
    std::array<float, 3> normal;
    std::array<float, 9> covariance;
    int point_count;
    bool is_planar;
};

struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        return (size_t)((int64_t)k.x * 73856093) ^
               (size_t)((int64_t)k.y * 19349663) ^
               (size_t)((int64_t)k.z * 83492791);
    }
};

class VoxelMap {
public:
    explicit VoxelMap(float voxelSize = 0.3f) : _voxelSize(voxelSize) {}

    // VGICP 공분산 고유값 상대 플로어 (조건수 상한 1/floor). 이방성 보존용.
    void setEigenFloor(float f) { _eigenFloor = f; }

    void insertAndUpdate(const std::vector<std::array<float,3>>& pts,
                         const std::array<float,3>& sensorPos = {0.f, 0.f, 0.f}) {
        for (const auto& p : pts) {
            VoxelKey key = hash(p);
            Accumulator& acc = _accumulators[key];
            acc.lastSensorPos = sensorPos;
            _dirty.insert(key);
            for (int i = 0; i < 3; ++i)
                acc.sum[i] += p[i];
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    acc.sumSquares[r*3+c] += p[r] * p[c];
            ++acc.count;
        }

        rebuildDirtyCells();
    }

    void slideWindow(const std::array<float,3>& current_pos, float radius) {
        const float radiusSq = radius * radius;
        for (auto it = _cells.begin(); it != _cells.end(); ) {
            const auto& c = it->second.center;
            float dx = c[0] - current_pos[0];
            float dy = c[1] - current_pos[1];
            float dz = c[2] - current_pos[2];
            if (dx*dx + dy*dy + dz*dz > radiusSq) {
                _accumulators.erase(it->first);
                _dirty.erase(it->first);
                it = _cells.erase(it);
            } else {
                ++it;
            }
        }
    }

    const std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash>& cells() const { return _cells; }
    size_t size() const { return _cells.size(); }

    void clear() {
        _cells.clear();
        _accumulators.clear();
        _dirty.clear();
    }

private:
    struct Accumulator {
        std::array<double, 3> sum = {0.0, 0.0, 0.0};
        std::array<double, 9> sumSquares = {0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0};
        std::array<float, 3> lastSensorPos = {0.f, 0.f, 0.f};
        int count = 0;
    };

    float _voxelSize;
    float _eigenFloor = 1e-3f;
    std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> _accumulators;
    std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> _cells;
    std::unordered_set<VoxelKey, VoxelKeyHash> _dirty;

    VoxelKey hash(const std::array<float,3>& p) const {
        int ix = (int)std::floor(p[0] / _voxelSize);
        int iy = (int)std::floor(p[1] / _voxelSize);
        int iz = (int)std::floor(p[2] / _voxelSize);
        return {ix, iy, iz};
    }

    static float dot(const std::array<float, 3>& a,
                     const std::array<float, 3>& b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    static std::array<float, 9> multiplyMat(const std::array<float, 9>& A,
                                            const std::array<float, 9>& B) {
        std::array<float, 9> C = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    C[i*3+j] += A[i*3+k] * B[k*3+j];
        return C;
    }

    static std::array<float, 9> jacobiEigenvectors(std::array<float, 9>& M) {
        std::array<float, 9> V = {1,0,0, 0,1,0, 0,0,1};

        for (int iter = 0; iter < 100; ++iter) {
            int p = 0, q = 1;
            float maxOff = std::abs(M[1]);
            if (std::abs(M[2]) > maxOff) { maxOff = std::abs(M[2]); p = 0; q = 2; }
            if (std::abs(M[5]) > maxOff) { maxOff = std::abs(M[5]); p = 1; q = 2; }
            if (maxOff < 1e-12f) break;

            float Mpq = M[p*3+q];
            float Mpp = M[p*3+p];
            float Mqq = M[q*3+q];
            float theta = 0.5f * std::atan2(2.0f * Mpq, Mpp - Mqq);
            float c = std::cos(theta);
            float s = std::sin(theta);

            std::array<float, 9> J  = {1,0,0, 0,1,0, 0,0,1};
            std::array<float, 9> Jt = {1,0,0, 0,1,0, 0,0,1};
            J[p*3+p]  =  c;  J[p*3+q] = -s;
            J[q*3+p]  =  s;  J[q*3+q] =  c;
            Jt[p*3+p] =  c;  Jt[p*3+q] =  s;
            Jt[q*3+p] = -s;  Jt[q*3+q] =  c;

            M = multiplyMat(Jt, multiplyMat(M, J));
            V = multiplyMat(V, J);
        }

        return V;
    }

    bool buildCell(const VoxelKey& key, const Accumulator& acc, VoxelCell& cell) {
        if (acc.count <= 0) return false;

        cell.point_count = acc.count;
        for (int i = 0; i < 3; ++i)
            cell.center[i] = static_cast<float>(acc.sum[i] / acc.count);

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                double meanOuter = (acc.sum[r] * acc.sum[c]) /
                                   static_cast<double>(acc.count * acc.count);
                double secondMoment = acc.sumSquares[r*3+c] /
                                      static_cast<double>(acc.count);
                cell.covariance[r*3+c] = static_cast<float>(secondMoment - meanOuter);
            }
        }

        // 원시 공분산 고유분해 (정규화 전)
        std::array<float, 9> eig = cell.covariance;
        std::array<float, 9> V = jacobiEigenvectors(eig);
        std::array<int, 3> eigCols = {0, 1, 2};
        std::sort(eigCols.begin(), eigCols.end(),
                  [&eig](int a, int b) {
                      return eig[a*3+a] < eig[b*3+b];
                  });
        int minCol = eigCols[0];
        int midCol = eigCols[1];
        int maxCol = eigCols[2];

        // VGICP 공분산 정규화: 고유값 상대 플로어로 클램프 후 재구성.
        // 이방성(disc 모양)은 유지하면서 특이행렬을 방지한다 (조건수 ≤ 1/_eigenFloor).
        float lamMax = std::max(0.0f, eig[maxCol*3+maxCol]);
        float fl = std::max(lamMax, 1e-12f) * _eigenFloor;
        std::array<float, 3> lam = {
            std::max(eig[0], fl), std::max(eig[4], fl), std::max(eig[8], fl)
        };
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) {
                float s = 0.0f;
                for (int c = 0; c < 3; ++c) s += V[r*3+c] * lam[c] * V[k*3+c];
                cell.covariance[r*3+k] = s;
            }

        cell.normal = {V[0*3+minCol], V[1*3+minCol], V[2*3+minCol]};
        float len = std::sqrt(dot(cell.normal, cell.normal));
        if (len > 1e-8f) {
            cell.normal[0] /= len;
            cell.normal[1] /= len;
            cell.normal[2] /= len;
        } else {
            cell.normal = {0.0f, 0.0f, 1.0f};
        }

        auto previousCell = _cells.find(key);
        if (previousCell != _cells.end() &&
            dot(cell.normal, previousCell->second.normal) < 0.0f) {
            cell.normal[0] = -cell.normal[0];
            cell.normal[1] = -cell.normal[1];
            cell.normal[2] = -cell.normal[2];
        }

        float minEigen = std::max(0.0f, eig[minCol*3+minCol]);
        float midEigen = std::max(0.0f, eig[midCol*3+midCol]);
        float maxEigen = std::max(0.0f, eig[maxCol*3+maxCol]);
        const bool hasThinNormalDirection =
            midEigen > 1e-12f && (minEigen / midEigen) < 0.1f;
        const bool isNotLinearStructure =
            maxEigen > 1e-12f && (midEigen / maxEigen) > 0.1f;
        cell.is_planar = acc.count >= 5 &&
                         hasThinNormalDirection &&
                         isNotLinearStructure;
        return true;
    }

    void rebuildDirtyCells() {
        for (const VoxelKey& key : _dirty) {
            auto accIt = _accumulators.find(key);
            if (accIt == _accumulators.end()) {
                _cells.erase(key);
                continue;
            }

            VoxelCell cell;
            if (buildCell(key, accIt->second, cell)) {
                _cells[key] = cell;
            } else {
                _cells.erase(key);
            }
        }
        _dirty.clear();
    }

    void rebuildCells() {
        _cells.clear();
        _cells.reserve(_accumulators.size());

        for (const auto& kv : _accumulators) {
            const VoxelKey& key = kv.first;
            const Accumulator& acc = kv.second;
            VoxelCell cell;
            if (buildCell(key, acc, cell))
                _cells.emplace(key, cell);
        }
    }
};
