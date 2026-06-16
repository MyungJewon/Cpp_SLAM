#include "ScanContext.h"
#include <limits>
#include <algorithm>
#include <numeric>

ScanContextDesc computeScanContext(const std::vector<std::array<float,3>>& points) {
    ScanContextDesc desc;
    desc.fill(-1.0f);

    for (const auto& p : points) {
        float x = p[0], y = p[1], z = p[2];
        float r = std::sqrt(x*x + y*y);
        if (r < 0.1f || r > SC_MAX_RADIUS) continue;

        int ring = (int)(r / SC_MAX_RADIUS * SC_RINGS);
        if (ring >= SC_RINGS) ring = SC_RINGS - 1;

        float theta = std::atan2(y, x);
        if (theta < 0) theta += 2.0f * M_PI;
        int sector = (int)(theta / (2.0f * M_PI) * SC_SECTORS);
        if (sector >= SC_SECTORS) sector = SC_SECTORS - 1;

        int idx = ring * SC_SECTORS + sector;
        if (z > desc[idx]) desc[idx] = z;
    }

    for (auto& v : desc)
        if (v < 0.0f) v = 0.0f;

    return desc;
}

std::pair<float,int> scanContextDistance(const ScanContextDesc& a, const ScanContextDesc& b) {
    float bestDist = std::numeric_limits<float>::max();
    int bestShift = 0;

    for (int shift = 0; shift < SC_SECTORS; ++shift) {
        float dist = 0.0f;
        int validCols = 0;

        for (int s = 0; s < SC_SECTORS; ++s) {
            int bs = (s + shift) % SC_SECTORS;
            float colDistSq = 0.0f;
            bool hasData = false;
            for (int r = 0; r < SC_RINGS; ++r) {
                float diff = a[r * SC_SECTORS + s] - b[r * SC_SECTORS + bs];
                colDistSq += diff * diff;
                if (a[r*SC_SECTORS+s] > 0 || b[r*SC_SECTORS+bs] > 0)
                    hasData = true;
            }
            if (hasData) {
                dist += std::sqrt(colDistSq / SC_RINGS);
                ++validCols;
            }
        }

        if (validCols > 0) dist /= validCols;
        if (dist < bestDist) {
            bestDist = dist;
            bestShift = shift;
        }
    }

    return {bestDist, bestShift};
}
