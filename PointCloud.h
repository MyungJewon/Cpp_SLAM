#pragma once
#include <vector>

// 포인트 클라우드 데이터 구조체
struct PointCloud {
    std::vector<unsigned char> rawData;

    int vertexCount = 0;
    int stride      = 0;
    int colorOffset = -1;

    float centerX   = 0.0f;
    float centerY   = 0.0f;
    float centerZ   = 0.0f;
    float orthoSize = 15.0f;
};
