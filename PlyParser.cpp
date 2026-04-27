#include "PlyParser.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>

// PLY 파일을 파싱해 PointCloud 구조체에 데이터를 채웁니다. ASCII/바이너리 포맷 모두 지원합니다.
bool loadPly(const std::string &filePath, PointCloud &cloud)
{
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open())
    {
        std::cout << "파일을 열 수 없습니다: " << filePath << std::endl;
        return false;
    }

    // 헤더에서 읽은 property 순서를 기록 (ASCII 파싱 시 순서대로 처리하기 위해)
    enum PropType { PROP_FLOAT, PROP_UCHAR };
    std::vector<PropType> propOrder;

    bool isAscii = false;
    std::string line;
    while (std::getline(file, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (line.find("format ascii") != std::string::npos)
        {
            isAscii = true;
        }
        else if (line.find("element vertex") != std::string::npos)
        {
            std::stringstream ss(line);
            std::string dummy;
            ss >> dummy >> dummy >> cloud.vertexCount;
        }
        else if (line.find("property float") != std::string::npos)
        {
            propOrder.push_back(PROP_FLOAT);
            cloud.stride += 4;
        }
        else if (line.find("property uchar") != std::string::npos)
        {
            if (line.find("red") != std::string::npos)
                cloud.colorOffset = cloud.stride;
            propOrder.push_back(PROP_UCHAR);
            cloud.stride += 1;
        }
        else if (line == "end_header")
        {
            break;
        }
    }

    std::cout << "Format       : " << (isAscii ? "ASCII" : "Binary") << std::endl;
    std::cout << "Vertex Count : " << cloud.vertexCount << std::endl;
    std::cout << "Stride       : " << cloud.stride << " bytes" << std::endl;
    std::cout << "Color Offset : " << (cloud.colorOffset == -1 ? "없음" : std::to_string(cloud.colorOffset)) << std::endl;

    cloud.rawData.resize(cloud.vertexCount * cloud.stride);

    if (isAscii)
    {
        // ASCII: 한 줄 = 한 vertex, 공백으로 토큰 분리 후 순서대로 rawData에 pack
        for (int i = 0; i < cloud.vertexCount; ++i)
        {
            if (!std::getline(file, line)) break;
            if (!line.empty() && line.back() == '\r') line.pop_back();

            std::stringstream ss(line);
            unsigned char* dst = &cloud.rawData[i * cloud.stride];

            for (PropType type : propOrder)
            {
                if (type == PROP_FLOAT)
                {
                    float val;
                    ss >> val;
                    std::memcpy(dst, &val, 4);
                    dst += 4;
                }
                else
                {
                    int val;
                    ss >> val;
                    *dst = static_cast<unsigned char>(val);
                    dst += 1;
                }
            }
        }
    }
    else
    {
        // Binary: 헤더 직후부터 raw bytes를 한번에 읽기
        file.read(reinterpret_cast<char *>(cloud.rawData.data()), cloud.rawData.size());
    }

    file.close();

    if (cloud.vertexCount > 0)
    {
        float sumX = 0, sumY = 0, sumZ = 0;
        for (int i = 0; i < cloud.vertexCount; ++i)
        {
            float *pos = reinterpret_cast<float *>(&cloud.rawData[i * cloud.stride]);
            sumX += pos[0];
            sumY += pos[1];
            sumZ += pos[2];
        }
        cloud.centerX = sumX / cloud.vertexCount;
        cloud.centerY = sumY / cloud.vertexCount;
        cloud.centerZ = sumZ / cloud.vertexCount;

        float maxDist = 0.0f;
        for (int i = 0; i < cloud.vertexCount; ++i)
        {
            float *pos = reinterpret_cast<float *>(&cloud.rawData[i * cloud.stride]);
            float dist = std::max({std::abs(pos[0] - cloud.centerX),
                                   std::abs(pos[1] - cloud.centerY),
                                   std::abs(pos[2] - cloud.centerZ)});
            if (dist > maxDist) maxDist = dist;
        }
        cloud.orthoSize = maxDist * 1.2f;
    }

    return true;
}
