// KD-Tree 테스트 진입점
#include <iostream>
#include "PointCloud.h"
#include "PlyParser.h"

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cout << "사용법: ./slam <ply파일경로>" << std::endl;
        return -1;
    }

    PointCloud cloud;
    if (!loadPly(argv[1], cloud))
    {
        std::cout << "PLY 로드 실패" << std::endl;
        return -1;
    }

    std::cout << "로드 완료: " << cloud.vertexCount << "개 점" << std::endl;

    return 0;
}