#pragma once
#include <string>
#include "PointCloud.h"

// PLY 파일을 파싱해 PointCloud 구조체에 데이터를 채웁니다.
bool loadPly(const std::string& filePath, PointCloud& cloud);
