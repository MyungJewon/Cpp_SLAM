#pragma once
// Session: 한 번의 SLAM 실행 결과(여러 서브맵). 멀티세션 병합의 입력 단위.
// points는 anchor-local, refPose는 (세션 내 루프 보정 후) local→world.
#include <string>
#include <vector>
#include "Submap.h"

struct Session {
    std::string         name;
    float               voxelSize = 0.15f;
    std::vector<Submap> submaps;
};
