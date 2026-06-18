#pragma once
// Submap: 점군 덩어리 + 월드 기준 자세(앵커) + 경계상자(AABB).
// 실시간 SLAM의 키프레임 누적 결과이자, 오프라인 PLY 병합 시 로드된 PLY 한 장이기도 하다.
// points는 항상 anchorId 노드의 "로컬 좌표계" 기준으로 저장된다.
#include <array>
#include <vector>
#include "PoseGraph.h"  // Pose3D

struct Submap {
    int                                id        = -1;  // 서브맵 식별자
    int                                anchorId  = -1;  // 연결된 PoseGraph 노드 id
    Pose3D                             refPose;          // 앵커 노드의 (생성 시점) world pose. local→world
    std::vector<std::array<float,3>>   points;           // 로컬 좌표계 점군
    std::array<float,6>                aabb = {0,0,0,0,0,0}; // [minx,miny,minz, maxx,maxy,maxz] (로컬)
};

// 두 서브맵 정합 결과
struct RegistrationResult {
    bool   success      = false;
    Pose3D relativePose;             // src(로컬) → dst(로컬) 변환. 성공 시 BetweenFactor(dst→src) 측정값
    float  fitness      = 0.0f;      // GICP 대응점 비율 (0~1)
    float  overlap      = 0.0f;      // 최종 정렬 후 dst voxel에 들어간 src 점 비율 (perceptual aliasing 방어)
    float  rmse         = 0.0f;      // GICP 최종 오차
};
