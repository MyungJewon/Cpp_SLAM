// ICP 선언: 두 포인트 클라우드를 정렬해 이동/회전을 추정합니다.
#pragma once
#include <array>
#include <unordered_map>
#include <vector>
#include "KDTree.h"
#include "VoxelMap.h"

// 3x3 행렬을 1차원 배열로 표현 (row-major)
// [0][1][2]
// [3][4][5]
// [6][7][8]
using Matrix3x3 = std::array<float, 9>;

// ICP 결과: 회전행렬 R과 이동벡터 t
struct ICPResult
{
    Matrix3x3            R;          // 회전행렬
    std::array<float, 3> t;          // 이동벡터
    int                  iterations; // 실제 반복 횟수
    float                error;      // 최종 오차
    float                fitness;    // 매칭된 점 비율 (0~1) — src 점 중 maxDistSq 이내로 대응된 비율
};

// src를 dst에 맞추는 ICP 실행
// initialR / initialT: ICP 시작 전 적용할 초기 변환 (IMU 예측값 등)
//                      nullptr이면 항등변환으로 시작
ICPResult runICP(const std::vector<std::array<float, 3>>& src,
                 const std::vector<std::array<float, 3>>& dst,
                 int              maxIterations = 20,
                 float            tolerance     = 1e-4f,
                 const Matrix3x3* initialR      = nullptr,
                 const std::array<float, 3>* initialT = nullptr,
                 bool             usePointToPlane = false,
                 bool             useGICP = false,
                 const std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash>* voxelMap = nullptr,
                 float            voxelSize = 0.3f,
                 // 중력 prior (L2): 바디 좌표계 "위" 단위벡터. nullptr이면 미사용.
                 // result.R * gravityBodyUp 가 월드 up(0,0,1)에 맞도록 roll/pitch를
                 // soft constraint로 구속 (yaw는 자유). GICP 경로에서만 적용.
                 const std::array<float, 3>* gravityBodyUp = nullptr,
                 float            gravityScale = 0.0f,
                 // 대응점 최대 거리 오버라이드(m). 0이면 기본값
                 // (GICP: max(0.5, voxel×1.25) / P2P: 0.35). 빠른 이동 시 예측 오차만큼
                 // 넓혀 대응을 회복한다 (P2P 폴백은 KDTree 전역 탐색이라 실효 큼).
                 float            corrMaxDist = 0.0f);
