// KD-Tree, ICP, Voxel Grid Filter 테스트 진입점
#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include "PointCloud.h"
#include "PlyParser.h"
#include "KDTree.h"
#include "ICP.h"
#include "VoxelGrid.h"

// 포인트 클라우드에서 KD-Tree에 넣을 형태로 변환
std::vector<std::array<float, 3>> extractPoints(const PointCloud& cloud)
{
    std::vector<std::array<float, 3>> points;
    points.reserve(cloud.vertexCount);

    for (int i = 0; i < cloud.vertexCount; ++i)
    {
        const float* pos = reinterpret_cast<const float*>(&cloud.rawData[i * cloud.stride]);
        points.push_back({ pos[0], pos[1], pos[2] });
    }
    return points;
}

// 브루트 포스: 전체를 다 뒤져서 가장 가까운 점 반환
std::array<float, 3> bruteForceNearest(const std::vector<std::array<float, 3>>& points,
                                        const std::array<float, 3>& query)
{
    float bestDist = std::numeric_limits<float>::max();
    std::array<float, 3> best = {};

    for (const auto& p : points)
    {
        float dist = 0.0f;
        for (int i = 0; i < 3; ++i)
            dist += (p[i] - query[i]) * (p[i] - query[i]);

        if (dist < bestDist)
        {
            bestDist = dist;
            best = p;
        }
    }
    return best;
}

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

    // 포인트 추출
    auto points = extractPoints(cloud);

    // KD-Tree 구축 시간 측정
    auto t0 = std::chrono::high_resolution_clock::now();
    KDTree tree;
    tree.build(points);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "KD-Tree 구축: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << "ms" << std::endl;

    // 테스트할 쿼리 점 (첫 번째 점 기준으로 살짝 이동)
    std::array<float, 3> query = { points[0][0] + 0.1f,
                                   points[0][1] + 0.1f,
                                   points[0][2] + 0.1f };

    // KD-Tree 탐색 시간 측정
    auto t2 = std::chrono::high_resolution_clock::now();
    auto kdResult = tree.nearest(query);
    auto t3 = std::chrono::high_resolution_clock::now();

    // 브루트 포스 탐색 시간 측정
    auto t4 = std::chrono::high_resolution_clock::now();
    auto bfResult = bruteForceNearest(points, query);
    auto t5 = std::chrono::high_resolution_clock::now();

    // 결과 출력
    std::cout << "\n--- 결과 비교 ---" << std::endl;
    std::cout << "쿼리 점      : ("  << query[0]    << ", " << query[1]    << ", " << query[2]    << ")" << std::endl;
    std::cout << "KD-Tree 결과 : ("  << kdResult[0] << ", " << kdResult[1] << ", " << kdResult[2] << ")" << std::endl;
    std::cout << "브루트포스   : ("  << bfResult[0] << ", " << bfResult[1] << ", " << bfResult[2] << ")" << std::endl;
    std::cout << "결과 일치    : "   << (kdResult == bfResult ? "✅ 일치" : "❌ 불일치") << std::endl;

    std::cout << "\n--- 속도 비교 ---" << std::endl;
    std::cout << "KD-Tree  : "
              << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count()
              << "μs" << std::endl;
    std::cout << "브루트포스: "
              << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count()
              << "μs" << std::endl;

    // ── 다운샘플링 ───────────────────────────────────────
    // dst(기준 맵)는 한 번만 다운샘플링해서 고정
    // src(현재 프레임)도 동일한 복셀 크기로 다운샘플링
    float voxelSize = 0.3f;

    auto dsStart = std::chrono::high_resolution_clock::now();
    auto dst_ds = voxelGridFilter(points, voxelSize);
    auto dsEnd   = std::chrono::high_resolution_clock::now();

    std::cout << "\n--- 다운샘플링 결과 ---" << std::endl;
    std::cout << "원본 점 개수     : " << points.size() << std::endl;
    std::cout << "다운샘플링 후    : " << dst_ds.size() << std::endl;
    std::cout << "축소 비율        : "
              << (1.0f - (float)dst_ds.size() / points.size()) * 100.0f
              << "%" << std::endl;
    std::cout << "다운샘플링 시간  : "
              << std::chrono::duration_cast<std::chrono::milliseconds>(dsEnd - dsStart).count()
              << "ms" << std::endl;

    // ── ICP 테스트 ───────────────────────────────────────
    // dst_ds를 직접 변환해서 src 생성 (추가 다운샘플링 없음)
    // → 복셀 격자 경계 불일치 문제를 제거하고 순수하게 ICP 정확도만 테스트

    // 테스트용 이동벡터 (작은 값으로 ICP 수렴 범위 안에 들어오도록)
    std::array<float, 3> trueT = { 0.05f, 0.03f, 0.02f };

    // 테스트용 회전행렬 (Z축 기준 1도 회전)
    float angle = 1.0f * M_PI / 180.0f;
    Matrix3x3 trueR = {
         std::cos(angle), -std::sin(angle), 0,
         std::sin(angle),  std::cos(angle), 0,
         0,                0,               1
    };

    // dst_ds를 직접 변환 → 같은 점 집합이므로 대응점 오류 없음
    std::vector<std::array<float, 3>> src_ds;
    src_ds.reserve(dst_ds.size());
    for (const auto& p : dst_ds)
    {
        std::array<float, 3> rotated = {
            trueR[0]*p[0] + trueR[1]*p[1] + trueR[2]*p[2],
            trueR[3]*p[0] + trueR[4]*p[1] + trueR[5]*p[2],
            trueR[6]*p[0] + trueR[7]*p[1] + trueR[8]*p[2]
        };
        src_ds.push_back({ rotated[0] + trueT[0],
                           rotated[1] + trueT[1],
                           rotated[2] + trueT[2] });
    }

    std::cout << "\n--- ICP 테스트 ---" << std::endl;
    std::cout << "dst 점 개수 : " << dst_ds.size() << std::endl;
    std::cout << "src 점 개수 : " << src_ds.size() << std::endl;
    std::cout << "실제 이동값 : ("
              << trueT[0] << ", " << trueT[1] << ", " << trueT[2] << ")" << std::endl;
    std::cout << "실제 회전각 : 1도 (Z축)" << std::endl;

    // ICP 실행
    auto icpStart = std::chrono::high_resolution_clock::now();
    ICPResult icp = runICP(src_ds, dst_ds);
    auto icpEnd   = std::chrono::high_resolution_clock::now();

    std::cout << "\n--- ICP 결과 ---" << std::endl;
    std::cout << "반복 횟수 : " << icp.iterations << std::endl;
    std::cout << "최종 오차 : " << icp.error << std::endl;
    std::cout << "추정 이동값: ("
              << -icp.t[0] << ", " << -icp.t[1] << ", " << -icp.t[2] << ")" << std::endl;
    std::cout << "ICP 소요시간: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(icpEnd - icpStart).count()
              << "ms" << std::endl;

    return 0;
}