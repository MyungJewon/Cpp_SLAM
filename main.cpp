// KD-Tree, ICP, Voxel Grid Filter, Odometry, Loop Closure, Map Builder, Bag Parser 테스트 진입점
#include <iostream>
#include <fstream>
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
#include "Odometry.h"
#include "LoopCloser.h"
#include "MapBuilder.h"
#include "BagParser.h"

// ── 경로를 색상 그라디언트 PLY로 저장 (초록=시작, 빨강=끝) ──────────
void saveTrajectoryPly(const std::vector<std::array<float, 3>>& traj,
                        const std::string& path)
{
    std::ofstream f(path);
    f << "ply\nformat ascii 1.0\n"
      << "element vertex " << traj.size() << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "end_header\n";

    int n = (int)traj.size();
    for (int i = 0; i < n; ++i)
    {
        float ratio = (n > 1) ? (float)i / (n - 1) : 0.0f;
        int r = (int)(ratio * 255);
        int g = (int)((1.0f - ratio) * 255);
        f << traj[i][0] << " " << traj[i][1] << " " << traj[i][2]
          << " " << r << " " << g << " 50\n";
    }
    std::cout << "[Visualizer] 경로 PLY 저장: " << path
              << " (" << n << "개 점)" << std::endl;
}

// ── 2D 탑다운 이미지 저장 (PPM 포맷 — 라이브러리 없음) ──────────────
// 맵 점은 청회색, 경로는 초록→빨강 그라디언트로 그려요.
// macOS에서는 미리보기(Preview.app)로 바로 열 수 있어요.
void saveMapImage(const std::vector<std::array<float, 3>>& traj,
                  const std::vector<std::array<float, 3>>& mapPts,
                  const std::string& path, int imgSize = 1024)
{
    if (traj.empty()) return;

    // 바운딩 박스 (경로 + 맵 점 모두 포함)
    float minX = traj[0][0], maxX = traj[0][0];
    float minY = traj[0][1], maxY = traj[0][1];
    for (const auto& p : traj)   { minX=std::min(minX,p[0]); maxX=std::max(maxX,p[0]);
                                    minY=std::min(minY,p[1]); maxY=std::max(maxY,p[1]); }
    for (const auto& p : mapPts) { minX=std::min(minX,p[0]); maxX=std::max(maxX,p[0]);
                                    minY=std::min(minY,p[1]); maxY=std::max(maxY,p[1]); }

    float range = std::max(maxX - minX, maxY - minY) * 1.1f + 0.1f;
    float cx = (minX + maxX) / 2.0f;
    float cy = (minY + maxY) / 2.0f;

    // 좌표 → 픽셀 (y축 반전: 화면 위 = 월드 위)
    auto toPixel = [&](float x, float y) -> std::pair<int,int> {
        int px = (int)((x - cx) / range * imgSize + imgSize * 0.5f);
        int py = (int)(-(y - cy) / range * imgSize + imgSize * 0.5f);
        return { std::max(0, std::min(imgSize-1, px)),
                 std::max(0, std::min(imgSize-1, py)) };
    };

    // 배경: 어두운 회색
    std::vector<uint8_t> buf(imgSize * imgSize * 3, 25);

    // 맵 점: 청회색
    for (const auto& p : mapPts)
    {
        auto [x, y] = toPixel(p[0], p[1]);
        int idx = (y * imgSize + x) * 3;
        buf[idx] = 60; buf[idx+1] = 70; buf[idx+2] = 80;
    }

    // 경로: 초록(시작) → 빨강(끝), 3×3 점으로 굵게
    int n = (int)traj.size();
    for (int i = 0; i < n; ++i)
    {
        auto [x, y] = toPixel(traj[i][0], traj[i][1]);
        float ratio = (n > 1) ? (float)i / (n - 1) : 0.0f;
        uint8_t r = (uint8_t)(ratio * 255);
        uint8_t g = (uint8_t)((1.0f - ratio) * 200 + 55);
        for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
        {
            int nx = std::max(0, std::min(imgSize-1, x+dx));
            int ny = std::max(0, std::min(imgSize-1, y+dy));
            int idx = (ny * imgSize + nx) * 3;
            buf[idx] = r; buf[idx+1] = g; buf[idx+2] = 50;
        }
    }

    // PPM P6 (바이너리) 저장
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << imgSize << " " << imgSize << "\n255\n";
    f.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());
    std::cout << "[Visualizer] 2D 맵 이미지 저장: " << path
              << " (" << imgSize << "x" << imgSize << "px)" << std::endl;
    std::cout << "             → macOS Finder에서 더블클릭하면 미리보기로 열려요" << std::endl;
}

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

    // ── Odometry 테스트 ──────────────────────────────────
    // 실제 센서가 없으니 PLY 포인트 클라우드를 매 프레임마다
    // 조금씩 이동시켜서 연속 프레임처럼 흉내냅니다.
    // 5프레임 동안 x축으로 0.05m씩 이동하는 상황을 시뮬레이션해요.

    std::cout << "\n--- Odometry 테스트 ---" << std::endl;
    std::cout << "시뮬레이션: 5프레임, 매 프레임 x축 +0.05m 이동" << std::endl;

    Odometry odom(0.3f);

    // 프레임 0: 원본 그대로 (시작 위치)
    odom.addFrame(points);

    // 프레임 1~4: x축으로 0.05m씩 누적 이동
    for (int frame = 1; frame <= 4; ++frame)
    {
        float offsetX = 0.05f * frame;

        std::vector<std::array<float, 3>> shifted;
        shifted.reserve(points.size());
        for (const auto& p : points)
            shifted.push_back({ p[0] + offsetX, p[1], p[2] });

        odom.addFrame(shifted);
    }

    // 최종 위치 출력
    auto finalPos = odom.getPosition();
    std::cout << "\n--- Odometry 결과 ---" << std::endl;
    std::cout << "예상 최종 위치 : (0.2, 0, 0)" << std::endl;
    std::cout << "추정 최종 위치 : ("
              << finalPos[0] << ", "
              << finalPos[1] << ", "
              << finalPos[2] << ")" << std::endl;

    std::cout << "\n경로 기록:" << std::endl;
    const auto& traj = odom.getTrajectory();
    for (int i = 0; i < (int)traj.size(); ++i)
        std::cout << "  프레임 " << i << " : ("
                  << traj[i][0] << ", "
                  << traj[i][1] << ", "
                  << traj[i][2] << ")" << std::endl;

    // ── Loop Closure 테스트 ───────────────────────────────
    // 시뮬레이션: x축으로 이동했다가 다시 출발점 근처로 돌아오는 경로
    // 프레임 0~4: x축 +0.05m씩 전진 (Odometry와 동일)
    // 프레임 5~8: x축 -0.05m씩 복귀 → 출발점 근처로 돌아옴
    // Loop Closer가 돌아온 시점을 감지하고 경로를 보정해야 해요.

    std::cout << "\n--- Loop Closure 테스트 ---" << std::endl;
    std::cout << "시뮬레이션: 전진 5프레임 → 복귀 4프레임 (루프 경로)" << std::endl;

    Odometry    odom2(0.3f);
    LoopCloser  loopCloser(0.08f, 0.05f, 3);

    auto ds_points = voxelGridFilter(points, 0.3f);

    // 전진 구간 (프레임 0~4)
    for (int frame = 0; frame <= 4; ++frame)
    {
        float offsetX = 0.05f * frame;
        std::vector<std::array<float, 3>> shifted;
        shifted.reserve(points.size());
        for (const auto& p : points)
            shifted.push_back({ p[0] + offsetX, p[1], p[2] });

        odom2.addFrame(shifted);

        auto pos    = odom2.getPosition();
        auto ds_pts = voxelGridFilter(shifted, 0.3f);
        loopCloser.addKeyFrame(frame, pos, ds_pts);
    }

    // 복귀 구간 (프레임 5~8): 출발점 방향으로 돌아옴
    for (int frame = 5; frame <= 8; ++frame)
    {
        float offsetX = 0.05f * (8 - frame);  // 점점 0으로 줄어듦
        std::vector<std::array<float, 3>> shifted;
        shifted.reserve(points.size());
        for (const auto& p : points)
            shifted.push_back({ p[0] + offsetX, p[1], p[2] });

        odom2.addFrame(shifted);

        auto pos     = odom2.getPosition();
        auto ds_pts  = voxelGridFilter(shifted, 0.3f);

        // 루프 감지 시도
        // trajectory는 복사본 — detect()가 보정한 결과를 corrected에 저장
        auto corrected = odom2.getTrajectory();
        bool loopFound = loopCloser.detect(pos, ds_pts, corrected);

        if (!loopFound)
            loopCloser.addKeyFrame(frame, pos, ds_pts);
        else
        {
            std::cout << "[Loop Closure] 경로 보정 완료" << std::endl;

            // 보정 전후 경로 비교 출력
            std::cout << "\n--- Loop Closure 결과 ---" << std::endl;
            std::cout << "보정 전 최종 위치 : ("
                      << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
            std::cout << "보정 후 최종 위치 : ("
                      << corrected.back()[0] << ", "
                      << corrected.back()[1] << ", "
                      << corrected.back()[2] << ")" << std::endl;
            std::cout << "예상 최종 위치   : (0, 0, 0)" << std::endl;

            std::cout << "\n보정된 경로:" << std::endl;
            for (int i = 0; i < (int)corrected.size(); ++i)
                std::cout << "  프레임 " << i << " : ("
                          << corrected[i][0] << ", "
                          << corrected[i][1] << ", "
                          << corrected[i][2] << ")" << std::endl;
            break;
        }
    }


    // ── Map Builder 테스트 ───────────────────────────────
    // Odometry로 위치를 추적하면서 동시에 MapBuilder로 전역 맵을 누적해요.
    // 5프레임 전진하면서 각 프레임 포인트를 전역 좌표로 변환해 맵에 쌓고
    // 최종적으로 PLY 파일로 저장해요.

    std::cout << "\n--- Map Builder 테스트 ---" << std::endl;
    std::cout << "시뮬레이션: 5프레임 전진하며 전역 맵 누적" << std::endl;

    Odometry   odom3(0.3f);
    MapBuilder mapBuilder(0.3f, 5);

    for (int frame = 0; frame <= 4; ++frame)
    {
        float offsetX = 0.05f * frame;

        // 현재 프레임 포인트 생성
        std::vector<std::array<float, 3>> shifted;
        shifted.reserve(points.size());
        for (const auto& p : points)
            shifted.push_back({ p[0] + offsetX, p[1], p[2] });

        // Odometry로 위치 추적
        odom3.addFrame(shifted);

        // MapBuilder에 현재 프레임 추가
        auto ds_pts = voxelGridFilter(shifted, 0.3f);
        mapBuilder.addFrame(ds_pts, odom3.getRotation(), odom3.getPosition());

        std::cout << "[MapBuilder] 프레임 " << frame
                  << " | 맵 점 개수: " << mapBuilder.getPointCount()
                  << std::endl;
    }

    // 최종 맵 저장
    std::cout << "\n--- Map Builder 결과 ---" << std::endl;
    std::cout << "최종 맵 점 개수: " << mapBuilder.getPointCount() << std::endl;
    mapBuilder.saveToPly("output_map.ply");

    // ── Bag Parser + SLAM 파이프라인 ─────────────────────
    // 실제 LiDAR bag 파일을 프레임별로 읽어서
    // Odometry로 위치를 추적하고 MapBuilder로 전역 맵을 생성해요.

    std::cout << "\n--- Bag Parser + SLAM ---" << std::endl;

    BagParser  bag("/Users/deepfine/Downloads/T3F2-2021-08-02-15-00-12.bag", "/velodyne_points");
    Odometry   bagOdom(0.3f);
    MapBuilder bagMap(0.3f, 10);

    // 지면 구속: 평지 데이터 → z=0 고정, Yaw만 추적
    bagOdom.setGroundMode(true);

    // 루프 클로저: 반경 3m 안에 50 키프레임 이상 차이 나는 과거 위치 발견 시 보정
    // (5프레임마다 키프레임 추가 → 50 키프레임 = 250 프레임 ≈ 25초 간격)
    LoopCloser bagLoopCloser(3.0f, 0.15f, 50);
    int keyFrameId = 0;
    int loopCount  = 0;

    if (!bag.open())
    {
        std::cout << "bag 파일 열기 실패" << std::endl;
        return -1;
    }

    std::vector<std::array<float, 3>> framePoints;
    int frameIdx = 0;

    while (bag.nextFrame(framePoints))
    {
        // 1. Odometry로 위치 추적
        bagOdom.addFrame(framePoints);
        auto pos = bagOdom.getPosition();

        // 2. 전역 맵에 추가
        auto ds = voxelGridFilter(framePoints, 0.3f);
        bagMap.addFrame(ds, bagOdom.getRotation(), pos);

        // 3. 5프레임마다 루프 클로저 시도
        if (frameIdx % 5 == 0)
        {
            auto corrected = bagOdom.getTrajectory();
            if (bagLoopCloser.detect(pos, ds, corrected))
            {
                bagOdom.setTrajectory(corrected);
                bagOdom.setPosition(corrected.back());
                ++loopCount;
                std::cout << "[SLAM] ✅ 루프 클로저 보정! (누적 " << loopCount << "회)" << std::endl;
            }
            else
            {
                bagLoopCloser.addKeyFrame(keyFrameId++, pos, ds);
            }
        }

        std::cout << "[SLAM] 프레임 " << frameIdx
                  << " | 점 개수: "   << framePoints.size()
                  << " | 위치: ("     << pos[0] << ", "
                                      << pos[1] << ", "
                                      << pos[2] << ")"
                  << " | 맵 크기: "   << bagMap.getPointCount()
                  << std::endl;

        ++frameIdx;
    }

    bag.close();

    // 결과 저장
    bagMap.saveToPly("slam_map.ply");
    saveTrajectoryPly(bagOdom.getTrajectory(), "slam_trajectory.ply");
    saveMapImage(bagOdom.getTrajectory(), bagMap.getMap(), "slam_map_2d.ppm");

    std::cout << "\n--- SLAM 결과 ---" << std::endl;
    std::cout << "처리 프레임 수  : " << frameIdx << std::endl;
    std::cout << "루프 클로저 횟수: " << loopCount << std::endl;
    std::cout << "최종 맵 점 개수 : " << bagMap.getPointCount() << std::endl;
    std::cout << "최종 위치       : ("
              << bagOdom.getPosition()[0] << ", "
              << bagOdom.getPosition()[1] << ", "
              << bagOdom.getPosition()[2] << ")" << std::endl;

    return 0;
}