// KD-Tree, ICP, Voxel Grid Filter, Odometry, Loop Closure, Map Builder, Bag Parser 테스트 진입점
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <thread>
#include "PointCloud.h"
#include "PlyParser.h"
#include "KDTree.h"
#include "ICP.h"
#include "VoxelGrid.h"
#include "Odometry.h"
#include "LoopCloser.h"
#include "MapBuilder.h"
#include "BagParser.h"
#include "IMUPreintegrator.h"
#include "PangolinViewer.h"

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
    // ── 인자 파싱 ────────────────────────────────────────
    // 모드 1: PLY 테스트     ./slam <ply파일>
    // 모드 2: SLAM 실행      ./slam <bag파일> <포인트클라우드토픽> <IMU토픽>
    //                        예) ./slam data.bag /velodyne_points /imu
    if (argc < 2)
    {
        std::cout << "사용법 (토픽 확인): ./slam --info <bag파일>" << std::endl;
        std::cout << "사용법 (테스트):    ./slam <ply파일>" << std::endl;
        std::cout << "사용법 (SLAM):      ./slam <bag파일> <포인트토픽> <IMU토픽>" << std::endl;
        std::cout << "  예) ./slam data.bag /velodyne_points /imu" << std::endl;
        return -1;
    }

    // 실행 파일 위치 기준으로 output 폴더 경로 결정
    // argv[0] = ".../build/slam" → 부모 폴더의 output/
    std::string exePath(argv[0]);
    std::string exeDir = exePath.substr(0, exePath.find_last_of("/\\"));
    std::string outputDir = exeDir + "/../output/";

    // --info 모드: bag 안의 토픽 목록 출력 후 종료
    if (std::string(argv[1]) == "--info")
    {
        if (argc < 3)
        {
            std::cout << "사용법: ./slam --info <bag파일>" << std::endl;
            return -1;
        }
        BagParser info(argv[2], "", "");
        info.printTopics();
        return 0;
    }

    std::string firstArg = argv[1];
    bool isSlamMode = (argc >= 4);  // bag + 포인트토픽 + IMU토픽

    std::string bagPath      = isSlamMode ? argv[1] : "";
    std::string pointsTopic  = isSlamMode ? argv[2] : "";
    std::string imuTopic     = isSlamMode ? argv[3] : "";

    if (isSlamMode)
    {
        std::cout << "[SLAM 모드]" << std::endl;
        std::cout << "  bag 파일       : " << bagPath << std::endl;
        std::cout << "  포인트 토픽    : " << pointsTopic << std::endl;
        std::cout << "  IMU 토픽       : " << imuTopic << std::endl;
    }

    // PLY 테스트 모드에서만 사용
    PointCloud cloud;
    if (!isSlamMode)
    {
        if (!loadPly(argv[1], cloud))
        {
            std::cout << "PLY 로드 실패" << std::endl;
            return -1;
        }
        std::cout << "로드 완료: " << cloud.vertexCount << "개 점" << std::endl;
    }

    if (!isSlamMode)
    {
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

        std::array<float, 3> query = { points[0][0] + 0.1f,
                                       points[0][1] + 0.1f,
                                       points[0][2] + 0.1f };

        auto t2 = std::chrono::high_resolution_clock::now();
        auto kdResult = tree.nearest(query);
        auto t3 = std::chrono::high_resolution_clock::now();

        auto t4 = std::chrono::high_resolution_clock::now();
        auto bfResult = bruteForceNearest(points, query);
        auto t5 = std::chrono::high_resolution_clock::now();

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

        // 다운샘플링
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

        // ICP 테스트
        std::array<float, 3> trueT = { 0.05f, 0.03f, 0.02f };
        float angle = 1.0f * M_PI / 180.0f;
        Matrix3x3 trueR = {
             std::cos(angle), -std::sin(angle), 0,
             std::sin(angle),  std::cos(angle), 0,
             0,                0,               1
        };

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
        std::cout << "실제 이동값 : ("
                  << trueT[0] << ", " << trueT[1] << ", " << trueT[2] << ")" << std::endl;
        std::cout << "실제 회전각 : 1도 (Z축)" << std::endl;

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

        // Odometry 테스트
        std::cout << "\n--- Odometry 테스트 ---" << std::endl;
        Odometry odom(0.3f);
        odom.addFrame(points);
        for (int frame = 1; frame <= 4; ++frame)
        {
            std::vector<std::array<float, 3>> shifted;
            shifted.reserve(points.size());
            for (const auto& p : points)
                shifted.push_back({ p[0] + 0.05f * frame, p[1], p[2] });
            odom.addFrame(shifted);
        }
        auto finalPos = odom.getPosition();
        std::cout << "예상 최종 위치 : (0.2, 0, 0)" << std::endl;
        std::cout << "추정 최종 위치 : ("
                  << finalPos[0] << ", " << finalPos[1] << ", " << finalPos[2] << ")" << std::endl;

        const auto& traj = odom.getTrajectory();
        for (int i = 0; i < (int)traj.size(); ++i)
            std::cout << "  프레임 " << i << " : ("
                      << traj[i][0] << ", " << traj[i][1] << ", " << traj[i][2] << ")" << std::endl;

        // Loop Closure 테스트
        std::cout << "\n--- Loop Closure 테스트 ---" << std::endl;
        Odometry   odom2(0.3f);
        LoopCloser loopCloser(0.08f, 0.05f, 3);

        for (int frame = 0; frame <= 4; ++frame)
        {
            std::vector<std::array<float, 3>> shifted;
            shifted.reserve(points.size());
            for (const auto& p : points)
                shifted.push_back({ p[0] + 0.05f * frame, p[1], p[2] });
            odom2.addFrame(shifted);
            auto pos    = odom2.getPosition();
            auto ds_pts = voxelGridFilter(shifted, 0.3f);
            loopCloser.addKeyFrame(frame, pos, ds_pts);
        }
        for (int frame = 5; frame <= 8; ++frame)
        {
            std::vector<std::array<float, 3>> shifted;
            shifted.reserve(points.size());
            for (const auto& p : points)
                shifted.push_back({ p[0] + 0.05f * (8 - frame), p[1], p[2] });
            odom2.addFrame(shifted);
            auto pos       = odom2.getPosition();
            auto ds_pts    = voxelGridFilter(shifted, 0.3f);
            auto corrected = odom2.getTrajectory();
            if (loopCloser.detect(pos, ds_pts, corrected))
            {
                std::cout << "보정 전: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
                std::cout << "보정 후: (" << corrected.back()[0] << ", "
                          << corrected.back()[1] << ", " << corrected.back()[2] << ")" << std::endl;
                break;
            }
            loopCloser.addKeyFrame(frame, pos, ds_pts);
        }

        // Map Builder 테스트
        std::cout << "\n--- Map Builder 테스트 ---" << std::endl;
        Odometry   odom3(0.3f);
        MapBuilder mapBuilder(0.3f, 5);
        for (int frame = 0; frame <= 4; ++frame)
        {
            std::vector<std::array<float, 3>> shifted;
            shifted.reserve(points.size());
            for (const auto& p : points)
                shifted.push_back({ p[0] + 0.05f * frame, p[1], p[2] });
            odom3.addFrame(shifted);
            auto ds_pts = voxelGridFilter(shifted, 0.3f);
            mapBuilder.addFrame(ds_pts, odom3.getRotation(), odom3.getPosition());
            std::cout << "[MapBuilder] 프레임 " << frame
                      << " | 맵 점 개수: " << mapBuilder.getPointCount() << std::endl;
        }
        mapBuilder.saveToPly(outputDir + "output_map.ply");

        return 0;
    }

    // ── SLAM 파이프라인 (bag 모드) ───────────────────────
    std::cout << "\n--- Bag Parser + SLAM ---" << std::endl;

    BagParser        bag(bagPath, pointsTopic, imuTopic);
    Odometry         bagOdom(0.3f);
    MapBuilder       bagMap(0.3f, 10);
    IMUPreintegrator imu;

    bagOdom.setGroundMode(true);
    bagOdom.setImuPreintegrator(&imu);
    bagOdom.setLocalMapMaxPts(5000);   // ICP 대상 최대 점 수 — 많을수록 정확하지만 느림 (권장: 3000~8000)
    PangolinViewer viewer;

    LoopCloser bagLoopCloser(10.0f, 0.5f, 50);
    int keyFrameId = 0;
    int loopCount  = 0;
    int slamExitCode = 0;

    std::vector<std::array<float, 3>> framePoints;
    int frameIdx = 0;

    std::thread slamThread([&]() {
        if (!bag.open())
        {
            std::cout << "bag 파일 열기 실패: " << bagPath << std::endl;
            slamExitCode = -1;
            viewer.requestStop();
            return;
        }

        while (bag.nextFrame(framePoints) && !viewer.shouldQuit())
        {
            // 이번 라이다 프레임 이전까지 쌓인 IMU 샘플을 적분
            for (const auto& s : bag.getImuBuffer())
                imu.addSample(s);
            bag.clearImuBuffer();

            bagOdom.addFrame(framePoints);
            auto pos = bagOdom.getPosition();

            // Odometry 내부에서 이미 다운샘플링한 프레임을 재활용 — 중복 VoxelGrid 제거
            const auto& ds = bagOdom.getLastFrame();
            bagMap.addFrame(ds, bagOdom.getRotation(), pos);
            viewer.update(bagMap.getMap(), bagOdom.getTrajectory(), bagOdom.getRotation(), pos, frameIdx, 0.0f);

            if (frameIdx % 5 == 0)
            {
                auto corrected = bagOdom.getTrajectory();
                if (bagLoopCloser.detect(pos, ds, corrected))
                {
                    bagOdom.setTrajectory(corrected);
                    bagOdom.setPosition(corrected.back());
                    ++loopCount;
                    std::cout << "[SLAM] 루프 클로저 보정! (누적 " << loopCount << "회)" << std::endl;
                }
                else
                {
                    bagLoopCloser.addKeyFrame(keyFrameId++, pos, ds);
                }
            }

            std::cout << "[SLAM] 프레임 " << frameIdx
                      << " | 점 개수: "   << framePoints.size()
                      << " | 위치: ("     << pos[0] << ", " << pos[1] << ", " << pos[2] << ")"
                      << " | 맵 크기: "   << bagMap.getPointCount()
                      << std::endl;

            ++frameIdx;
        }

        bag.close();

        bagMap.saveToPly(outputDir + "slam_map.ply");
        saveTrajectoryPly(bagOdom.getTrajectory(), outputDir + "slam_trajectory.ply");
        saveMapImage(bagOdom.getTrajectory(), bagMap.getMap(), outputDir + "slam_map_2d.ppm");

        std::cout << "\n--- SLAM 결과 ---" << std::endl;
        std::cout << "처리 프레임 수  : " << frameIdx << std::endl;
        std::cout << "루프 클로저 횟수: " << loopCount << std::endl;
        std::cout << "최종 맵 점 개수 : " << bagMap.getPointCount() << std::endl;
        std::cout << "최종 위치       : ("
                  << bagOdom.getPosition()[0] << ", "
                  << bagOdom.getPosition()[1] << ", "
                  << bagOdom.getPosition()[2] << ")" << std::endl;

        viewer.requestStop();
    });

    viewer.runBlocking();
    slamThread.join();

    return slamExitCode;
}
