#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "BagParser.h"
#include "ImuOdometry.h"
#include "IMUPreintegrator.h"
#include "LoopCloser.h"
#include "MapBuilder.h"
#include "Odometry.h"
#include "PangolinViewer.h"
#include "PoseGraph.h"

void printUsage()
{
    std::cout << "사용법 (토픽 확인): ./slam --info <bag파일>" << std::endl;
    std::cout << "사용법 (SLAM):      ./slam <bag파일> <포인트토픽> <IMU토픽> [옵션]" << std::endl;
    std::cout << "  예) ./slam data.bag /velodyne_points /imu" << std::endl;
    std::cout << "  옵션: --no-lc    루프 클로저 비활성화" << std::endl;
    std::cout << "        --imu     IMU 타이트 커플링 활성화" << std::endl;
    std::cout << "        --no-imu  IMU 타이트 커플링 비활성화 (기본값, 회전 디스큐잉만 사용)" << std::endl;
}

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

// 두 점 사이를 직선으로 그려서 buf에 색을 입힘 (DDA 알고리즘)
void drawLine(std::vector<uint8_t>& buf, int imgSize,
              int x0, int y0, int x1, int y1,
              uint8_t r, uint8_t g, uint8_t b)
{
    int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    if (steps == 0)
    {
        int idx = (y0 * imgSize + x0) * 3;
        buf[idx] = r; buf[idx + 1] = g; buf[idx + 2] = b;
        return;
    }
    for (int i = 0; i <= steps; ++i)
    {
        float t = (float)i / steps;
        int x = (int)(x0 + (x1 - x0) * t);
        int y = (int)(y0 + (y1 - y0) * t);
        if (x < 0 || x >= imgSize || y < 0 || y >= imgSize) continue;
        int idx = (y * imgSize + x) * 3;
        buf[idx] = r; buf[idx + 1] = g; buf[idx + 2] = b;
    }
}

void saveMapImage(const std::vector<std::array<float, 3>>& traj,
                  const std::vector<std::array<float, 3>>& mapPts,
                  const std::string& path,
                  const std::vector<std::pair<int, int>>& loopEdges = {},
                  int imgSize = 1024)
{
    if (traj.empty()) return;

    float minX = traj[0][0], maxX = traj[0][0];
    float minY = traj[0][1], maxY = traj[0][1];
    for (const auto& p : traj)
    {
        minX = std::min(minX, p[0]); maxX = std::max(maxX, p[0]);
        minY = std::min(minY, p[1]); maxY = std::max(maxY, p[1]);
    }
    for (const auto& p : mapPts)
    {
        minX = std::min(minX, p[0]); maxX = std::max(maxX, p[0]);
        minY = std::min(minY, p[1]); maxY = std::max(maxY, p[1]);
    }

    float range = std::max(maxX - minX, maxY - minY) * 1.1f + 0.1f;
    float cx = (minX + maxX) / 2.0f;
    float cy = (minY + maxY) / 2.0f;

    auto toPixel = [&](float x, float y) -> std::pair<int, int> {
        int px = (int)((x - cx) / range * imgSize + imgSize * 0.5f);
        int py = (int)(-(y - cy) / range * imgSize + imgSize * 0.5f);
        return {std::max(0, std::min(imgSize - 1, px)),
                std::max(0, std::min(imgSize - 1, py))};
    };

    std::vector<uint8_t> buf(imgSize * imgSize * 3, 25);

    for (const auto& p : mapPts)
    {
        auto [x, y] = toPixel(p[0], p[1]);
        int idx = (y * imgSize + x) * 3;
        buf[idx] = 60; buf[idx + 1] = 70; buf[idx + 2] = 80;
    }

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
            int nx = std::max(0, std::min(imgSize - 1, x + dx));
            int ny = std::max(0, std::min(imgSize - 1, y + dy));
            int idx = (ny * imgSize + nx) * 3;
            buf[idx] = r; buf[idx + 1] = g; buf[idx + 2] = 50;
        }
    }

    // 루프 클로저로 연결된 두 지점을 마젠타 선으로 표시 — 어떤 두 위치가
    // 잘못 매칭됐는지 한눈에 확인할 수 있음
    for (const auto& edge : loopEdges)
    {
        int fromId = edge.first, toId = edge.second;
        if (fromId < 0 || toId < 0 || fromId >= n || toId >= n) continue;
        auto [x0, y0] = toPixel(traj[fromId][0], traj[fromId][1]);
        auto [x1, y1] = toPixel(traj[toId][0], traj[toId][1]);
        drawLine(buf, imgSize, x0, y0, x1, y1, 255, 0, 255);
    }

    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << imgSize << " " << imgSize << "\n255\n";
    f.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());
    std::cout << "[Visualizer] 2D 맵 이미지 저장: " << path
              << " (" << imgSize << "x" << imgSize << "px)" << std::endl;
    std::cout << "             → macOS Finder에서 더블클릭하면 미리보기로 열려요" << std::endl;
}

Matrix3x3 transposeMat(const Matrix3x3& R)
{
    return {
        R[0], R[3], R[6],
        R[1], R[4], R[7],
        R[2], R[5], R[8]
    };
}

Matrix3x3 multiplyMat3(const Matrix3x3& A, const Matrix3x3& B)
{
    Matrix3x3 C = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C[i*3+j] += A[i*3+k] * B[k*3+j];
    return C;
}

std::array<float, 3> multiplyVec3(const Matrix3x3& R, const std::array<float, 3>& v)
{
    return {
        R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
        R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
        R[6]*v[0] + R[7]*v[1] + R[8]*v[2]
    };
}

Pose3D makePose(const Matrix3x3& R, const std::array<float, 3>& t)
{
    return {R, t};
}

Pose3D relativePose(const Pose3D& from, const Pose3D& to)
{
    Matrix3x3 invR = transposeMat(from.R);
    Matrix3x3 dR = multiplyMat3(invR, to.R);
    std::array<float, 3> dt = {
        to.t[0] - from.t[0],
        to.t[1] - from.t[1],
        to.t[2] - from.t[2]
    };
    return {dR, multiplyVec3(invR, dt)};
}

std::vector<std::array<float, 3>> poseTrajectory(const std::vector<Pose3D>& poses)
{
    std::vector<std::array<float, 3>> traj;
    traj.reserve(poses.size());
    for (const auto& pose : poses)
        traj.push_back(pose.t);
    return traj;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printUsage();
        return -1;
    }

    std::string exePath(argv[0]);
    size_t slashPos = exePath.find_last_of("/\\");
    std::string exeDir = (slashPos == std::string::npos) ? "." : exePath.substr(0, slashPos);
    std::string outputDir = exeDir + "/../output/";

    if (std::string(argv[1]) == "--info")
    {
        if (argc < 3)
        {
            printUsage();
            return -1;
        }
        BagParser info(argv[2], "", "");
        info.printTopics();
        return 0;
    }

    bool useLoopClosure = true;
    bool useImuOdom = false;
    std::vector<std::string> posArgs;
    for (int i = 1; i < argc; ++i)
    {
        std::string a(argv[i]);
        if (a == "--no-lc" || a == "-nlc")
            useLoopClosure = false;
        else if (a == "--imu")
            useImuOdom = true;
        else if (a == "--no-imu" || a == "-nimu")
            useImuOdom = false;
        else
            posArgs.push_back(a);
    }

    if (posArgs.size() < 3)
    {
        printUsage();
        return -1;
    }

    std::string bagPath = posArgs[0];
    std::string pointsTopic = posArgs[1];
    std::string imuTopic = posArgs[2];

    std::cout << "[SLAM 모드]" << std::endl;
    std::cout << "  bag 파일       : " << bagPath << std::endl;
    std::cout << "  포인트 토픽    : " << pointsTopic << std::endl;
    std::cout << "  IMU 토픽       : " << imuTopic << std::endl;
    std::cout << "  루프 클로저    : " << (useLoopClosure ? "ON" : "OFF (--no-lc)") << std::endl;
    std::cout << "  IMU 타이트커플링: " << (useImuOdom ? "ON (--imu)" : "OFF (기본값, 회전 디스큐잉만 사용)") << std::endl;
    std::cout << "\n--- Bag Parser + SLAM ---" << std::endl;

    BagParser        bag(bagPath, pointsTopic, imuTopic);
    Odometry         bagOdom(0.5f);
    MapBuilder       bagMap(0.3f, 10);
    PoseGraph        poseGraph;
    IMUPreintegrator imu;
    imu.calibrate();
    std::unique_ptr<ImuOdometry> imuOdom;
    bool firstFrameDone = false;

    bagOdom.setGroundMode(false);
    bagOdom.setMaxStepDist(3.5f);
    bagOdom.setImuPreintegrator(&imu);
    bagOdom.setLocalMapMaxPts(5000);
    bagOdom.setStationaryThresh(0.03f, 0.5f);
    PangolinViewer viewer;

    LoopCloser bagLoopCloser(5.0f, 0.2f, 50);
    bagLoopCloser.setLoopCooldown(100);
    bool poseGraphInited = false;
    Pose3D prevPose = {
        {1,0,0, 0,1,0, 0,0,1},
        {0,0,0}
    };
    int loopCount = 0;
    int slamExitCode = 0;
    std::vector<std::pair<int, int>> loopEdges;

    std::vector<std::array<float, 3>> framePoints;
    std::vector<float> pointTimes;
    int frameIdx = 0;
    bool imuCalibrationLogged = false;

    std::thread slamThread([&]() {
        if (!bag.open())
        {
            std::cout << "bag 파일 열기 실패: " << bagPath << std::endl;
            slamExitCode = -1;
            viewer.requestStop();
            return;
        }

        while (bag.nextFrame(framePoints, &pointTimes) && !viewer.shouldQuit())
        {
            const auto& imuSamples = bag.getImuBuffer();
            for (const auto& s : imuSamples)
                imu.addSample(s);

            if (!imuCalibrationLogged && imu.isCalibrated())
            {
                auto bias = imu.getGyroBias();
                std::cout << "[IMU] 캘리브레이션 완료 (자이로 바이어스: "
                          << bias[0] << ", " << bias[1] << ", " << bias[2]
                          << ")" << std::endl;
                imuCalibrationLogged = true;
            }

            if (useImuOdom && !imuOdom && imu.isCalibrated() && firstFrameDone)
            {
                imuOdom = std::make_unique<ImuOdometry>(imu.getGyroBias(), 9.81);
                imuOdom->init(bagOdom.getRotation(), bagOdom.getPosition());
                bagOdom.setImuOdometry(imuOdom.get());
            }

            if (imuOdom)
                imuOdom->integrateImu(imuSamples);

            bag.clearImuBuffer();
            bagOdom.addFrame(framePoints, &pointTimes);
            if (!firstFrameDone)
                firstFrameDone = true;
            Pose3D currentPose = makePose(bagOdom.getRotation(), bagOdom.getPosition());

            if (!poseGraphInited)
            {
                poseGraph.init(currentPose);
                poseGraphInited = true;
            }
            else
            {
                Pose3D delta = relativePose(prevPose, currentPose);
                // B1 자세 factor: 가속도계 중력이 이 데이터에서 신뢰 불가로 판명되어
                // 현재 비활성. 신뢰 가능한 IMU/캘리브레이션 확보 후 재활성.
                const bool useAttitudeFactor = false;
                std::array<float, 3> gUp;
                bool hasG = useAttitudeFactor && bagOdom.getLastGravityUp(gUp);
                poseGraph.addOdometry(delta, hasG ? &gUp : nullptr);
            }
            prevPose = currentPose;

            auto pos = currentPose.t;

            const auto& ds = bagOdom.getLastFrame();
            bagMap.addFrame(ds, bagOdom.getRotation(), pos);

            if (frameIdx % 20 == 0)
            {
                const int keyFrameId = poseGraph.getCurrentId();
                bagMap.storeKeyFramePoints(keyFrameId, ds, currentPose);
                float moveDist = 0.0f;
                if (!bagOdom.getTrajectory().empty())
                {
                    const auto& traj = bagOdom.getTrajectory();
                    int n = (int)traj.size();
                    int lookback = std::min(n - 1, 10);
                    if (lookback > 0)
                    {
                        float dx = traj[n-1][0] - traj[n-1-lookback][0];
                        float dy = traj[n-1][1] - traj[n-1-lookback][1];
                        moveDist = std::sqrt(dx*dx + dy*dy);
                    }
                }

                if (useLoopClosure && moveDist > 0.05f && bagLoopCloser.detect(keyFrameId, pos, currentPose.R, ds))
                {
                    loopEdges.push_back({bagLoopCloser.getLastLoopFromId(), bagLoopCloser.getLastLoopToId()});
                    // 맵 재구성은 여기서 하지 않음 — 중간 추정치로 전체를 다시 그리면
                    // 작은 보정 오차가 먼 거리 점에 크게 증폭됨. 트래젝토리만 갱신하고
                    // 맵 재구성은 모든 프레임 처리가 끝난 뒤 한 번만 수행한다.
                    const auto optimizedPoses = poseGraph.addLoopClosure(
                        bagLoopCloser.getLastLoopFromId(),
                        bagLoopCloser.getLastLoopToId(),
                        bagLoopCloser.getLastLoopRelativePose());
                    const auto optimizedTrajectory = poseTrajectory(optimizedPoses);
                    bagOdom.setTrajectory(optimizedTrajectory);
                    if (!optimizedPoses.empty())
                        bagOdom.setPosition(optimizedPoses.back().t);
                    ++loopCount;
                    std::cout << "[SLAM] 루프 클로저 보정! (누적 " << loopCount << "회)" << std::endl;
                }
                else
                {
                    bagLoopCloser.addKeyFrame(keyFrameId, pos, currentPose.R, ds);
                }
            }

            viewer.update(bagMap.getMap(), bagOdom.getTrajectory(), bagOdom.getRotation(), bagOdom.getPosition(), frameIdx, 0.0f);

            std::cout << "[SLAM] 프레임 " << frameIdx
                      << " | 점 개수: " << framePoints.size()
                      << " | 위치: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")"
                      << " | 맵 크기: " << bagMap.getPointCount()
                      << std::endl;

            ++frameIdx;
        }

        bag.close();

        // 모든 프레임 처리 후 포즈그래프 최적화 결과(자세 factor + 루프 포함)로
        // 궤적/맵을 항상 재구성한다. 루프가 없어도 자세 factor가 pitch/z 드리프트를
        // 보정하므로 front-end 궤적보다 우수하다.
        if (poseGraph.getCurrentId() > 0)
        {
            const auto finalPoses = poseGraph.getAllPoses();
            bagMap.rebuildFromPoses(finalPoses);
            bagOdom.setTrajectory(poseTrajectory(finalPoses));
        }

        bagMap.saveToPly(outputDir + "slam_map.ply");
        saveTrajectoryPly(bagOdom.getTrajectory(), outputDir + "slam_trajectory.ply");
        saveMapImage(bagOdom.getTrajectory(), bagMap.getMap(), outputDir + "slam_map_2d.ppm", loopEdges);

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
