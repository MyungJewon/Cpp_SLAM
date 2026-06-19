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
#include "SubmapBuilder.h"
#include "MapBuilder.h"
#include "SessionIO.h"
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
    std::string sessionDir;  // --save-session <dir>: 멀티세션 병합용 세션 저장
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
        else if (a == "--save-session" && i + 1 < argc)
            sessionDir = argv[++i];
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
    Odometry         bagOdom(0.2f);
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

    // 서브맵 기반 연속 루프 클로저 (GLIM식). 키프레임을 누적해 서브맵을 만들고,
    // 현재 추정 위치 근방의 과거 서브맵과 GICP 정합 후 overlap 게이트를 통과하면 채택.
    // 다운샘플(0.15m)은 가장 미세한 등록 voxel(0.5m)보다 촘촘해야 한다 — 그래야 각
    // 0.5m voxel에 점이 6개 이상 모여 ICP의 공분산 게이트(point_count>=6)를 통과한다.
    // (GLIM은 0.25m를 쓰지만 GLIM의 VGICP는 이 6점 제약이 없음)
    // 작고 조밀한 서브맵 — 재방문 시 앵커가 가까이 겹쳐 깨끗한 small-dist 루프가 생긴다.
    // (크면 띠가 길어져 앵커가 어긋나고, 부분겹침으로 드리프트만 재확인하는 가짜 루프가 됨)
    SubmapBuilder submapBuilder(5, 0.15f);  // 키프레임 5개당 서브맵 1개, 0.15m 다운샘플
    LoopCloser bagLoopCloser;
    {
        RegistrationParams rp;
        rp.resolutions = {1.0f, 0.5f};  // coarse-to-fine 다해상도 GICP
        // overlap이 진짜/가짜 루프를 깔끔히 가르는 주 판별자(진짜 0.45+, 가짜 0.13-).
        // fitness는 본질적으로 더 빡빡(voxel 6점 게이트)해 진짜 루프를 죽이므로 floor만 둔다.
        rp.minFitness = 0.25f;
        rp.minOverlap = 0.4f;
        rp.maxRmse = 0.5f;
        bagLoopCloser.setRegistrationParams(rp);
        bagLoopCloser.setSearchRadius(12.0f);   // 앵커가 실제 겹치는 진짜 루프만 (부분겹침 배제)
        bagLoopCloser.setMinNodeGap(200);       // 인접 서브맵(~100 노드)은 루프에서 제외
        bagLoopCloser.setMaxCandidates(8);
    }
    bool poseGraphInited = false;
    Pose3D prevPose = {
        {1,0,0, 0,1,0, 0,0,1},
        {0,0,0}
    };
    int loopCount = 0;
    int slamExitCode = 0;
    std::vector<std::pair<int, int>> loopEdges;
    // 디스플레이용 궤적 — 포즈그래프 추정치를 반영해 루프가 닫히면 화면에서 스냅된다.
    // (front-end 내부 상태는 건드리지 않음)
    std::vector<std::array<float, 3>> displayTraj;
    // 루프 클로저 이벤트는 별도 파일로 — 프레임 로그 홍수에 묻히지 않게.
    std::ofstream loopLog(outputDir + "loops.log");
    loopLog << "# frame  from_anchor  to_anchor  fitness  overlap\n";

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

            // --imu: IMU 캘리브레이션(중력/자이로바이어스)이 끝나야 LIO 그래프를 시작한다.
            // 캘리브 전 프레임은 건너뛴다 (보통 데이터 시작부 정지 구간).
            if (useImuOdom && !imu.isCalibrated())
            {
                bag.clearImuBuffer();
                continue;
            }

            // 그래프가 활성화돼 있으면 직전~현재 사이 IMU 샘플을 preintegration에 누적
            // (그래프 init 전에는 no-op)
            if (useImuOdom)
                poseGraph.integrateImu(imuSamples);

            bag.clearImuBuffer();
            bagOdom.addFrame(framePoints, &pointTimes);
            if (!firstFrameDone)
                firstFrameDone = true;
            Pose3D currentPose = makePose(bagOdom.getRotation(), bagOdom.getPosition());

            if (!poseGraphInited)
            {
                // IMU 모드: enableImu는 반드시 init 전에 (init이 V0/B0 노드를 생성).
                if (useImuOdom)
                {
                    std::array<float, 3> gUp0;
                    std::array<float, 3> navG = {0.0f, 0.0f, -9.81f};
                    if (imu.getGravityUp(gUp0))
                        navG = {-9.81f*gUp0[0], -9.81f*gUp0[1], -9.81f*gUp0[2]};
                    poseGraph.enableImu(imu.getGyroBias(), 9.81, navG);
                    std::cout << "[SLAM] LIO 그래프 시작 (IMU 타이트커플링, nav중력 "
                              << navG[0] << "," << navG[1] << "," << navG[2] << ")" << std::endl;
                }
                poseGraph.init(currentPose);
                poseGraphInited = true;
            }
            else
            {
                Pose3D delta = relativePose(prevPose, currentPose);
                // GICP delta는 LiDAR 측정 BetweenFactor로, IMU는 CombinedImuFactor로
                // 같은 그래프에서 동시 최적화 (addOdometry 내부에서 처리).
                poseGraph.addOdometry(delta);
            }
            prevPose = currentPose;

            auto pos = currentPose.t;

            // 디스플레이 궤적에 이번 노드의 포즈그래프 추정치를 추가
            displayTraj.push_back(poseGraph.getPose(poseGraph.getCurrentId()).t);

            const auto& ds = bagOdom.getLastFrame();
            bagMap.addFrame(ds, bagOdom.getRotation(), pos);

            if (frameIdx % 20 == 0)
            {
                const int keyFrameId = poseGraph.getCurrentId();
                bagMap.storeKeyFramePoints(keyFrameId, ds, currentPose);

                // 키프레임을 서브맵에 누적. 서브맵 완성 시 LoopCloser에 등록(세션 저장용)
                // 하고, 루프 클로저가 켜져 있으면 보정도 적용한다.
                if ((useLoopClosure || !sessionDir.empty()) &&
                    submapBuilder.addKeyframe(keyFrameId, currentPose, ds))
                {
                    Submap completed = submapBuilder.takeSubmap();
                    const Pose3D anchorNow = poseGraph.getPose(completed.anchorId);
                    auto loops = bagLoopCloser.add(
                        completed, anchorNow,
                        [&](int id){ return poseGraph.getPose(id); },
                        useLoopClosure ? &loopLog : nullptr);

                    if (useLoopClosure)
                    for (const auto& lp : loops)
                    {
                        loopEdges.push_back({lp.fromId, lp.toId});
                        // 보정은 포즈그래프에만 누적한다. front-end(Odometry)의 위치/회전/로컬맵은
                        // 중간에 건드리지 않는다 — 위치만 끌어당기면 다음 스캔이 옛 프레임 로컬맵과
                        // 매칭돼 깨진다(teleportation 원인). odometry delta는 프레임 불변이라
                        // 포즈그래프 factor에는 문제없고, 궤적/맵은 종료 후 rebuildFromPoses로 일괄 보정.
                        // confidence = overlap×fitness 로 정합 신뢰도를 노이즈에 반영 (GLIM Hessian 근사).
                        float confidence = lp.overlap * lp.fitness;
                        poseGraph.addLoopClosure(lp.fromId, lp.toId, lp.relativePose, confidence);
                        ++loopCount;
                        std::cout << "[SLAM] 루프 클로저 보정! 앵커 "
                                  << lp.fromId << "<->" << lp.toId
                                  << " (fitness=" << lp.fitness << ", overlap=" << lp.overlap
                                  << ", 누적 " << loopCount << "회)" << std::endl;
                        loopLog << frameIdx << "  " << lp.fromId << "  " << lp.toId
                                << "  " << lp.fitness << "  " << lp.overlap << std::endl;
                    }

                    // 루프가 채택됐으면 디스플레이 궤적/맵을 최적화 결과로 즉시 갱신
                    // → Pangolin 화면에서 루프가 닫히는 순간이 보인다 (GLIM식).
                    // bagMap은 디스플레이/출력 전용이라 front-end 매칭(_localMap)과 무관.
                    if (useLoopClosure && !loops.empty())
                    {
                        const auto optimized = poseGraph.getAllPoses();
                        displayTraj = poseTrajectory(optimized);
                        bagMap.rebuildFromPoses(optimized);

                        // 루프 연결선 갱신 (어느 앵커끼리 묶였는지 시각화)
                        std::vector<std::pair<std::array<float,3>, std::array<float,3>>> segs;
                        for (const auto& e : loopEdges)
                            segs.push_back({poseGraph.getPose(e.first).t,
                                            poseGraph.getPose(e.second).t});
                        viewer.setLoopEdges(segs);
                    }
                }
            }

            const Pose3D curGraphPose = poseGraph.getPose(poseGraph.getCurrentId());
            viewer.update(bagMap.getMap(), displayTraj, curGraphPose.R, curGraphPose.t, frameIdx, 0.0f);

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
            displayTraj = poseTrajectory(finalPoses);  // 최종 디스플레이 갱신
            const Pose3D last = finalPoses.back();
            viewer.update(bagMap.getMap(), displayTraj, last.R, last.t, frameIdx, 0.0f);

            // 멀티세션 병합용 세션 저장: 서브맵 점(anchor-local) + 보정된 anchor 포즈.
            if (!sessionDir.empty())
            {
                Session sess;
                sess.name = "session";
                const auto& subs = bagLoopCloser.submaps();
                for (const auto& sm : subs)
                {
                    Submap m = sm;
                    if (m.anchorId >= 0 && m.anchorId < (int)finalPoses.size())
                        m.refPose = finalPoses[m.anchorId];  // 보정된 anchor 포즈
                    sess.submaps.push_back(std::move(m));
                }
                saveSession(sessionDir, sess);
                std::cout << "[SLAM] 세션 저장: " << sessionDir
                          << " (" << sess.submaps.size() << " 서브맵)" << std::endl;
            }
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

        // SLAM 처리는 끝났지만 창은 닫지 않는다 — 사용자가 결과를 확인하고
        // 직접 창을 닫을 때까지 뷰어를 유지한다.
        std::cout << "\n[SLAM] 처리 완료. 결과를 확인한 뒤 창을 닫으면 종료됩니다." << std::endl;
    });

    viewer.runBlocking();
    slamThread.join();

    return slamExitCode;
}
