#include "SubmapRegistration.h"

#include <cmath>
#include "ICP.h"
#include "VoxelMap.h"
#include "PoseMath.h"

namespace {

// dst 점군을 voxelSize로 해시한 occupied 키 집합 (overlap 계산용)
std::unordered_set<VoxelKey, VoxelKeyHash> occupiedKeys(
    const std::vector<std::array<float,3>>& pts, float voxelSize)
{
    std::unordered_set<VoxelKey, VoxelKeyHash> keys;
    keys.reserve(pts.size());
    for (const auto& p : pts) {
        keys.insert({(int)std::floor(p[0]/voxelSize),
                     (int)std::floor(p[1]/voxelSize),
                     (int)std::floor(p[2]/voxelSize)});
    }
    return keys;
}

} // namespace

bool aabbOverlap(const Submap& src, const Submap& dst,
                 const Pose3D& guess, float margin)
{
    // src AABB의 8개 코너를 guess로 변환해 dst 로컬에서의 AABB를 만든 뒤 dst AABB와 겹침 검사.
    const auto& a = src.aabb;
    float minx=1e30f, miny=1e30f, minz=1e30f;
    float maxx=-1e30f, maxy=-1e30f, maxz=-1e30f;
    for (int i = 0; i < 8; ++i) {
        std::array<float,3> c = {
            (i & 1) ? a[3] : a[0],
            (i & 2) ? a[4] : a[1],
            (i & 4) ? a[5] : a[2]
        };
        auto t = posemath::transformPoint(guess, c);
        if (t[0] < minx) minx = t[0];
        if (t[1] < miny) miny = t[1];
        if (t[2] < minz) minz = t[2];
        if (t[0] > maxx) maxx = t[0];
        if (t[1] > maxy) maxy = t[1];
        if (t[2] > maxz) maxz = t[2];
    }
    const auto& b = dst.aabb;
    return (minx - margin) <= b[3] && (maxx + margin) >= b[0] &&
           (miny - margin) <= b[4] && (maxy + margin) >= b[1] &&
           (minz - margin) <= b[5] && (maxz + margin) >= b[2];
}

RegistrationResult registerSubmaps(const Submap& src,
                                   const Submap& dst,
                                   const Pose3D& initialGuess,
                                   const RegistrationParams& params)
{
    RegistrationResult result;
    result.relativePose = initialGuess;

    if (src.points.size() < 10 || dst.points.size() < 10)
        return result;

    // coarse-to-fine: 큰 voxel부터 작은 voxel로 순차 GICP. 각 단계 결과를 다음 단계 초기값으로.
    Matrix3x3 curR = initialGuess.R;
    std::array<float,3> curT = initialGuess.t;
    ICPResult icp{};
    bool any = false;
    float finestVoxel = params.finest();

    for (float res : params.resolutions) {
        VoxelMap dstMap(res);
        dstMap.setEigenFloor(params.eigenFloor);
        dstMap.insertAndUpdate(dst.points);
        if (dstMap.size() == 0)
            continue;

        std::vector<std::array<float,3>> dstVec;
        dstVec.reserve(dstMap.cells().size());
        for (const auto& kv : dstMap.cells())
            dstVec.push_back(kv.second.center);

        ICPResult step = runICP(src.points, dstVec, params.maxIterations, 1e-4f,
                                &curR, &curT, false, true,
                                &dstMap.cells(), res);

        // 단계 결과 유한성 확인 — 발산하면 이 단계는 버리고 이전 추정 유지
        bool finite = std::isfinite(step.error) &&
                      std::isfinite(step.t[0]) && std::isfinite(step.t[1]) && std::isfinite(step.t[2]);
        for (int i = 0; i < 9 && finite; ++i) finite = std::isfinite(step.R[i]);
        if (!finite)
            continue;

        curR = step.R;
        curT = step.t;
        icp = step;       // 최종(가장 미세) 단계의 fitness/error 사용
        any = true;
    }

    if (!any)
        return result;

    Pose3D rel = {icp.R, icp.t};

    // overlap: 최종 변환으로 src 점을 dst 로컬로 보내 dst occupied voxel에 들어간 비율
    auto dstKeys = occupiedKeys(dst.points, finestVoxel);
    int hit = 0;
    for (const auto& p : src.points) {
        auto tp = posemath::transformPoint(rel, p);
        VoxelKey k = {(int)std::floor(tp[0]/finestVoxel),
                      (int)std::floor(tp[1]/finestVoxel),
                      (int)std::floor(tp[2]/finestVoxel)};
        if (dstKeys.count(k)) ++hit;
    }
    float overlap = src.points.empty() ? 0.0f
                    : (float)hit / (float)src.points.size();

    result.relativePose = rel;
    result.fitness = icp.fitness;
    result.overlap = overlap;
    result.rmse    = icp.error;
    result.success = (icp.fitness >= params.minFitness) &&
                     (overlap     >= params.minOverlap) &&
                     (icp.error   <= params.maxRmse);
    return result;
}
