#include "SubmapBuilder.h"

#include <cmath>
#include "PoseMath.h"
#include "VoxelGrid.h"

bool SubmapBuilder::addKeyframe(int nodeId,
                                const Pose3D& worldPose,
                                const std::vector<std::array<float,3>>& pointsSensor)
{
    _completed = false;

    if (!_active) {
        // 새 서브맵 시작: 첫 키프레임을 앵커로 삼는다.
        _active = true;
        _anchorId = nodeId;
        _anchorPose = worldPose;
        _accum.clear();
        _kfCount = 0;
    }

    // 센서-로컬 점 → world → 앵커 로컬.  T = inv(anchor) * keyframeWorld
    const Pose3D toAnchor = posemath::relativePoseOf(_anchorPose, worldPose);
    _accum.reserve(_accum.size() + pointsSensor.size());
    for (const auto& p : pointsSensor)
        _accum.push_back(posemath::transformPoint(toAnchor, p));

    ++_kfCount;

    if (_kfCount >= _keyframesPerSubmap) {
        finalize();
        return true;
    }
    return false;
}

void SubmapBuilder::finalize()
{
    Submap sm;
    sm.id       = _nextSubmapId++;
    sm.anchorId = _anchorId;
    sm.refPose  = _anchorPose;

    if (_downsampleVoxel > 0.0f) {
        sm.points = voxelGridFilter(_accum, _downsampleVoxel);
    } else {
        sm.points = _accum;
    }

    // AABB 계산
    float minx=1e30f, miny=1e30f, minz=1e30f;
    float maxx=-1e30f, maxy=-1e30f, maxz=-1e30f;
    for (const auto& p : sm.points) {
        minx = std::min(minx, p[0]); maxx = std::max(maxx, p[0]);
        miny = std::min(miny, p[1]); maxy = std::max(maxy, p[1]);
        minz = std::min(minz, p[2]); maxz = std::max(maxz, p[2]);
    }
    sm.aabb = {minx, miny, minz, maxx, maxy, maxz};

    _ready = std::move(sm);
    _completed = true;

    // 상태 초기화 (다음 서브맵 대기)
    _active = false;
    _accum.clear();
    _kfCount = 0;
}

bool SubmapBuilder::flush()
{
    if (_active && _kfCount > 0) {
        finalize();
        return true;
    }
    return false;
}

Submap SubmapBuilder::takeSubmap()
{
    _completed = false;
    return std::move(_ready);
}
