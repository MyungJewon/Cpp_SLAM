#pragma once
// SubmapBuilder: 실시간 SLAM에서 키프레임을 누적해 Submap을 만든다.
// 각 키프레임의 센서-로컬 점군을 "앵커 노드 로컬 좌표계"로 변환해 모은다.
#include "Submap.h"

class SubmapBuilder {
public:
    explicit SubmapBuilder(int keyframesPerSubmap = 10, float downsampleVoxel = 0.0f)
        : _keyframesPerSubmap(keyframesPerSubmap), _downsampleVoxel(downsampleVoxel) {}

    // 키프레임 추가.
    //   nodeId    : 이 키프레임에 대응하는 PoseGraph 노드 id
    //   worldPose : 이 키프레임 센서의 world pose (sensor-local → world)
    //   pointsSensor : 센서-로컬 좌표계 점군
    // 반환: 이번 추가로 서브맵이 완성되면 true. takeSubmap()으로 회수.
    bool addKeyframe(int nodeId,
                     const Pose3D& worldPose,
                     const std::vector<std::array<float,3>>& pointsSensor);

    // 완성된 서브맵 회수 (addKeyframe이 true 반환한 직후 호출).
    Submap takeSubmap();
    bool hasCompleted() const { return _completed; }

    // 처리 종료 시 남은 부분 서브맵을 강제로 마감 (점이 충분하면 반환, 아니면 success=false).
    bool flush();

private:
    void finalize();

    int   _keyframesPerSubmap;
    float _downsampleVoxel;

    // 진행 중인 서브맵 상태
    bool   _active = false;
    int    _anchorId = -1;
    Pose3D _anchorPose;            // 앵커(=첫 키프레임) world pose
    std::vector<std::array<float,3>> _accum;  // 앵커 로컬 좌표계 누적 점군
    int    _kfCount = 0;

    int    _nextSubmapId = 0;

    bool   _completed = false;
    Submap _ready;
};
