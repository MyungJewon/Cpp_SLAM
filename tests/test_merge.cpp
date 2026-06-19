// 멀티세션 병합 단위 테스트 (assert 기반, 외부 프레임워크 없음).
#include "Session.h"
#include "SessionIO.h"
#include "SessionMerger.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static void test_sessionio_roundtrip() {
    Session s; s.name="t"; s.voxelSize=0.2f;
    Submap m; m.id=0; m.anchorId=5;
    m.refPose = {{1,0,0, 0,1,0, 0,0,1}, {1.0f,2.0f,3.0f}};
    m.points = {{0.1f,0.2f,0.3f},{1.0f,1.0f,1.0f}};
    s.submaps.push_back(m);

    saveSession("/tmp/_sess_test", s);
    Session r = loadSession("/tmp/_sess_test");
    assert(r.submaps.size()==1);
    assert(r.submaps[0].anchorId==5);
    assert(r.submaps[0].points.size()==2);
    assert(std::fabs(r.submaps[0].refPose.t[0]-1.0f) < 1e-5f);
    assert(std::fabs(r.submaps[0].points[0][2]-0.3f) < 1e-5f);
    assert(std::fabs(r.voxelSize-0.2f) < 1e-5f);
    printf("SessionIO roundtrip OK\n");
}

static void test_cluster() {
    SessionMerger mg;
    std::vector<InterMatch> in;
    Pose3D I = {{1,0,0,0,1,0,0,0,1},{0,0,0}};
    Pose3D good = {{1,0,0,0,1,0,0,0,1},{5,0,0}};
    Pose3D bad  = {{1,0,0,0,1,0,0,0,1},{50,0,0}};
    for (int i=0;i<4;++i) in.push_back({i,i,I,good,0.7f,0.7f});
    in.push_back({9,9,I,bad,0.6f,0.6f});
    Pose3D T; std::vector<InterMatch> inl;
    bool ok = mg.bestCluster(in, T, inl);
    assert(ok);
    assert(inl.size()==4);
    assert(std::fabs(T.t[0]-5.0f) < 1e-3f);
    printf("cluster OK\n");
}

int main() {
    test_sessionio_roundtrip();
    test_cluster();
    printf("ALL TESTS PASSED\n");
    return 0;
}
