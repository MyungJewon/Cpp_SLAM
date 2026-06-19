#include "SessionMerger.h"

#include "ScanContext.h"
#include "PoseMath.h"
#include "VoxelGrid.h"
#include "KDTree.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

gtsam::Pose3 toG(const Pose3D& p) {
    const auto& R = p.R;
    return gtsam::Pose3(gtsam::Rot3(R[0],R[1],R[2], R[3],R[4],R[5], R[6],R[7],R[8]),
                        gtsam::Point3(p.t[0], p.t[1], p.t[2]));
}

Pose3D fromG(const gtsam::Pose3& g) {
    const gtsam::Matrix3 R = g.rotation().matrix();
    const gtsam::Point3 t = g.translation();
    return {{(float)R(0,0),(float)R(0,1),(float)R(0,2),
             (float)R(1,0),(float)R(1,1),(float)R(1,2),
             (float)R(2,0),(float)R(2,1),(float)R(2,2)},
            {(float)t.x(),(float)t.y(),(float)t.z()}};
}

gtsam::SharedNoiseModel diag6(double r, double t) {
    gtsam::Vector6 s; s << r,r,r, t,t,t;
    return gtsam::noiseModel::Diagonal::Sigmas(s);
}

float transDist(const Pose3D& a, const Pose3D& b) {
    float dx=a.t[0]-b.t[0], dy=a.t[1]-b.t[1], dz=a.t[2]-b.t[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

float rotDist(const Pose3D& a, const Pose3D& b) {
    float tr=0; for(int i=0;i<9;++i) tr += a.R[i]*b.R[i];  // trace(Ra^T Rb)는 elementwise 곱합
    float cosang=(tr-1.0f)*0.5f;
    cosang=std::max(-1.0f, std::min(1.0f, cosang));
    return std::acos(cosang);
}

std::vector<std::array<float,3>> removeOutliers(
        const std::vector<std::array<float,3>>& pts, int k, float stdMul) {
    if ((int)pts.size() <= k+1) return pts;
    KDTree tree; tree.build(pts);
    std::vector<float> meanDist(pts.size(), 0.0f);
    double sum=0.0, sum2=0.0;
    for (size_t i=0;i<pts.size();++i) {
        auto idx = tree.kNearestIdx(pts[i], k+1);  // 자기 자신 포함
        float d=0; int cnt=0;
        for (int j : idx) {
            if (j==(int)i) continue;
            float dx=pts[i][0]-pts[j][0], dy=pts[i][1]-pts[j][1], dz=pts[i][2]-pts[j][2];
            d += std::sqrt(dx*dx+dy*dy+dz*dz); ++cnt;
        }
        float m = cnt? d/cnt : 0.0f;
        meanDist[i]=m; sum+=m; sum2+=(double)m*m;
    }
    double mu=sum/pts.size();
    double var=std::max(0.0, sum2/pts.size()-mu*mu);
    double thr=mu+stdMul*std::sqrt(var);
    std::vector<std::array<float,3>> out; out.reserve(pts.size());
    for (size_t i=0;i<pts.size();++i)
        if (meanDist[i] <= thr) out.push_back(pts[i]);
    return out;
}

} // namespace

std::vector<InterMatch> SessionMerger::findMatches(const Session& ref,
                                                   const Session& src) const {
    std::vector<InterMatch> out;
    std::vector<ScanContextDesc> refDesc(ref.submaps.size());
    for (size_t i=0;i<ref.submaps.size();++i)
        refDesc[i]=computeScanContextCentered(ref.submaps[i].points);

    for (size_t j=0;j<src.submaps.size();++j) {
        ScanContextDesc d = computeScanContextCentered(src.submaps[j].points);
        std::vector<std::pair<float,int>> cand;
        cand.reserve(ref.submaps.size());
        for (size_t i=0;i<ref.submaps.size();++i)
            cand.push_back({scanContextDistance(d, refDesc[i]).first, (int)i});
        std::sort(cand.begin(), cand.end());

        for (int t=0; t<_p.topK && t<(int)cand.size(); ++t) {
            if (cand[t].first > _p.maxScDist) break;
            int i = cand[t].second;
            int shift = scanContextDistance(d, refDesc[i]).second;
            float yaw = shift * (2.0f*(float)M_PI / SC_SECTORS);
            float c=std::cos(yaw), s=std::sin(yaw);
            Pose3D guess = {{c,-s,0, s,c,0, 0,0,1},{0,0,0}};
            RegistrationResult r = registerSubmaps(src.submaps[j], ref.submaps[i],
                                                   guess, _p.reg);
            if (!r.success) continue;
            // T_SR = T_anchorRef · rel · inv(T_anchorS)
            Pose3D T_SR = posemath::compose(ref.submaps[i].refPose,
                          posemath::compose(r.relativePose,
                          posemath::invert(src.submaps[j].refPose)));
            out.push_back({i, (int)j, r.relativePose, T_SR, r.fitness, r.overlap});
        }
    }
    return out;
}

bool SessionMerger::bestCluster(const std::vector<InterMatch>& m,
                                Pose3D& outT,
                                std::vector<InterMatch>& inliers) const {
    int best=-1;
    std::vector<InterMatch> bestSet;
    for (size_t i=0;i<m.size();++i) {
        std::vector<InterMatch> set;
        for (size_t j=0;j<m.size();++j)
            if (transDist(m[i].T_SR,m[j].T_SR)<_p.clusterTransTol &&
                rotDist (m[i].T_SR,m[j].T_SR)<_p.clusterRotTol)
                set.push_back(m[j]);
        if ((int)set.size()>best) { best=(int)set.size(); bestSet=set; }
    }
    if (best < _p.minClusterSize) return false;
    Pose3D T = bestSet[0].T_SR;  // 회전: 소회전 가정으로 첫 인라이어 사용
    float tx=0,ty=0,tz=0;
    for (auto& e: bestSet) { tx+=e.T_SR.t[0]; ty+=e.T_SR.t[1]; tz+=e.T_SR.t[2]; }
    T.t = {tx/bestSet.size(), ty/bestSet.size(), tz/bestSet.size()};
    outT = T; inliers = bestSet;
    return true;
}

MergeResult SessionMerger::merge(const std::vector<Session>& sessions) const {
    MergeResult res;
    if (sessions.empty()) return res;

    const int N = (int)sessions.size();
    std::vector<int> offset(N, 0);          // 세션별 글로벌 노드 시작 id
    std::vector<Pose3D> sessionT(N, posemath::identity());  // T_SR (세션0=identity)
    std::vector<bool> accepted(N, false);
    accepted[0]=true;
    for (int sgi=1, acc=(int)sessions[0].submaps.size(); sgi<N; ++sgi) {
        offset[sgi]=acc; acc+=(int)sessions[sgi].submaps.size();
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    auto priorNoise = diag6(1e-6, 1e-6);
    auto odomNoise  = diag6(0.05, 0.1);

    // 세션0: 기준. 초기값 = 저장 refPose.
    for (size_t k=0;k<sessions[0].submaps.size();++k)
        initial.insert(gtsam::Symbol('x', offset[0]+(int)k),
                       toG(sessions[0].submaps[k].refPose));
    if (!sessions[0].submaps.empty())
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            gtsam::Symbol('x', offset[0]),
            toG(sessions[0].submaps[0].refPose), priorNoise));

    // 추가 세션 정렬 + 노드/엣지.
    std::vector<std::vector<InterMatch>> interInliers(N);
    for (int S=1; S<N; ++S) {
        auto matches = findMatches(sessions[0], sessions[S]);
        Pose3D T_SR; std::vector<InterMatch> inl;
        if (!bestCluster(matches, T_SR, inl)) {
            std::cerr << "[Merge] 세션 " << S << " 정렬 실패 (일관 매칭 "
                      << matches.size() << "건, 클러스터 부족) — 제외" << std::endl;
            continue;
        }
        accepted[S]=true; sessionT[S]=T_SR; interInliers[S]=inl;
        std::cout << "[Merge] 세션 " << S << " 정렬 OK (인라이어 " << inl.size()
                  << ", T_SR t=(" << T_SR.t[0] << "," << T_SR.t[1] << ","
                  << T_SR.t[2] << "))" << std::endl;
        for (size_t k=0;k<sessions[S].submaps.size();++k) {
            Pose3D init = posemath::compose(T_SR, sessions[S].submaps[k].refPose);
            initial.insert(gtsam::Symbol('x', offset[S]+(int)k), toG(init));
        }
    }

    // 세션 내 순차 엣지 (채택된 세션만).
    for (int S=0; S<N; ++S) {
        if (!accepted[S]) continue;
        const auto& subs = sessions[S].submaps;
        for (size_t k=0;k+1<subs.size();++k) {
            Pose3D rel = posemath::relativePoseOf(subs[k].refPose, subs[k+1].refPose);
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', offset[S]+(int)k),
                gtsam::Symbol('x', offset[S]+(int)k+1),
                toG(rel), odomNoise));
        }
    }

    // 세션 간 엣지 (인라이어). 측정 = relativePose (src로컬→ref로컬 = inv(T_ref)·T_src).
    for (int S=1; S<N; ++S) {
        if (!accepted[S]) continue;
        for (const auto& e : interInliers[S]) {
            float conf = std::min(std::max(e.overlap*e.fitness, 0.1f), 1.0f);
            auto loopNoise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(1.0),
                diag6(0.02/conf, 0.05/conf));
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', offset[0]+e.refIdx),
                gtsam::Symbol('x', offset[S]+e.srcIdx),
                toG(e.relativePose), loopNoise));
        }
    }

    // 최적화.
    gtsam::Values result;
    try {
        gtsam::LevenbergMarquardtParams lp;
        result = gtsam::LevenbergMarquardtOptimizer(graph, initial, lp).optimize();
    } catch (const std::exception& ex) {
        std::cerr << "[Merge] 최적화 실패 (" << ex.what()
                  << ") — 초기 정렬값으로 폴백" << std::endl;
        result = initial;
    }

    // 보정 포즈 추출 + 점군 누적.
    res.poses.resize(N);
    std::vector<std::array<float,3>> merged;
    for (int S=0; S<N; ++S) {
        if (!accepted[S]) { res.poses[S] = {}; continue; }
        res.poses[S].resize(sessions[S].submaps.size());
        for (size_t k=0;k<sessions[S].submaps.size();++k) {
            gtsam::Symbol sym('x', offset[S]+(int)k);
            Pose3D wp = result.exists(sym) ? fromG(result.at<gtsam::Pose3>(sym))
                                           : sessions[S].submaps[k].refPose;
            res.poses[S][k]=wp;
            for (const auto& p : sessions[S].submaps[k].points)
                merged.push_back(posemath::transformPoint(wp, p));
        }
    }

    merged = voxelGridFilter(merged, _p.outputVoxel);
    merged = removeOutliers(merged, _p.outlierK, _p.outlierStdMul);
    res.cloud = std::move(merged);
    res.success = true;
    return res;
}
