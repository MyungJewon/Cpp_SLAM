// mapmerge: 저장된 세션 N개를 하나의 맵으로 병합한다.
//   mapmerge <session_dir1> <session_dir2> [...] -o merged.ply [--save-session <dir>]
#include "SessionIO.h"
#include "SessionMerger.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static void writePly(const std::string& path,
                     const std::vector<std::array<float,3>>& pts) {
    std::ofstream f(path);
    f << "ply\nformat ascii 1.0\nelement vertex " << pts.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    for (const auto& p : pts) f << p[0] << " " << p[1] << " " << p[2] << "\n";
}

// 세션별 색상 PLY (정렬 검증용): 세션0=빨강, 1=파랑, 2=초록, 3=노랑...
static void writeColoredPly(const std::string& path,
                            const std::vector<std::array<float,3>>& pts,
                            const std::vector<int>& sid) {
    static const unsigned char pal[][3] = {
        {220,40,40}, {40,90,220}, {40,200,80}, {230,200,40},
        {200,60,200}, {40,200,200}};
    std::ofstream f(path);
    f << "ply\nformat ascii 1.0\nelement vertex " << pts.size()
      << "\nproperty float x\nproperty float y\nproperty float z\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (size_t i=0;i<pts.size();++i) {
        const auto* c = pal[(i<sid.size()?sid[i]:0) % 6];
        f << pts[i][0] << " " << pts[i][1] << " " << pts[i][2] << " "
          << (int)c[0] << " " << (int)c[1] << " " << (int)c[2] << "\n";
    }
}

int main(int argc, char** argv) {
    std::vector<std::string> dirs;
    std::string out = "merged.ply";
    std::string saveDir;
    bool colored = false;
    MergeParams mp;
    for (int i=1;i<argc;++i) {
        std::string a=argv[i];
        if (a=="-o" && i+1<argc) out=argv[++i];
        else if (a=="--save-session" && i+1<argc) saveDir=argv[++i];
        else if (a=="--min-cluster" && i+1<argc) mp.minClusterSize=std::stoi(argv[++i]);
        else if (a=="--trans-tol" && i+1<argc) mp.clusterTransTol=std::stof(argv[++i]);
        else if (a=="--rot-tol" && i+1<argc) mp.clusterRotTol=std::stof(argv[++i]);
        else if (a=="--max-scdist" && i+1<argc) mp.maxScDist=std::stof(argv[++i]);
        else if (a=="--min-overlap" && i+1<argc) mp.reg.minOverlap=std::stof(argv[++i]);
        else if (a=="--min-fitness" && i+1<argc) mp.reg.minFitness=std::stof(argv[++i]);
        else if (a=="--topk" && i+1<argc) mp.topK=std::stoi(argv[++i]);
        else if (a=="--output-voxel" && i+1<argc) mp.outputVoxel=std::stof(argv[++i]);
        else if (a=="--outlier-std" && i+1<argc) mp.outlierStdMul=std::stof(argv[++i]);
        else if (a=="--no-outlier") mp.outlierStdMul=0.0f;  // 아웃라이어 제거 비활성
        else if (a=="--coarse-voxel" && i+1<argc) mp.coarseVoxel=std::stof(argv[++i]);
        else if (a=="--coarse-fitness" && i+1<argc) mp.coarseMinFitness=std::stof(argv[++i]);
        else if (a=="--coarse-overlap" && i+1<argc) mp.coarseMinOverlap=std::stof(argv[++i]);
        else if (a=="--no-coarse") mp.coarseAlign=false;
        else if (a=="--colored") colored=true;
        else dirs.push_back(a);
    }
    if (dirs.size()<2) {
        std::cerr << "usage: mapmerge <session_dir1> <session_dir2> [...] -o merged.ply\n"
                     "       [--save-session <dir>] [--min-cluster N] [--trans-tol M]\n"
                     "       [--rot-tol RAD] [--max-scdist D] [--min-overlap O]\n"
                     "       [--min-fitness F] [--topk K]\n";
        return 1;
    }

    std::vector<Session> sessions;
    for (const auto& d : dirs) {
        try { sessions.push_back(loadSession(d)); }
        catch (const std::exception& e) {
            std::cerr << "세션 로드 실패: " << d << " (" << e.what() << ")\n";
            return 1;
        }
        std::cout << "로드: " << d << " (" << sessions.back().submaps.size()
                  << " 서브맵)\n";
    }

    SessionMerger merger(mp);
    MergeResult r = merger.merge(sessions);
    if (!r.success) {
        std::cerr << "병합 실패: 세션 간 일관 정렬을 찾지 못했습니다.\n";
        return 2;
    }

    writePly(out, r.cloud);
    std::cout << "병합 완료: " << out << " (" << r.cloud.size() << " 점)\n";

    if (colored) {
        std::string cpath = out.substr(0, out.find_last_of('.')) + "_colored.ply";
        writeColoredPly(cpath, r.cloud, r.sessionId);
        std::cout << "색상 검증 출력: " << cpath
                  << " (세션0=빨강, 세션1=파랑, ...)\n";
    }

    if (!saveDir.empty()) {
        Session merged; merged.name="merged";
        // 병합 결과를 단일 세션(서브맵 1개)으로 저장 — N개 체이닝용.
        Submap m; m.id=0; m.anchorId=0;
        m.refPose = {{1,0,0,0,1,0,0,0,1},{0,0,0}};
        m.points = r.cloud;
        merged.submaps.push_back(std::move(m));
        saveSession(saveDir, merged);
        std::cout << "병합 세션 저장: " << saveDir << "\n";
    }
    return 0;
}
