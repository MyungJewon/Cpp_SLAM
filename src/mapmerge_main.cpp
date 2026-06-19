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

int main(int argc, char** argv) {
    std::vector<std::string> dirs;
    std::string out = "merged.ply";
    std::string saveDir;
    for (int i=1;i<argc;++i) {
        std::string a=argv[i];
        if (a=="-o" && i+1<argc) out=argv[++i];
        else if (a=="--save-session" && i+1<argc) saveDir=argv[++i];
        else dirs.push_back(a);
    }
    if (dirs.size()<2) {
        std::cerr << "usage: mapmerge <session_dir1> <session_dir2> [...] "
                     "-o merged.ply [--save-session <dir>]\n";
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

    SessionMerger merger;
    MergeResult r = merger.merge(sessions);
    if (!r.success) {
        std::cerr << "병합 실패: 세션 간 일관 정렬을 찾지 못했습니다.\n";
        return 2;
    }

    writePly(out, r.cloud);
    std::cout << "병합 완료: " << out << " (" << r.cloud.size() << " 점)\n";

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
