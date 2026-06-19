#include "SessionIO.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {

std::string submapFile(int i) {
    std::ostringstream o;
    o << "submap_" << std::setfill('0') << std::setw(4) << i << ".ply";
    return o.str();
}

// 단순 xyz ASCII PLY 저장 (MapBuilder와 동일 포맷).
void writePlyXYZ(const std::string& path,
                 const std::vector<std::array<float, 3>>& pts) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("PLY 저장 실패: " + path);
    f << "ply\nformat ascii 1.0\nelement vertex " << pts.size()
      << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
    for (const auto& p : pts)
        f << p[0] << " " << p[1] << " " << p[2] << "\n";
}

// xyz ASCII PLY 로드 (end_header 이후 앞 3컬럼만 사용).
std::vector<std::array<float, 3>> readPlyXYZ(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("PLY 로드 실패: " + path);
    std::string line;
    bool body = false;
    std::vector<std::array<float, 3>> pts;
    while (std::getline(f, line)) {
        if (!body) {
            if (line.rfind("end_header", 0) == 0) body = true;
            continue;
        }
        std::istringstream ss(line);
        float x, y, z;
        if (ss >> x >> y >> z) pts.push_back({x, y, z});
    }
    return pts;
}

void computeAabb(Submap& m) {
    if (m.points.empty()) { m.aabb = {0, 0, 0, 0, 0, 0}; return; }
    std::array<float, 6> b = {m.points[0][0], m.points[0][1], m.points[0][2],
                              m.points[0][0], m.points[0][1], m.points[0][2]};
    for (const auto& p : m.points)
        for (int d = 0; d < 3; ++d) {
            b[d]     = std::min(b[d], p[d]);
            b[d + 3] = std::max(b[d + 3], p[d]);
        }
    m.aabb = b;
}

} // namespace

void saveSession(const std::string& dir, const Session& session) {
    fs::create_directories(dir);
    std::ofstream poses(dir + "/poses.txt");
    if (!poses) throw std::runtime_error("poses.txt 저장 실패: " + dir);
    poses << std::setprecision(9);
    for (size_t i = 0; i < session.submaps.size(); ++i) {
        const Submap& m = session.submaps[i];
        writePlyXYZ(dir + "/" + submapFile((int)i), m.points);
        poses << i << " " << m.anchorId;
        for (int k = 0; k < 9; ++k) poses << " " << m.refPose.R[k];
        poses << " " << m.refPose.t[0] << " " << m.refPose.t[1] << " "
              << m.refPose.t[2] << "\n";
    }
    std::ofstream meta(dir + "/meta.txt");
    meta << "name " << (session.name.empty() ? "session" : session.name) << "\n"
         << "count " << session.submaps.size() << "\n"
         << "voxel " << session.voxelSize << "\n";
}

Session loadSession(const std::string& dir) {
    Session s;
    std::ifstream meta(dir + "/meta.txt");
    std::string key;
    while (meta >> key) {
        if (key == "name")        meta >> s.name;
        else if (key == "voxel")  meta >> s.voxelSize;
        else { std::string skip; std::getline(meta, skip); }
    }

    std::ifstream poses(dir + "/poses.txt");
    if (!poses) throw std::runtime_error("poses.txt 없음: " + dir);
    std::string line;
    while (std::getline(poses, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        int idx, anchorId;
        if (!(ss >> idx >> anchorId)) continue;
        Submap m;
        m.id = idx;
        m.anchorId = anchorId;
        for (int k = 0; k < 9; ++k) ss >> m.refPose.R[k];
        ss >> m.refPose.t[0] >> m.refPose.t[1] >> m.refPose.t[2];
        m.points = readPlyXYZ(dir + "/" + submapFile(idx));
        computeAabb(m);
        s.submaps.push_back(std::move(m));
    }
    return s;
}
