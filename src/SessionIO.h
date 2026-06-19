#pragma once
// 세션 디렉터리 입출력.
//   <dir>/submap_NNNN.ply : anchor-local 점군 (xyz ASCII)
//   <dir>/poses.txt       : 행마다 "idx anchorId R0..R8 tx ty tz"
//   <dir>/meta.txt        : name / count / voxel
#include <string>
#include "Session.h"

void    saveSession(const std::string& dir, const Session& session);
Session loadSession(const std::string& dir);
