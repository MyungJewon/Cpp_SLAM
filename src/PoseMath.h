#pragma once
// Pose3D(회전 R + 이동 t) 변환 수학 모음.
// 규약: Pose3D는 변환 p_out = R * p_in + t 를 의미한다.
//   - 서브맵/노드의 worldPose(refPose): local 좌표 → world 좌표
//   - composePose(A, B): T_A * T_B (먼저 B, 그 다음 A 적용)
//   - relativePoseOf(from, to): inv(from) * to (from 좌표계에서 본 to)
// 실시간 루프클로저와 오프라인 PLY 병합이 공통으로 사용한다.
#include <array>
#include <vector>
#include "PoseGraph.h"  // Pose3D, Matrix3x3

namespace posemath {

inline Matrix3x3 transpose(const Matrix3x3& A) {
    return {A[0], A[3], A[6],
            A[1], A[4], A[7],
            A[2], A[5], A[8]};
}

inline Matrix3x3 mul(const Matrix3x3& A, const Matrix3x3& B) {
    Matrix3x3 C = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C[i*3+j] += A[i*3+k] * B[k*3+j];
    return C;
}

inline std::array<float,3> mulVec(const Matrix3x3& R, const std::array<float,3>& v) {
    return {R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
            R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
            R[6]*v[0] + R[7]*v[1] + R[8]*v[2]};
}

// p_out = pose.R * p + pose.t
inline std::array<float,3> transformPoint(const Pose3D& pose, const std::array<float,3>& p) {
    auto rp = mulVec(pose.R, p);
    return {rp[0] + pose.t[0], rp[1] + pose.t[1], rp[2] + pose.t[2]};
}

// 회전이 직교(rigid)임을 가정한 역변환: inv(T).R = R^T, inv(T).t = -R^T t
inline Pose3D invert(const Pose3D& pose) {
    Matrix3x3 Rt = transpose(pose.R);
    auto nt = mulVec(Rt, pose.t);
    return {Rt, {-nt[0], -nt[1], -nt[2]}};
}

// T_A * T_B : 먼저 B, 그 다음 A
inline Pose3D compose(const Pose3D& a, const Pose3D& b) {
    Matrix3x3 R = mul(a.R, b.R);
    auto ab = mulVec(a.R, b.t);
    return {R, {ab[0] + a.t[0], ab[1] + a.t[1], ab[2] + a.t[2]}};
}

// inv(from) * to : from 좌표계 기준으로 본 to (BetweenFactor 측정값과 동일 규약)
inline Pose3D relativePoseOf(const Pose3D& from, const Pose3D& to) {
    return compose(invert(from), to);
}

inline Pose3D identity() {
    return {{1,0,0, 0,1,0, 0,0,1}, {0,0,0}};
}

} // namespace posemath
