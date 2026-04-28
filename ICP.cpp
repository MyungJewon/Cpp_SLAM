// ICP 구현: 두 포인트 클라우드를 반복적으로 정렬해 이동/회전을 추정합니다.
#include "ICP.h"
#include <cmath>
#include <numeric>
#include <limits>

// ── 행렬 × 행렬 ─────────────────────────────────────
static Matrix3x3 multiplyMat(const Matrix3x3& A, const Matrix3x3& B)
{
    Matrix3x3 C = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C[i*3+j] += A[i*3+k] * B[k*3+j];
    return C;
}

// ── 행렬 × 벡터 ─────────────────────────────────────
static std::array<float, 3> multiplyVec(const Matrix3x3& R, const std::array<float, 3>& v)
{
    return {
        R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
        R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
        R[6]*v[0] + R[7]*v[1] + R[8]*v[2]
    };
}

// ── 중심점 계산 ──────────────────────────────────────
static std::array<float, 3> computeCentroid(const std::vector<std::array<float, 3>>& pts)
{
    std::array<float, 3> c = {0, 0, 0};
    for (const auto& p : pts)
    {
        c[0] += p[0];
        c[1] += p[1];
        c[2] += p[2];
    }
    c[0] /= pts.size();
    c[1] /= pts.size();
    c[2] /= pts.size();
    return c;
}

// ── SVD 근사로 회전행렬 계산 ─────────────────────────
// H = 공분산 행렬, 여기서 R을 추출합니다.
static Matrix3x3 computeSVD(const Matrix3x3& H)
{
    // 야코비 반복법으로 3x3 SVD 근사
    // U * S * V^T = H  →  R = V * U^T
    Matrix3x3 U = {1,0,0, 0,1,0, 0,0,1};
    Matrix3x3 V = {1,0,0, 0,1,0, 0,0,1};
    Matrix3x3 S = H;

    for (int iter = 0; iter < 100; ++iter)
    {
        for (int p = 0; p < 2; ++p)
        {
            for (int q = p+1; q < 3; ++q)
            {
                float Spq = S[p*3+q];
                float Spp = S[p*3+p];
                float Sqq = S[q*3+q];

                if (std::abs(Spq) < 1e-10f) continue;

                float theta = 0.5f * std::atan2(2.0f * Spq, Spp - Sqq);
                float c = std::cos(theta);
                float s = std::sin(theta);

                // 야코비 회전행렬 J
                Matrix3x3 J = {1,0,0, 0,1,0, 0,0,1};
                J[p*3+p] =  c; J[p*3+q] = -s;
                J[q*3+p] =  s; J[q*3+q] =  c;

                Matrix3x3 Jt = {1,0,0, 0,1,0, 0,0,1};
                Jt[p*3+p] =  c; Jt[p*3+q] =  s;
                Jt[q*3+p] = -s; Jt[q*3+q] =  c;

                S = multiplyMat(Jt, multiplyMat(S, J));
                U = multiplyMat(U, J);
                V = multiplyMat(V, J);
            }
        }
    }

    // det(V*U^T) 확인해서 반사 방지
    Matrix3x3 Ut = {
        U[0], U[3], U[6],
        U[1], U[4], U[7],
        U[2], U[5], U[8]
    };
    Matrix3x3 R = multiplyMat(V, Ut);

    float det = R[0]*(R[4]*R[8]-R[5]*R[7])
              - R[1]*(R[3]*R[8]-R[5]*R[6])
              + R[2]*(R[3]*R[7]-R[4]*R[6]);

    if (det < 0)
    {
        V[2] = -V[2]; V[5] = -V[5]; V[8] = -V[8];
        R = multiplyMat(V, Ut);
    }

    return R;
}

// ── ICP 메인 루프 ────────────────────────────────────
ICPResult runICP(const std::vector<std::array<float, 3>>& src,
                 const std::vector<std::array<float, 3>>& dst,
                 int maxIterations, float tolerance)
{
    ICPResult result;
    // 단위행렬로 초기화 (아무 변환도 안 한 상태)
    result.R = {1,0,0, 0,1,0, 0,0,1};
    result.t = {0, 0, 0};
    result.error = std::numeric_limits<float>::max();

    // dst로 KD-Tree 구축 (매 반복마다 가장 가까운 점 탐색용)
    KDTree tree;
    tree.build(dst);

    // 현재 변환이 적용된 src 복사본
    std::vector<std::array<float, 3>> current = src;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // 1. 각 점의 대응점 찾기
        std::vector<std::array<float, 3>> matched;
        float totalError = 0.0f;

        for (const auto& p : current)
        {
            auto nearest = tree.nearest(p);
            matched.push_back(nearest);

            float dx = p[0] - nearest[0];
            float dy = p[1] - nearest[1];
            float dz = p[2] - nearest[2];
            totalError += dx*dx + dy*dy + dz*dz;
        }
        totalError /= current.size();

        // 2. 수렴 확인
        if (std::abs(result.error - totalError) < tolerance)
        {
            result.iterations = iter;
            result.error = totalError;
            break;
        }
        result.error = totalError;

        // 3. 중심점 계산
        auto srcCentroid = computeCentroid(current);
        auto dstCentroid = computeCentroid(matched);

        // 4. 공분산 행렬 H 계산
        Matrix3x3 H = {};
        for (int i = 0; i < (int)current.size(); ++i)
        {
            // 중심점 기준으로 이동한 벡터
            std::array<float, 3> ps = {
                current[i][0] - srcCentroid[0],
                current[i][1] - srcCentroid[1],
                current[i][2] - srcCentroid[2]
            };
            std::array<float, 3> pd = {
                matched[i][0] - dstCentroid[0],
                matched[i][1] - dstCentroid[1],
                matched[i][2] - dstCentroid[2]
            };

            // H += ps * pd^T (외적)
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    H[r*3+c] += ps[r] * pd[c];
        }

        // 5. SVD로 회전행렬 계산
        Matrix3x3 R = computeSVD(H);

        // 6. 이동벡터 계산
        auto rotatedSrc = multiplyVec(R, srcCentroid);
        std::array<float, 3> t = {
            dstCentroid[0] - rotatedSrc[0],
            dstCentroid[1] - rotatedSrc[1],
            dstCentroid[2] - rotatedSrc[2]
        };

        // 7. current에 변환 적용
        for (auto& p : current)
        {
            p = multiplyVec(R, p);
            p[0] += t[0];
            p[1] += t[1];
            p[2] += t[2];
        }

        // 8. 누적 변환 갱신
        // 반드시 이전 result.t를 먼저 복사한 뒤 계산해야 해요.
        // 바로 result.t에 쓰면 아직 쓰지 않은 값이 덮어씌워집니다.
        result.R = multiplyMat(R, result.R);
        auto newT = multiplyVec(R, result.t);
        result.t[0] = newT[0] + t[0];
        result.t[1] = newT[1] + t[1];
        result.t[2] = newT[2] + t[2];

        result.iterations = iter + 1;
    }

    return result;
}