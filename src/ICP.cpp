// ICP 구현: 두 포인트 클라우드를 반복적으로 정렬해 이동/회전을 추정합니다.
#include "ICP.h"
#include <algorithm>
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

static std::array<float, 3> crossVec(const std::array<float, 3>& a,
                                     const std::array<float, 3>& b)
{
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

static float dotVec(const std::array<float, 3>& a,
                    const std::array<float, 3>& b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static Matrix3x3 rodrigues(const std::array<float, 3>& w)
{
    float theta = std::sqrt(dotVec(w, w));
    Matrix3x3 I = {1,0,0, 0,1,0, 0,0,1};
    if (theta < 1e-8f) return I;

    Matrix3x3 K = { 0,    -w[2],  w[1],
                    w[2],  0,    -w[0],
                   -w[1],  w[0],  0 };
    Matrix3x3 K2 = multiplyMat(K, K);

    float a = std::sin(theta) / theta;
    float b = (1.0f - std::cos(theta)) / (theta * theta);
    Matrix3x3 R = {};
    for (int i = 0; i < 9; ++i)
        R[i] = I[i] + a * K[i] + b * K2[i];
    return R;
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

static Matrix3x3 jacobiEigenvectors(Matrix3x3& M)
{
    Matrix3x3 V = {1,0,0, 0,1,0, 0,0,1};

    for (int iter = 0; iter < 100; ++iter)
    {
        int p = 0, q = 1;
        float maxOff = std::abs(M[0*3+1]);
        if (std::abs(M[0*3+2]) > maxOff) { maxOff = std::abs(M[0*3+2]); p=0; q=2; }
        if (std::abs(M[1*3+2]) > maxOff) { maxOff = std::abs(M[1*3+2]); p=1; q=2; }
        if (maxOff < 1e-12f) break;

        float Mpq = M[p*3+q];
        float Mpp = M[p*3+p];
        float Mqq = M[q*3+q];

        float theta = 0.5f * std::atan2(2.0f * Mpq, Mpp - Mqq);
        float c = std::cos(theta);
        float s = std::sin(theta);

        Matrix3x3 J  = {1,0,0, 0,1,0, 0,0,1};
        Matrix3x3 Jt = {1,0,0, 0,1,0, 0,0,1};
        J[p*3+p]  =  c;  J[p*3+q] = -s;
        J[q*3+p]  =  s;  J[q*3+q] =  c;
        Jt[p*3+p] =  c;  Jt[p*3+q] =  s;
        Jt[q*3+p] = -s;  Jt[q*3+q] =  c;

        M = multiplyMat(Jt, multiplyMat(M, J));
        V = multiplyMat(V, J);
    }

    return V;
}

static std::vector<std::array<float,3>> estimateNormals(
    const std::vector<std::array<float,3>>& pts, int k = 10)
{
    std::vector<std::array<float,3>> normals;
    normals.reserve(pts.size());

    KDTree tree;
    tree.build(pts);

    if (pts.size() < 3)
    {
        normals.assign(pts.size(), {0.0f, 0.0f, 1.0f});
        return normals;
    }

    int neighborCount = std::max(3, std::min(k, (int)pts.size()));
    for (const auto& p : pts)
    {
        std::vector<std::pair<float, int>> distances;
        distances.reserve(pts.size());
        for (int i = 0; i < (int)pts.size(); ++i)
        {
            float dx = pts[i][0] - p[0];
            float dy = pts[i][1] - p[1];
            float dz = pts[i][2] - p[2];
            distances.push_back({dx*dx + dy*dy + dz*dz, i});
        }

        if (neighborCount < (int)distances.size())
        {
            std::nth_element(distances.begin(),
                             distances.begin() + neighborCount,
                             distances.end());
        }

        std::array<float, 3> centroid = {0, 0, 0};
        for (int i = 0; i < neighborCount; ++i)
        {
            const auto& n = pts[distances[i].second];
            centroid[0] += n[0];
            centroid[1] += n[1];
            centroid[2] += n[2];
        }
        centroid[0] /= neighborCount;
        centroid[1] /= neighborCount;
        centroid[2] /= neighborCount;

        Matrix3x3 C = {};
        for (int i = 0; i < neighborCount; ++i)
        {
            const auto& n = pts[distances[i].second];
            std::array<float, 3> d = {
                n[0] - centroid[0],
                n[1] - centroid[1],
                n[2] - centroid[2]
            };
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    C[r*3+c] += d[r] * d[c];
        }

        Matrix3x3 eig = C;
        Matrix3x3 V = jacobiEigenvectors(eig);
        int minCol = 0;
        if (eig[4] < eig[minCol*3+minCol]) minCol = 1;
        if (eig[8] < eig[minCol*3+minCol]) minCol = 2;

        std::array<float, 3> normal = {
            V[0*3+minCol],
            V[1*3+minCol],
            V[2*3+minCol]
        };
        float len = std::sqrt(dotVec(normal, normal));
        if (len > 1e-8f)
        {
            normal[0] /= len;
            normal[1] /= len;
            normal[2] /= len;
        }
        else
        {
            normal = {0.0f, 0.0f, 1.0f};
        }
        normals.push_back(normal);
    }

    return normals;
}

static bool solveGaussian6x6(float A[6][6], float b[6], float x[6])
{
    float aug[6][7] = {};
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
            aug[r][c] = A[r][c];
        aug[r][6] = b[r];
    }

    for (int col = 0; col < 6; ++col)
    {
        int pivot = col;
        for (int r = col + 1; r < 6; ++r)
            if (std::abs(aug[r][col]) > std::abs(aug[pivot][col]))
                pivot = r;

        if (std::abs(aug[pivot][col]) < 1e-9f)
            return false;

        if (pivot != col)
        {
            for (int c = col; c < 7; ++c)
                std::swap(aug[col][c], aug[pivot][c]);
        }

        float div = aug[col][col];
        for (int c = col; c < 7; ++c)
            aug[col][c] /= div;

        for (int r = 0; r < 6; ++r)
        {
            if (r == col) continue;
            float factor = aug[r][col];
            for (int c = col; c < 7; ++c)
                aug[r][c] -= factor * aug[col][c];
        }
    }

    for (int i = 0; i < 6; ++i)
        x[i] = aug[i][6];
    return true;
}

static bool computePointToPlaneStep(const std::vector<std::array<float, 3>>& matchedSrc,
                                    const std::vector<std::array<float, 3>>& matchedDst,
                                    const std::vector<std::array<float, 3>>& matchedNormals,
                                    Matrix3x3& R,
                                    std::array<float, 3>& t)
{
    float A[6][6] = {};
    float b[6] = {};

    for (int i = 0; i < (int)matchedSrc.size(); ++i)
    {
        const auto& p = matchedSrc[i];
        const auto& q = matchedDst[i];
        const auto& n = matchedNormals[i];
        std::array<float, 3> pq = {p[0] - q[0], p[1] - q[1], p[2] - q[2]};
        float residual = dotVec(pq, n);
        auto pxn = crossVec(p, n);
        float J[6] = {pxn[0], pxn[1], pxn[2], n[0], n[1], n[2]};

        for (int r = 0; r < 6; ++r)
        {
            b[r] -= J[r] * residual;
            for (int c = 0; c < 6; ++c)
                A[r][c] += J[r] * J[c];
        }
    }

    for (int i = 0; i < 6; ++i)
        A[i][i] += 1e-6f;

    float x[6] = {};
    if (!solveGaussian6x6(A, b, x))
        return false;

    std::array<float, 3> w = {x[0], x[1], x[2]};
    R = rodrigues(w);
    t = {x[3], x[4], x[5]};
    return true;
}

// ── SVD로 회전행렬 계산 ─────────────────────────────
// H = 공분산 행렬 (비대칭), R = V * U^T 를 반환합니다.
//
// 이전 구현은 야코비 반복법을 H에 직접 적용했는데,
// 야코비는 대칭 행렬 전용입니다. H가 비대칭이면 U ≠ V 가 되어
// 회전행렬이 틀리게 나옵니다.
//
// 올바른 방법:
//   1. M = H^T * H (대칭 PSD) 에 야코비 적용 → 고유벡터 행렬 V 획득
//   2. U 열 = H * v_i / sigma_i  (sigma_i = sqrt(eigenvalue_i))
//   3. R = V * U^T,  det < 0 이면 U 마지막 열 부호 반전
static Matrix3x3 computeSVD(const Matrix3x3& H)
{
    // Step 1: M = H^T * H  (대칭 PSD)
    Matrix3x3 Ht = { H[0], H[3], H[6],
                     H[1], H[4], H[7],
                     H[2], H[5], H[8] };
    Matrix3x3 M = multiplyMat(Ht, H);

    // Step 2: 대칭 행렬 M에 야코비 반복 → V, 대각(고유값)
    Matrix3x3 V = {1,0,0, 0,1,0, 0,0,1};

    for (int iter = 0; iter < 100; ++iter)
    {
        // 비대각 원소 중 절대값이 가장 큰 (p,q) 선택
        int p = 0, q = 1;
        float maxOff = std::abs(M[0*3+1]);
        if (std::abs(M[0*3+2]) > maxOff) { maxOff = std::abs(M[0*3+2]); p=0; q=2; }
        if (std::abs(M[1*3+2]) > maxOff) { maxOff = std::abs(M[1*3+2]); p=1; q=2; }
        if (maxOff < 1e-12f) break;

        float Mpq = M[p*3+q];
        float Mpp = M[p*3+p];
        float Mqq = M[q*3+q];

        float theta = 0.5f * std::atan2(2.0f * Mpq, Mpp - Mqq);
        float c = std::cos(theta);
        float s = std::sin(theta);

        Matrix3x3 J  = {1,0,0, 0,1,0, 0,0,1};
        Matrix3x3 Jt = {1,0,0, 0,1,0, 0,0,1};
        J[p*3+p]  =  c;  J[p*3+q] = -s;
        J[q*3+p]  =  s;  J[q*3+q] =  c;
        Jt[p*3+p] =  c;  Jt[p*3+q] =  s;
        Jt[q*3+p] = -s;  Jt[q*3+q] =  c;

        M = multiplyMat(Jt, multiplyMat(M, J));
        V = multiplyMat(V, J);
    }

    // Step 3: U 열 계산  u_i = H * v_i / sigma_i
    // V 열 벡터: V[row*3 + col]  →  v_i = (V[0,i], V[1,i], V[2,i])
    Matrix3x3 U = {};
    for (int i = 0; i < 3; ++i)
    {
        float sigma = std::sqrt(std::max(0.0f, M[i*3+i]));
        std::array<float,3> vi = { V[0*3+i], V[1*3+i], V[2*3+i] };
        auto ui = multiplyVec(H, vi);
        if (sigma > 1e-8f)
        {
            ui[0] /= sigma;
            ui[1] /= sigma;
            ui[2] /= sigma;
        }
        else if (i == 2)
        {
            // 특이값이 0에 가까운 경우: u0 × u1 로 직교 완성
            std::array<float,3> u0 = { U[0], U[3], U[6] };
            std::array<float,3> u1 = { U[1], U[4], U[7] };
            ui = { u0[1]*u1[2] - u0[2]*u1[1],
                   u0[2]*u1[0] - u0[0]*u1[2],
                   u0[0]*u1[1] - u0[1]*u1[0] };
        }
        U[0*3+i] = ui[0];
        U[1*3+i] = ui[1];
        U[2*3+i] = ui[2];
    }

    // Step 4: R = V * U^T
    Matrix3x3 Ut = { U[0], U[3], U[6],
                     U[1], U[4], U[7],
                     U[2], U[5], U[8] };
    Matrix3x3 R = multiplyMat(V, Ut);

    // Step 5: 반사 보정 — det(R) = -1 이면 U 마지막 열 부호 반전
    float det = R[0]*(R[4]*R[8]-R[5]*R[7])
              - R[1]*(R[3]*R[8]-R[5]*R[6])
              + R[2]*(R[3]*R[7]-R[4]*R[6]);

    if (det < 0)
    {
        U[0*3+2] = -U[0*3+2];
        U[1*3+2] = -U[1*3+2];
        U[2*3+2] = -U[2*3+2];
        Ut = { U[0], U[3], U[6],
               U[1], U[4], U[7],
               U[2], U[5], U[8] };
        R = multiplyMat(V, Ut);
    }

    return R;
}

// ── ICP 메인 루프 ────────────────────────────────────
ICPResult runICP(const std::vector<std::array<float, 3>>& src,
                 const std::vector<std::array<float, 3>>& dst,
                 int maxIterations, float tolerance,
                 const Matrix3x3* initialR,
                 const std::array<float, 3>* initialT,
                 bool usePointToPlane)
{
    ICPResult result;
    result.R = {1,0,0, 0,1,0, 0,0,1};
    result.t = {0, 0, 0};
    result.error = std::numeric_limits<float>::max();

    // dst로 KD-Tree 구축
    KDTree tree;
    tree.build(dst);
    std::vector<std::array<float, 3>> dstNormals;
    if (usePointToPlane)
        dstNormals = estimateNormals(dst);

    // 초기 변환 적용 (IMU 예측값 등)
    std::vector<std::array<float, 3>> current = src;
    if (initialR)
    {
        std::array<float, 3> t0 = initialT ? *initialT : std::array<float,3>{0,0,0};
        for (auto& p : current)
        {
            p = multiplyVec(*initialR, p);
            p[0] += t0[0]; p[1] += t0[1]; p[2] += t0[2];
        }
        result.R = *initialR;
        result.t = t0;
    }

    // 대응점 최대 거리: 복셀 크기의 3배를 기본값으로 사용
    // 이 거리보다 먼 대응점은 아웃라이어로 버립니다.
    const float maxDistSq = 1.0f * 1.0f;  // 1.0m (복셀 0.3m 기준 3배)

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // 1. 각 점의 대응점 찾기 — 거리 제한 초과 시 버림
        std::vector<std::array<float, 3>> matchedSrc;
        std::vector<std::array<float, 3>> matchedDst;
        std::vector<std::array<float, 3>> matchedNormals;
        float totalError = 0.0f;

        for (const auto& p : current)
        {
            std::array<float, 3> nearest;
            int nearestIdx = -1;
            if (usePointToPlane)
            {
                nearestIdx = tree.nearestIdx(p);
                if (nearestIdx < 0) continue;
                nearest = dst[nearestIdx];
            }
            else
            {
                nearest = tree.nearest(p);
            }

            float dx = p[0] - nearest[0];
            float dy = p[1] - nearest[1];
            float dz = p[2] - nearest[2];
            float distSq = dx*dx + dy*dy + dz*dz;

            if (distSq > maxDistSq) continue;  // 아웃라이어 제거

            matchedSrc.push_back(p);
            matchedDst.push_back(nearest);
            if (usePointToPlane)
                matchedNormals.push_back(dstNormals[nearestIdx]);
            totalError += distSq;
        }

        // 유효 대응점이 너무 적으면 중단
        if (matchedSrc.size() < 10) break;

        totalError /= matchedSrc.size();

        // 2. 수렴 확인
        if (std::abs(result.error - totalError) < tolerance)
        {
            result.iterations = iter;
            result.error = totalError;
            break;
        }
        result.error = totalError;

        Matrix3x3 R = {1,0,0, 0,1,0, 0,0,1};
        std::array<float, 3> t = {0, 0, 0};

        if (usePointToPlane)
        {
            if (!computePointToPlaneStep(matchedSrc, matchedDst, matchedNormals, R, t))
                break;
        }
        else
        {
            // 3. 중심점 계산
            auto srcCentroid = computeCentroid(matchedSrc);
            auto dstCentroid = computeCentroid(matchedDst);

            // 4. 공분산 행렬 H 계산
            Matrix3x3 H = {};
            for (int i = 0; i < (int)matchedSrc.size(); ++i)
            {
                std::array<float, 3> ps = {
                    matchedSrc[i][0] - srcCentroid[0],
                    matchedSrc[i][1] - srcCentroid[1],
                    matchedSrc[i][2] - srcCentroid[2]
                };
                std::array<float, 3> pd = {
                    matchedDst[i][0] - dstCentroid[0],
                    matchedDst[i][1] - dstCentroid[1],
                    matchedDst[i][2] - dstCentroid[2]
                };

                // H += ps * pd^T (외적)
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        H[r*3+c] += ps[r] * pd[c];
            }

            // 5. SVD로 회전행렬 계산
            R = computeSVD(H);

            // 6. 이동벡터 계산
            auto rotatedSrc = multiplyVec(R, srcCentroid);
            t = {
                dstCentroid[0] - rotatedSrc[0],
                dstCentroid[1] - rotatedSrc[1],
                dstCentroid[2] - rotatedSrc[2]
            };
        }

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
