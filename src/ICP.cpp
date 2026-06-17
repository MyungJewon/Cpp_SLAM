#include "ICP.h"
#include "VoxelMap.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <limits>

static Matrix3x3 multiplyMat(const Matrix3x3& A, const Matrix3x3& B)
{
    Matrix3x3 C = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C[i*3+j] += A[i*3+k] * B[k*3+j];
    return C;
}

static std::array<float, 3> multiplyVec(const Matrix3x3& R, const std::array<float, 3>& v)
{
    return {
        R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
        R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
        R[6]*v[0] + R[7]*v[1] + R[8]*v[2]
    };
}

static Matrix3x3 transposeMat(const Matrix3x3& R)
{
    return {
        R[0], R[3], R[6],
        R[1], R[4], R[7],
        R[2], R[5], R[8]
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

[[maybe_unused]] static size_t voxelHash(int ix, int iy, int iz)
{
    return (size_t)((int64_t)ix * 73856093) ^
           (size_t)((int64_t)iy * 19349663) ^
           (size_t)((int64_t)iz * 83492791);
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

static Matrix3x3 transformCovariance(const Matrix3x3& R, const Matrix3x3& C)
{
    return multiplyMat(multiplyMat(R, C), transposeMat(R));
}

static bool invertSymmetric3x3(const Matrix3x3& A, Matrix3x3& inv)
{
    float a = A[0], b = A[1], c = A[2];
    float d = A[4], e = A[5], f = A[8];

    float A00 = d*f - e*e;
    float A01 = c*e - b*f;
    float A02 = b*e - c*d;
    float A11 = a*f - c*c;
    float A12 = b*c - a*e;
    float A22 = a*d - b*b;

    float det = a*A00 + b*A01 + c*A02;
    if (std::abs(det) < 1e-12f)
        return false;

    float invDet = 1.0f / det;
    inv = {
        A00 * invDet, A01 * invDet, A02 * invDet,
        A01 * invDet, A11 * invDet, A12 * invDet,
        A02 * invDet, A12 * invDet, A22 * invDet
    };
    return true;
}

static std::array<float, 3> multiplyCovVec(const Matrix3x3& A,
                                           const std::array<float, 3>& v)
{
    return {
        A[0]*v[0] + A[1]*v[1] + A[2]*v[2],
        A[3]*v[0] + A[4]*v[1] + A[5]*v[2],
        A[6]*v[0] + A[7]*v[1] + A[8]*v[2]
    };
}

static std::vector<Matrix3x3> estimateCovariances(
    const std::vector<std::array<float,3>>& pts, int k = 10)
{
    std::vector<Matrix3x3> covariances;
    covariances.reserve(pts.size());

    if (pts.size() < 3)
    {
        covariances.assign(pts.size(), {1e-3f,0,0, 0,1e-3f,0, 0,0,1e-3f});
        return covariances;
    }

    KDTree tree;
    tree.build(pts);

    int neighborCount = std::max(3, std::min(k, (int)pts.size()));
    for (const auto& p : pts)
    {
        std::vector<int> idx = tree.kNearestIdx(p, neighborCount);
        int cnt = (int)idx.size();

        std::array<float, 3> centroid = {0, 0, 0};
        for (int i : idx)
        {
            centroid[0] += pts[i][0];
            centroid[1] += pts[i][1];
            centroid[2] += pts[i][2];
        }
        centroid[0] /= cnt;
        centroid[1] /= cnt;
        centroid[2] /= cnt;

        Matrix3x3 C = {};
        for (int i : idx)
        {
            std::array<float, 3> d = {
                pts[i][0] - centroid[0],
                pts[i][1] - centroid[1],
                pts[i][2] - centroid[2]
            };
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    C[r*3+c] += d[r] * d[c] / (float)cnt;
        }

        // VGICP 공분산 정규화: 고유값 상대 플로어 (dst와 동일 방식)
        Matrix3x3 eig = C;
        Matrix3x3 V = jacobiEigenvectors(eig);
        float lamMax = std::max(eig[0], std::max(eig[4], eig[8]));
        float fl = std::max(lamMax, 1e-12f) * 1e-3f;
        std::array<float, 3> lam = {
            std::max(eig[0], fl), std::max(eig[4], fl), std::max(eig[8], fl)
        };
        Matrix3x3 Creg = {};
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k)
            {
                float s = 0.0f;
                for (int c = 0; c < 3; ++c) s += V[r*3+c] * lam[c] * V[k*3+c];
                Creg[r*3+k] = s;
            }
        covariances.push_back(Creg);
    }

    return covariances;
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
        std::vector<int> idx = tree.kNearestIdx(p, neighborCount);
        int cnt = (int)idx.size();

        std::array<float, 3> centroid = {0, 0, 0};
        for (int i : idx)
        {
            centroid[0] += pts[i][0];
            centroid[1] += pts[i][1];
            centroid[2] += pts[i][2];
        }
        centroid[0] /= cnt;
        centroid[1] /= cnt;
        centroid[2] /= cnt;

        Matrix3x3 C = {};
        for (int i : idx)
        {
            std::array<float, 3> d = {
                pts[i][0] - centroid[0],
                pts[i][1] - centroid[1],
                pts[i][2] - centroid[2]
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

static bool computeGICPStep(const std::vector<std::array<float, 3>>& matchedSrc,
                            const std::vector<std::array<float, 3>>& matchedCenters,
                            const std::vector<Matrix3x3>& matchedSrcCovariances,
                            const std::vector<Matrix3x3>& matchedDstCovariances,
                            Matrix3x3& R,
                            std::array<float, 3>& t,
                            const std::array<float, 3>* gravityWorldUp = nullptr,
                            float gravityScale = 0.0f)
{
    float A[6][6] = {};
    float b[6] = {};
    int validConstraints = 0;

    for (int i = 0; i < (int)matchedSrc.size(); ++i)
    {
        const auto& p = matchedSrc[i];
        const auto& q = matchedCenters[i];

        // 결합 공분산 (양쪽 모두 고유값 플로어 적용됨 → 추가 정규화 불필요)
        Matrix3x3 C = {};
        for (int j = 0; j < 9; ++j)
            C[j] = matchedSrcCovariances[i][j] + matchedDstCovariances[i][j];

        Matrix3x3 W = {};
        if (!invertSymmetric3x3(C, W))
            continue;
        ++validConstraints;

        std::array<float, 3> residual = {
            p[0] - q[0],
            p[1] - q[1],
            p[2] - q[2]
        };
        std::array<float, 3> Wr = multiplyCovVec(W, residual);

        float J[3][6] = {
            { 0.0f,  p[2], -p[1], 1.0f, 0.0f, 0.0f },
            {-p[2], 0.0f,  p[0], 0.0f, 1.0f, 0.0f },
            { p[1],-p[0], 0.0f, 0.0f, 0.0f, 1.0f }
        };

        for (int r = 0; r < 6; ++r)
        {
            float JrWr = 0.0f;
            for (int m = 0; m < 3; ++m)
                JrWr += J[m][r] * Wr[m];
            b[r] -= JrWr;

            for (int c = 0; c < 6; ++c)
            {
                float JWJ = 0.0f;
                for (int m = 0; m < 3; ++m)
                {
                    float WJc = 0.0f;
                    for (int n = 0; n < 3; ++n)
                        WJc += W[m*3+n] * J[n][c];
                    JWJ += J[m][r] * WJc;
                }
                A[r][c] += JWJ;
            }
        }
    }

    if (validConstraints < 10)
        return false;

    // 중력 prior (L2): roll/pitch soft constraint.
    // v = result.R * g_body (현재 추정한 "위" 방향, 월드 좌표) 가 (0,0,1)에 맞도록.
    // 오차 e0 = v × (0,0,1), 회전증분 ω에 대한 야코비안 J_g = v·tᵀ - (v·t)I.
    // yaw 성분(t 방향)은 e0에 거의 기여하지 않아 자동으로 자유.
    // 가중치는 회전 블록 평균 대각에 비례 — LiDAR 회전 구속이 약한 축(예: pitch)에서
    // 상대적으로 중력이 지배하여 드리프트를 잡는다.
    if (gravityWorldUp && gravityScale > 0.0f)
    {
        const auto& v = *gravityWorldUp;
        const std::array<float, 3> tgt = {0.0f, 0.0f, 1.0f};
        std::array<float, 3> e0 = crossVec(v, tgt);
        float vDotT = dotVec(v, tgt);

        float Jg[3][3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Jg[r][c] = v[r] * tgt[c] - (r == c ? vDotT : 0.0f);

        float rotDiagMean = (A[0][0] + A[1][1] + A[2][2]) / 3.0f;
        float wg = gravityScale * rotDiagMean;

        for (int r = 0; r < 3; ++r)
        {
            float JTe = 0.0f;
            for (int m = 0; m < 3; ++m) JTe += Jg[m][r] * e0[m];
            b[r] -= wg * JTe;
            for (int c = 0; c < 3; ++c)
            {
                float JTJ = 0.0f;
                for (int m = 0; m < 3; ++m) JTJ += Jg[m][r] * Jg[m][c];
                A[r][c] += wg * JTJ;
            }
        }
    }

    // 적응형 Tikhonov 정규화 (degenerate 방향 처리).
    // 고속도로 직선 구간처럼 진행방향 평행이동이 관측되지 않으면 해당 축의
    // Hessian 대각이 거의 0이 된다. 회전/이동 블록 각각의 평균 대각에 비례한
    // damping을 더하면, 잘 구속된 축은 거의 영향이 없고(대각 >> damping)
    // 관측이 약한 축은 예측값(=초기 추정)으로 고정되어 노이즈 발산을 막는다.
    float rotDiagMean   = (A[0][0] + A[1][1] + A[2][2]) / 3.0f;
    float transDiagMean = (A[3][3] + A[4][4] + A[5][5]) / 3.0f;
    float rotLambda   = 1e-2f * rotDiagMean   + 1e-9f;
    float transLambda = 1e-2f * transDiagMean + 1e-9f;
    for (int i = 0; i < 3; ++i) A[i][i] += rotLambda;
    for (int i = 3; i < 6; ++i) A[i][i] += transLambda;

    float x[6] = {};
    if (!solveGaussian6x6(A, b, x))
        return false;

    std::array<float, 3> w = {x[0], x[1], x[2]};
    R = rodrigues(w);
    t = {x[3], x[4], x[5]};
    return true;
}

static Matrix3x3 computeSVD(const Matrix3x3& H)
{
    Matrix3x3 Ht = { H[0], H[3], H[6],
                     H[1], H[4], H[7],
                     H[2], H[5], H[8] };
    Matrix3x3 M = multiplyMat(Ht, H);

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

    Matrix3x3 Ut = { U[0], U[3], U[6],
                     U[1], U[4], U[7],
                     U[2], U[5], U[8] };
    Matrix3x3 R = multiplyMat(V, Ut);

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

ICPResult runICP(const std::vector<std::array<float, 3>>& src,
                 const std::vector<std::array<float, 3>>& dst,
                 int maxIterations, float tolerance,
                 const Matrix3x3* initialR,
                 const std::array<float, 3>* initialT,
                 bool usePointToPlane,
                 bool useGICP,
                 const std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash>* voxelMap,
                 float voxelSize,
                 const std::array<float, 3>* gravityBodyUp,
                 float gravityScale)
{
    (void)tolerance;

    ICPResult result;
    result.R = {1,0,0, 0,1,0, 0,0,1};
    result.t = {0, 0, 0};
    result.error = std::numeric_limits<float>::max();
    result.fitness = 0.0f;
    result.iterations = 0;

    KDTree tree;
    if (!useGICP)
        tree.build(dst);
    std::vector<std::array<float, 3>> dstNormals;
    if (!useGICP)
        dstNormals = estimateNormals(dst);
    std::vector<Matrix3x3> srcCovariances;
    if (useGICP)
        srcCovariances = estimateCovariances(src);

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

    const float gicpMaxDist = std::max(0.5f, voxelSize * 1.25f);
    const float maxDistSq = useGICP ? gicpMaxDist * gicpMaxDist : 0.35f * 0.35f;
    // 공분산이 신뢰 가능할 최소 점수 (is_planar 게이트 대체)
    const int kMinCovPoints = 6;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        std::vector<std::array<float, 3>> matchedSrc;
        std::vector<std::array<float, 3>> matchedDst;
        std::vector<std::array<float, 3>> matchedNormals;
        std::vector<Matrix3x3> matchedSrcCovariances;
        std::vector<Matrix3x3> matchedDstCovariances;
        float totalPointToPlaneSquaredError = 0.0f;

        for (size_t pointIdx = 0; pointIdx < current.size(); ++pointIdx)
        {
            const auto& p = current[pointIdx];
            std::array<float, 3> nearest;
            int nearestIdx = -1;
            if (useGICP && voxelMap)
            {
                const VoxelCell* nearestCell = nullptr;
                float nearestDistSq = std::numeric_limits<float>::max();

                int ix = (int)std::floor(p[0] / voxelSize);
                int iy = (int)std::floor(p[1] / voxelSize);
                int iz = (int)std::floor(p[2] / voxelSize);

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            VoxelKey key{ix + dx, iy + dy, iz + dz};
                            auto cellIt = voxelMap->find(key);
                            if (cellIt == voxelMap->end()) continue;

                            const VoxelCell& cell = cellIt->second;
                            // VGICP: 평면 게이트 대신 공분산이 신뢰 가능한(점 충분한) voxel을
                            // 모두 사용. 기하 가중은 공분산 W가 알아서 처리한다.
                            if (cell.point_count < kMinCovPoints) continue;

                            float cx = p[0] - cell.center[0];
                            float cy = p[1] - cell.center[1];
                            float cz = p[2] - cell.center[2];
                            float distSq = cx*cx + cy*cy + cz*cz;
                            if (distSq < nearestDistSq)
                            {
                                nearestDistSq = distSq;
                                nearestCell = &cell;
                            }
                        }
                    }
                }

                if (!nearestCell || nearestDistSq > maxDistSq) continue;

                matchedSrc.push_back(p);
                matchedDst.push_back(nearestCell->center);
                matchedNormals.push_back(nearestCell->normal);
                matchedSrcCovariances.push_back(
                    transformCovariance(result.R, srcCovariances[pointIdx]));
                matchedDstCovariances.push_back(nearestCell->covariance);
                float residual = dotVec({p[0] - nearestCell->center[0],
                                         p[1] - nearestCell->center[1],
                                         p[2] - nearestCell->center[2]},
                                        nearestCell->normal);
                totalPointToPlaneSquaredError += residual * residual;
                continue;
            }
            else
            {
                nearestIdx = tree.nearestIdx(p);
                if (nearestIdx < 0) continue;
                nearest = dst[nearestIdx];
            }

            float dx = p[0] - nearest[0];
            float dy = p[1] - nearest[1];
            float dz = p[2] - nearest[2];
            float distSq = dx*dx + dy*dy + dz*dz;

            if (distSq > maxDistSq) continue;  // 아웃라이어 제거

            matchedSrc.push_back(p);
            matchedDst.push_back(nearest);
            if (usePointToPlane)
            {
                matchedNormals.push_back(dstNormals[nearestIdx]);
            }

            float residual = dotVec({dx, dy, dz}, dstNormals[nearestIdx]);
            totalPointToPlaneSquaredError += residual * residual;
        }

        if (matchedSrc.size() < 10) break;

        const float previousError = result.error;
        const float inlierRatio = current.empty() ? 0.0f
            : (float)matchedSrc.size() / (float)current.size();
        const float rmse = std::sqrt(totalPointToPlaneSquaredError / (float)matchedSrc.size());

        result.fitness = inlierRatio;
        result.error = rmse;

        Matrix3x3 R = {1,0,0, 0,1,0, 0,0,1};
        std::array<float, 3> t = {0, 0, 0};

        if (useGICP && voxelMap)
        {
            // 현재 추정 자세로 바디 "위"를 월드로 회전 → 중력 prior 입력
            std::array<float, 3> gravityWorldUp;
            const std::array<float, 3>* gravityArg = nullptr;
            if (gravityBodyUp && gravityScale > 0.0f)
            {
                gravityWorldUp = multiplyVec(result.R, *gravityBodyUp);
                gravityArg = &gravityWorldUp;
            }
            if (!computeGICPStep(matchedSrc, matchedDst,
                                 matchedSrcCovariances, matchedDstCovariances,
                                 R, t, gravityArg, gravityScale))
                break;
        }
        else if (usePointToPlane)
        {
            if (!computePointToPlaneStep(matchedSrc, matchedDst, matchedNormals, R, t))
                break;
        }
        else
        {
            auto srcCentroid = computeCentroid(matchedSrc);
            auto dstCentroid = computeCentroid(matchedDst);

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

                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        H[r*3+c] += ps[r] * pd[c];
            }

            R = computeSVD(H);

            auto rotatedSrc = multiplyVec(R, srcCentroid);
            t = {
                dstCentroid[0] - rotatedSrc[0],
                dstCentroid[1] - rotatedSrc[1],
                dstCentroid[2] - rotatedSrc[2]
            };
        }

        for (auto& p : current)
        {
            p = multiplyVec(R, p);
            p[0] += t[0];
            p[1] += t[1];
            p[2] += t[2];
        }

        // 반드시 이전 result.t를 먼저 복사한 뒤 계산해야 해요.
        // 바로 result.t에 쓰면 아직 쓰지 않은 값이 덮어씌워집니다.
        result.R = multiplyMat(R, result.R);
        auto newT = multiplyVec(R, result.t);
        result.t[0] = newT[0] + t[0];
        result.t[1] = newT[1] + t[1];
        result.t[2] = newT[2] + t[2];

        result.iterations = iter + 1;

        const float deltaTranslationNorm = std::sqrt(dotVec(t, t));
        float deltaRotationFrobenius = 0.0f;
        for (int i = 0; i < 9; ++i)
        {
            const float identityValue = (i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f;
            const float diff = identityValue - R[i];
            deltaRotationFrobenius += diff * diff;
        }
        deltaRotationFrobenius = std::sqrt(deltaRotationFrobenius);
        const float errorImprovement =
            std::isfinite(previousError) ? std::abs(previousError - rmse)
                                         : std::numeric_limits<float>::max();

        if (deltaTranslationNorm < 1e-4f &&
            deltaRotationFrobenius < 1e-4f &&
            errorImprovement < 1e-4f)
        {
            break;
        }
    }

    return result;
}
