// KD-Tree 구현: 트리 구축, 최근접 탐색, 메모리 해제
#include "KDTree.h"
#include <algorithm>
#include <limits>
#include <cmath>

// ── 소멸자 ──────────────────────────────────────────
KDTree::~KDTree()
{
    clear(root);
}

void KDTree::clear(KDNode* node)
{
    if (node == nullptr) return;
    clear(node->left);
    clear(node->right);
    delete node;
}

// ── 트리 구축 ────────────────────────────────────────
void KDTree::build(const std::vector<std::array<float, 3>>& points)
{
    std::vector<std::array<float, 3>> pts(points);
    root = buildRecursive(pts, 0);
}

KDNode* KDTree::buildRecursive(std::vector<std::array<float, 3>> points, int depth)
{
    if (points.empty()) return nullptr;

    // 현재 depth에서 기준이 되는 축 (0=X, 1=Y, 2=Z, 반복)
    int axis = depth % 3;

    // 해당 축 기준으로 정렬 후 중간값을 이 노드로 선택
    std::sort(points.begin(), points.end(),
        [axis](const std::array<float, 3>& a, const std::array<float, 3>& b) {
            return a[axis] < b[axis];
        });

    int mid = points.size() / 2;

    KDNode* node  = new KDNode();
    node->point   = points[mid];
    node->left    = buildRecursive(
                        std::vector<std::array<float, 3>>(points.begin(), points.begin() + mid),
                        depth + 1);
    node->right   = buildRecursive(
                        std::vector<std::array<float, 3>>(points.begin() + mid + 1, points.end()),
                        depth + 1);

    return node;
}

// ── 최근접 탐색 ──────────────────────────────────────
std::array<float, 3> KDTree::nearest(const std::array<float, 3>& query) const
{
    KDNode* best    = nullptr;
    float bestDist  = std::numeric_limits<float>::max();
    nearestRecursive(root, query, 0, best, bestDist);
    return best->point;
}

void KDTree::nearestRecursive(KDNode* node, const std::array<float, 3>& query,
                               int depth, KDNode*& best, float& bestDist) const
{
    if (node == nullptr) return;

    // 현재 노드와 query 사이의 거리 계산
    float dist = 0.0f;
    for (int i = 0; i < 3; ++i)
        dist += (node->point[i] - query[i]) * (node->point[i] - query[i]);

    // 지금까지 찾은 것보다 가까우면 갱신
    if (dist < bestDist)
    {
        bestDist = dist;
        best     = node;
    }

    int axis = depth % 3;

    // query가 현재 노드 기준 어느 쪽인지 판단
    KDNode* first  = (query[axis] < node->point[axis]) ? node->left  : node->right;
    KDNode* second = (query[axis] < node->point[axis]) ? node->right : node->left;

    // 가까운 쪽 먼저 탐색
    nearestRecursive(first, query, depth + 1, best, bestDist);

    // 반대쪽도 더 가까운 점이 있을 수 있으면 탐색
    float axisDist = (query[axis] - node->point[axis]) * (query[axis] - node->point[axis]);
    if (axisDist < bestDist)
        nearestRecursive(second, query, depth + 1, best, bestDist);
}