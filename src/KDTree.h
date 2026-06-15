// KD-Tree 선언: 3D 포인트 클라우드의 최근접 탐색을 위한 자료구조
#pragma once
#include <vector>
#include <array>

// 트리를 구성하는 노드 하나
struct KDNode
{
    std::array<float, 3> point;  // x, y, z
    int index = -1;              // 원본 배열에서의 인덱스
    KDNode* left  = nullptr;
    KDNode* right = nullptr;
};

class KDTree
{
public:
    KDTree() : root(nullptr) {}
    ~KDTree();

    // 포인트 목록을 받아서 트리를 구축
    void build(const std::vector<std::array<float, 3>>& points);

    // query와 가장 가까운 점을 반환
    std::array<float, 3> nearest(const std::array<float, 3>& query) const;

    // query와 가장 가까운 점의 원본 배열 인덱스를 반환
    int nearestIdx(const std::array<float, 3>& query) const;

private:
    struct IndexedPoint
    {
        std::array<float, 3> point;
        int index;
    };

    KDNode* root;

    KDNode* buildRecursive(std::vector<IndexedPoint> points, int depth);
    void nearestRecursive(KDNode* node, const std::array<float, 3>& query,
                          int depth, KDNode*& best, float& bestDist) const;
    void clear(KDNode* node);
};
