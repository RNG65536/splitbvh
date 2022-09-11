#pragma once

// median split BVH

#include <iostream>
#include <memory>
#include <string>

#include "AABB.h"
#include "Geometry.h"
#include "Logger.h"
#include "Math.h"
#include "Mesh.h"
#include "MeshGroup.h"
#include "Triangle.h"

// when to use enable_shared_from_this ??

struct BVHNode : public Geometry
{
    // TODO : move this to Geometry ??
    std::shared_ptr<AABB> boundingBox;

    uint32_t    debug_level;
    std::string debug_tag;
};

struct BVHLeafNode : public BVHNode
{
    std::shared_ptr<Geometry> primitive;

    bool intersect(const Ray& ray, float& tNear) const override;
};

struct BVHBranchNode : public BVHNode
{
    std::shared_ptr<BVHNode> leftChild;
    std::shared_ptr<BVHNode> rightChild;
    int                      split_axis;

    bool intersect(const Ray& ray, float& tNear) const override;
};

struct SplitAxisComparator
{
    SplitAxisComparator(int split_axis);

    bool operator()(const std::shared_ptr<Triangle>& a,
                    const std::shared_ptr<Triangle>& b) const;

    int m_split_axis;
};

class BVH : public Geometry
{
public:
    BVH(const Mesh& mesh);

    std::shared_ptr<BVHNode> build(int                begin,
                                   int                end,
                                   uint32_t           level,
                                   const std::string& direction);

    bool intersect(const Ray& ray, float& tNear) const override;

private:
    std::vector<std::shared_ptr<Triangle>> m_triangles;
    std::shared_ptr<BVHNode>               m_rootNode;
};