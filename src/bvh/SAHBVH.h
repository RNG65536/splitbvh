#pragma once

// SAH BVH using binning

#include <iostream>
#include <memory>
#include <string>

#include "AABB.h"
#include "Geometry.h"
#include "Logger.h"
#include "Math.h"
#include "Mesh.h"

#define DEBUG_BINS 0
#define DEBUG_INFO 0

constexpr int   k_leaf_node_size      = 4;
constexpr int   k_num_binning_buckets = 16;  // 256;  // 128;
constexpr float k_edge_threshold      = 1e-3f;

// when to use enable_shared_from_this ??

struct SAHBVHNode : public Geometry
{
    // TODO : move this to Geometry ??
    std::shared_ptr<AABB> bounding_box;

    uint32_t    debug_level;
    std::string debug_tag;
};

struct SAHBVHLeafNode : public SAHBVHNode
{
    std::shared_ptr<Geometry> primitive[k_leaf_node_size];

    bool intersect(const Ray& ray, float& tNear) const override;
};

struct SAHBVHBranchNode : public SAHBVHNode
{
    std::shared_ptr<SAHBVHNode> left_child;
    std::shared_ptr<SAHBVHNode> right_child;
    int                         split_axis;

    bool intersect(const Ray& ray, float& tNear) const override;
};

struct SAHSplitAxisComparator
{
    SAHSplitAxisComparator(int split_axis);

    bool operator()(const std::shared_ptr<Triangle>& a,
                    const std::shared_ptr<Triangle>& b) const;

    int m_split_axis;
};

struct MedianSplitter
{
    MedianSplitter(std::vector<std::shared_ptr<Triangle>>& triangles,
                   const AABB&                             node_bound,
                   int                                     begin,
                   int                                     end);

    float cost() const;

    void sort(int& axis, int& mid);

    float m_cost       = kInfinity;
    int   m_split_axis = -1;
    int   m_mid        = -1;

    std::vector<std::shared_ptr<Triangle>>& m_triangles;
    const AABB&                             m_node_bound;
    int                                     m_begin;
    int                                     m_end;
};

struct SAHBinningBucket
{
    AABB m_bounding_box;
    int  m_num_entries = 0;
    int  m_is_valid    = 0;
};

struct SAHSplitter
{
    SAHSplitter(std::vector<std::shared_ptr<Triangle>>& triangles,
                const AABB&                             node_bound,
                int                                     begin,
                int                                     end);

    void sort(int& axis, int& mid);

    float m_min_cost;
    int   m_min_axis;  // min cost split axis
    int   m_min_bin;   // min cost split bin, index of the first bucket in the
                       // second child

    std::vector<std::shared_ptr<Triangle>>& m_triangles;
    const AABB&                             m_node_bound;
    int                                     m_begin;
    int                                     m_end;
    float                                   m_edge[3];
};

class SAHBVH : public Geometry
{
public:
    SAHBVH(const Mesh& mesh);

    std::shared_ptr<SAHBVHNode> buildLeaf(int                begin,
                                          int                end,
                                          uint32_t           level,
                                          const std::string& direction);

    std::shared_ptr<SAHBVHBranchNode> buildBranch(int                begin,
                                                  int                end,
                                                  uint32_t           level,
                                                  const std::string& direction,
                                                  int&               mid);

    std::shared_ptr<SAHBVHNode> build(int                begin,
                                      int                end,
                                      uint32_t           level,
                                      const std::string& direction);

    bool intersect(const Ray& ray, float& tNear) const override;

private:
    std::vector<std::shared_ptr<Triangle>> m_triangles;
    std::shared_ptr<SAHBVHNode>            m_rootNode;
};