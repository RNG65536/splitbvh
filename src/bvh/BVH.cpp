#include "BVH.h"

bool BVHLeafNode::intersect(const Ray& ray, float& tNear) const
{
    tNear = kInfinity;

    float t_near = kInfinity;
    bool  hit    = primitive->intersect(ray, t_near);
    if (hit)
    {
        Logger::debug() << "[ " << debug_level << " ][ " << debug_tag << " ]"
                        << std::endl;
        Logger::debug() << "    primitive t_near = " << t_near << std::endl;

        assert(t_near < ray.tmax());
        ray.setTMax(t_near);
        tNear = t_near;
    }
    return hit;  // if no hit, returns infinity by default
}

bool BVHBranchNode::intersect(const Ray& ray, float& tNear) const
{
    Logger::debug() << "[ " << debug_level << " ][ " << debug_tag << " ]"
                    << std::endl;

    tNear = kInfinity;  // this is necessary

    // AABB culling
    float tBox = kInfinity;
    // bool  hitBox = boundingBox->intersect(ray, tBox);  // wrong
    bool hitBox = boundingBox->overlap(ray);
    if (!hitBox)
    {
        return false;
    }

    float t_near = kInfinity;

    bool anyHit = false;
    if (leftChild)
    {
        float t;
        bool  hit = leftChild->intersect(ray, t);
        if (hit)
        {
            t_near = std::min(t_near, t);
            anyHit = true;
        }

        // not really necessary
        // if (t_near < ray.tmax())
        //{
        //    ray.setTMax(t_near);
        //}
    }

    if (rightChild)
    {
        float t;
        bool  hit = rightChild->intersect(ray, t);
        if (hit)
        {
            t_near = std::min(t_near, t);
            anyHit = true;
        }

        // not really necessary
        // if (t_near < ray.tmax())
        //{
        //    ray.setTMax(t_near);
        //}
    }

    if (anyHit)
    {
        tNear = std::min(tNear, t_near);
        return true;
    }
    else
    {
        tNear = kInfinity;
        return false;
    }
}

SplitAxisComparator::SplitAxisComparator(int split_axis)
    : m_split_axis(split_axis)
{
}

bool SplitAxisComparator::operator()(const std::shared_ptr<Triangle>& a,
                                     const std::shared_ptr<Triangle>& b) const
{
    const float3& ca = a->center();
    const float3& cb = b->center();

    if (m_split_axis == 0)
    {
        return ca.x < cb.x;
    }
    else if (m_split_axis == 1)
    {
        return ca.y < cb.y;
    }
    else if (m_split_axis == 2)
    {
        return ca.z < cb.z;
    }
    else
    {
        assert(false);
        return false;
    }
}

bool BVH::intersect(const Ray& ray, float& tNear) const
{
    // BVH traversal
    tNear = kInfinity;

    bool hit = m_rootNode->intersect(ray, tNear);

    // if (hit && tNear > ray.tmin() && tNear < ray.tmax())
    if (hit)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::shared_ptr<BVHNode> BVH::build(int                begin,
                                    int                end,
                                    uint32_t           level,
                                    const std::string& direction)
{
    assert(begin < end);

    // if (end - begin < 2)  // leaf node
    if (end - begin == 1)  // leaf node
    {
        auto node         = std::make_shared<BVHLeafNode>();
        auto bounding_box = std::make_shared<AABB>();

        // TODO : check how to handle out-of-range more robustly
        assert(begin < m_triangles.size());
        node->primitive = m_triangles[begin];

        node->debug_level = level;
        node->debug_tag   = std::string("leaf ") + direction;

        bounding_box->expandBy(m_triangles[begin]->boundingBox());
        node->boundingBox = bounding_box;

        return node;
    }
    else
    {
        auto node         = std::make_shared<BVHBranchNode>();
        auto bounding_box = std::make_shared<AABB>();

        for (int i = begin; i < end; ++i)
        {
            const auto& box = m_triangles[i]->boundingBox();
            bounding_box->expandBy(box);
        }
        int split_axis = bounding_box->largestDimension();

        // TODO : use nth_element
        std::sort(m_triangles.begin() + begin,
                  m_triangles.begin() + end,
                  SplitAxisComparator(split_axis));

        auto mid         = begin + (end - begin) / 2;
        node->leftChild  = build(begin, mid, level + 1, "left");
        node->rightChild = build(mid, end, level + 1, "right");

        node->debug_level = level;
        node->debug_tag   = std::string("branch ") + direction;

#if 0  // debug rebuild bounding box
            bounding_box = std::make_shared<AABB>();
            bounding_box->expandBy(*node->leftChild->boundingBox);
            bounding_box->expandBy(*node->rightChild->boundingBox);
#endif
        node->boundingBox = bounding_box;

        return node;
    }
}

BVH::BVH(const Mesh& mesh)
{
    m_triangles.clear();
    for (const auto& t : mesh)
    {
        // copy constructor
        m_triangles.push_back(std::make_shared<Triangle>(t));
    }

    m_rootNode = build(0, (int)m_triangles.size(), 0u, "root");
}
