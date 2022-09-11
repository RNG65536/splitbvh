#pragma once

#include <string>
#include <vector>

#include "Geometry.h"
#include "LinearMath.h"

class Triangle;
class BoundedTriangle;

class AABB : public Geometry
{
public:
    AABB() : m_min(kInfinity), m_max(-kInfinity) {}

    AABB(const float3& min, const float3& max) : m_min(min), m_max(max) {}

    void reset()
    {
        m_min = float3(kInfinity);
        m_max = float3(-kInfinity);
    }

    bool intersect(const Ray& ray, float& tNear) const override;

    // MUST use this for AABB culling because the ray's origin can be inside the
    // AABB
    bool overlap(const Ray& ray) const;

    bool isValid() const
    {
        return m_max.x >= m_min.x && m_max.y >= m_min.y && m_max.z >= m_min.z;
    }

    int largestDimension() const
    {
        assert(isValid());
        float e0 = m_max.x - m_min.x;
        float e1 = m_max.y - m_min.y;
        float e2 = m_max.z - m_min.z;

        int   max_dim = 0;
        float max_e   = e0;

        if (e1 > max_e)
        {
            max_e   = e1;
            max_dim = 1;
        }

        if (e2 > max_e)
        {
            max_e   = e2;
            max_dim = 2;
        }

        return max_dim;
    }

    void expandBy(const AABB& box)
    {
        // this AABB could be invalid, but the floating point
        // arithmetic would handle it
        assert(box.isValid());

        m_min.x = std::min(m_min.x, box.m_min.x);
        m_min.y = std::min(m_min.y, box.m_min.y);
        m_min.z = std::min(m_min.z, box.m_min.z);

        m_max.x = std::max(m_max.x, box.m_max.x);
        m_max.y = std::max(m_max.y, box.m_max.y);
        m_max.z = std::max(m_max.z, box.m_max.z);
    }

    void expandBy(const float3& point)
    {
        // this AABB could be invalid, but the floating point
        // arithmetic would handle it

        m_min.x = std::min(m_min.x, point.x);
        m_min.y = std::min(m_min.y, point.y);
        m_min.z = std::min(m_min.z, point.z);

        m_max.x = std::max(m_max.x, point.x);
        m_max.y = std::max(m_max.y, point.y);
        m_max.z = std::max(m_max.z, point.z);
    }

    void intersectBy(const AABB& box)
    {
        assert(box.isValid());

        m_min.x = std::max(m_min.x, box.m_min.x);
        m_max.x = std::min(m_max.x, box.m_max.x);
        if (m_min.x > m_max.x)
        {
            this->reset();
            return;
        }

        m_min.y = std::max(m_min.y, box.m_min.y);
        m_max.y = std::min(m_max.y, box.m_max.y);
        if (m_min.y > m_max.y)
        {
            this->reset();
            return;
        }

        m_min.z = std::max(m_min.z, box.m_min.z);
        m_max.z = std::min(m_max.z, box.m_max.z);
        if (m_min.z > m_max.z)
        {
            this->reset();
            return;
        }
    }

    AABB intersect(const AABB& box) const
    {
        assert(box.isValid());

        AABB ret;

        ret.m_min.x = std::max(m_min.x, box.m_min.x);
        ret.m_max.x = std::min(m_max.x, box.m_max.x);
        if (ret.m_min.x > ret.m_max.x)
        {
            ret.reset();
            return ret;
        }

        ret.m_min.y = std::max(m_min.y, box.m_min.y);
        ret.m_max.y = std::min(m_max.y, box.m_max.y);
        if (ret.m_min.y > ret.m_max.y)
        {
            ret.reset();
            return ret;
        }

        ret.m_min.z = std::max(m_min.z, box.m_min.z);
        ret.m_max.z = std::min(m_max.z, box.m_max.z);
        if (ret.m_min.z > ret.m_max.z)
        {
            ret.reset();
            return ret;
        }

        return ret;
    }

    const float3& min() const { return m_min; }

    const float3& max() const { return m_max; }

    float3 center() const { return (m_min + m_max) * 0.5f; }

    float surfaceArea() const
    {
        float3 edge = m_max - m_min;
        return fabs(edge.x * edge.y + edge.y * edge.z + edge.x * edge.z) * 2.0f;
    }

    // bound the triangle portion between the clip planes
    // void clipXAndExpandTight(const BoundedTriangle& triangle, float min,
    // float max); void clipYAndExpandTight(const BoundedTriangle& triangle,
    // float min, float max); void clipZAndExpandTight(const BoundedTriangle&
    // triangle, float min, float max);
    void clipXAndExpand(const BoundedTriangle& triangle, float min, float max);
    void clipYAndExpand(const BoundedTriangle& triangle, float min, float max);
    void clipZAndExpand(const BoundedTriangle& triangle, float min, float max);

    AABB clipAxis(float min, float max, int axis) const;
    AABB clipX(float min, float max) const;
    AABB clipY(float min, float max) const;
    AABB clipZ(float min, float max) const;

    std::string toString() const;

    std::vector<float3> getCorners() const
    {
        return std::vector<float3>{
            float3(m_min.x, m_min.y, m_min.z),
            float3(m_max.x, m_min.y, m_min.z),
            float3(m_min.x, m_max.y, m_min.z),
            float3(m_max.x, m_max.y, m_min.z),
            float3(m_min.x, m_min.y, m_max.z),
            float3(m_max.x, m_min.y, m_max.z),
            float3(m_min.x, m_max.y, m_max.z),
            float3(m_max.x, m_max.y, m_max.z),
        };
    }

private:
    float3 m_min;
    float3 m_max;
};