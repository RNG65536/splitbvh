#pragma once

#include "AABB.h"
#include "Geometry.h"
#include "Math.h"

class Triangle : public Geometry
{
public:
    static void setIntersectionEpsilon(float eps)
    {
        m_intersectionEpsilon = eps;
    }

    bool intersect(const Ray& ray, float& tNear) const override;

    Triangle(const float3& a, const float3& b, const float3& c)
        : m_a(a), m_b(b), m_c(c)
    {
        m_aabb.expandBy(m_a);
        m_aabb.expandBy(m_b);
        m_aabb.expandBy(m_c);

#if 0
        m_center = (m_a + m_b + m_c) / 3.0f;
#else
        m_center = m_aabb.center();
#endif
    }

    const float3& va() const
    {
        return m_a;
    }

    const float3& vb() const
    {
        return m_b;
    }

    const float3& vc() const
    {
        return m_c;
    }

    const float3& center() const
    {
        return m_center;
    }

    const AABB& boundingBox() const
    {
        return m_aabb;
    }

    std::string toString() const override;

private:
    // vertices
    float3 m_a;
    float3 m_b;
    float3 m_c;
    float3 m_center;
    AABB   m_aabb;

    static float m_intersectionEpsilon;
};
