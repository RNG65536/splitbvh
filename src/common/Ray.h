#pragma once

#include <iostream>
#include <string>

#include "LinearMath.h"
#include "Logger.h"

class Ray
{
public:
    static void set_ray_epsilon(float eps) { m_rayEpsilon = eps; }

    Ray() : m_tMin(m_rayEpsilon), m_tMax(kInfinity) {}

    Ray(const float3& origin, const float3& direction)
        : m_origin(origin)
        , m_direction(direction)
        , m_tMin(m_rayEpsilon)
        , m_tMax(kInfinity)
    {
        // direction must be normalized
        assert(isNormalized(direction));

        float inv_x = m_direction.x == 0.0f ? 0.0f : 1.0f / m_direction.x;
        float inv_y = m_direction.y == 0.0f ? 0.0f : 1.0f / m_direction.y;
        float inv_z = m_direction.z == 0.0f ? 0.0f : 1.0f / m_direction.z;
        m_directionReciprocal = float3(inv_x, inv_y, inv_z);

        m_sign.x = m_direction.x < 0;
        m_sign.y = m_direction.y < 0;
        m_sign.z = m_direction.z < 0;
    }

    float3 at(float t) const { return m_origin + m_direction * t; }

    const float3& origin() const { return m_origin; }

    const float3& direction() const { return m_direction; }

    const float3& directionReciprocal() const { return m_directionReciprocal; }

    const int3& sign() const { return m_sign; }

    float tmin() const { return m_tMin; }

    float tmax() const { return m_tMax; }

    void setOrigin(const float3& origin) { m_origin = origin; }

    void setDirection(const float3& direction)
    {
        m_direction = direction;

        float inv_x = m_direction.x == 0.0f ? 0.0f : 1.0f / m_direction.x;
        float inv_y = m_direction.y == 0.0f ? 0.0f : 1.0f / m_direction.y;
        float inv_z = m_direction.z == 0.0f ? 0.0f : 1.0f / m_direction.z;
        m_directionReciprocal = float3(inv_x, inv_y, inv_z);

        m_sign.x = m_direction.x < 0;
        m_sign.y = m_direction.y < 0;
        m_sign.z = m_direction.z < 0;
    }

    void setTMax(float t) const
    {
// UPDATE : this seems to be fixed
// FIXME : this is certainly buggy, but seems to be caused by somewhere else.
// aabb hit test is tricky, must test range overlap
// between ray min/max and hit near/far
#if 1
        assert(t > m_tMin && t <= m_tMax);
        // Logger::debug() << "    set ray tmax from " << m_tMax << " to " << t
        //                << std::endl;
        m_tMax = t;
#else
        // Logger::debug() << "    unmodified ray tmax = " << m_tMax <<
        // std::endl;
#endif
    }

    void setTMin(float t) const { m_tMin = std::max(t, m_rayEpsilon); }

    std::string toString() const;

private:
    float3        m_origin;
    float3        m_direction;
    float3        m_directionReciprocal;
    mutable float m_tMax;
    mutable float m_tMin;
    int3          m_sign;

    static float m_rayEpsilon;
};