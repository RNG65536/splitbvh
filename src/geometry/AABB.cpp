#include <sstream>
#include "AABB.h"
#include "BoundedTriangle.h"
#include "Logger.h"
#include "Triangle.h"

using std::endl;

// test if AABB and ray overlap in the range of [tmin, tmax]
// the actual hit distance is not needed
static bool rayAABBOverlap(const Ray&    r,
                           const float3& bounds_min,
                           const float3& bounds_max)
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    const float3 bounds[2] = {bounds_min, bounds_max};

    tmin = (bounds[r.sign().x].x - r.origin().x) * r.directionReciprocal().x;
    tmax =
        (bounds[1 - r.sign().x].x - r.origin().x) * r.directionReciprocal().x;
    tymin = (bounds[r.sign().y].y - r.origin().y) * r.directionReciprocal().y;
    tymax =
        (bounds[1 - r.sign().y].y - r.origin().y) * r.directionReciprocal().y;

    if ((tmin > tymax) || (tymin > tmax))
    {
        return false;
    }
    if (tymin > tmin)
    {
        tmin = tymin;
    }
    if (tymax < tmax)
    {
        tmax = tymax;
    }

    tzmin = (bounds[r.sign().z].z - r.origin().z) * r.directionReciprocal().z;
    tzmax =
        (bounds[1 - r.sign().z].z - r.origin().z) * r.directionReciprocal().z;

    if ((tmin > tzmax) || (tzmin > tmax))
    {
        return false;
    }
    if (tzmin > tmin)
    {
        tmin = tzmin;
    }
    if (tzmax < tmax)
    {
        tmax = tzmax;
    }

        // modification in order to handle cases where
        // ray origin is inside of the box
#if 0
    float rayTMin = r.tmin();
    float rayTMax = r.tmax();
    if (tmin > rayTMax || tmax < rayTMin)
    {
        return false;
    }
    else
    {
        return true;
    }
#else
    float rayTMax = r.tmax();
    if (tmin > rayTMax)
    {
        return false;
    }
    float rayTMin = r.tmin();
    if (tmax < rayTMin)
    {
        return false;
    }
    return true;
#endif
}

static bool rayAABBIntersect(const Ray&    r,
                             const float3& bounds_min,
                             const float3& bounds_max,
                             float&        t_near)
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;

    const float3 bounds[2] = {bounds_min, bounds_max};

    tmin = (bounds[r.sign().x].x - r.origin().x) * r.directionReciprocal().x;
    tmax =
        (bounds[1 - r.sign().x].x - r.origin().x) * r.directionReciprocal().x;
    tymin = (bounds[r.sign().y].y - r.origin().y) * r.directionReciprocal().y;
    tymax =
        (bounds[1 - r.sign().y].y - r.origin().y) * r.directionReciprocal().y;

    if ((tmin > tymax) || (tymin > tmax))
    {
        t_near = kInfinity;
        return false;
    }
    if (tymin > tmin)
    {
        tmin = tymin;
    }
    if (tymax < tmax)
    {
        tmax = tymax;
    }

    tzmin = (bounds[r.sign().z].z - r.origin().z) * r.directionReciprocal().z;
    tzmax =
        (bounds[1 - r.sign().z].z - r.origin().z) * r.directionReciprocal().z;

    if ((tmin > tzmax) || (tzmin > tmax))
    {
        t_near = kInfinity;
        return false;
    }
    if (tzmin > tmin)
    {
        tmin = tzmin;
    }
    if (tzmax < tmax)
    {
        tmax = tzmax;
    }

#if 1
    // in order to handle cases where
    // ray origin is inside of the box
    float rayTMin = r.tmin();
    float rayTMax = r.tmax();
    if (tmin > rayTMin && tmin <= rayTMax)
    {
        t_near = tmin;
        return true;
    }
    else if (tmax > rayTMin && tmax <= rayTMax)
    {
        t_near = tmax;
        return true;
    }
    else
    {
        t_near = kInfinity;
        return false;
    }
#else
    t_near = tmin;
    return true;
#endif
}

bool AABB::intersect(const Ray& ray, float& tNear) const
{
    assert(this->isValid());

#if 0
    Logger::debug() << "    ray = " << ray.toString() << endl;
    Logger::debug() << "    AABB = " << this->toString() << endl;
#endif

    float t_near = kInfinity;
    bool  hit = rayAABBIntersect(ray, this->min(), this->max(), t_near);

    if (hit && t_near > ray.tmin() && t_near < ray.tmax())
    {
        tNear = t_near;
        return true;
    }
    else
    {
        tNear = kInfinity;
        return false;
    }
}

bool AABB::overlap(const Ray& ray) const
{
    assert(this->isValid());

#if 0
    Logger::debug() << "    ray = " << ray.toString() << endl;
    Logger::debug() << "    AABB = " << this->toString() << endl;
#endif

    bool isOverlapping = rayAABBOverlap(ray, m_min, m_max);
#if 0
    if (isOverlapping)
    {
        Logger::debug() << "    ray and AABB do overlap" << endl;
    }
    else
    {
        Logger::debug() << "    ray and AABB do NOT overlap" << endl;
    }
#endif
    return isOverlapping;
}

// void AABB::clipXAndExpandTight(const BoundedTriangle& triangle,
//                               float                  min,
//                               float                  max)
//{
//}
//
// void AABB::clipYAndExpandTight(const BoundedTriangle& triangle,
//                               float                  min,
//                               float                  max)
//{
//}
//
// void AABB::clipZAndExpandTight(const BoundedTriangle& triangle,
//                               float                  min,
//                               float                  max)
//{
//}

void AABB::clipXAndExpand(const BoundedTriangle& triangle, float min, float max)
{
    assert(min < max);
}

void AABB::clipYAndExpand(const BoundedTriangle& triangle, float min, float max)
{
    assert(min < max);
}

void AABB::clipZAndExpand(const BoundedTriangle& triangle, float min, float max)
{
    assert(min < max);
}

AABB AABB::clipAxis(float min, float max, int axis) const
{
#if 0
    assert(min < max);
    float3 min_v = m_min;
    float3 max_v = m_max;
    switch (axis)
    {
        case 0:
            min_v.x = clampf(min_v.x, min, max);
            max_v.x = clampf(max_v.x, min, max);
            break;
        case 1:
            min_v.y = clampf(min_v.y, min, max);
            max_v.y = clampf(max_v.y, min, max);
            break;
        case 2:
            min_v.z = clampf(min_v.z, min, max);
            max_v.z = clampf(max_v.z, min, max);
            break;
        default:
            Logger::info() << "AABB invalid axis being clipped against"
                           << std::endl;
            exit(-1);
            break;
    }
    return AABB(min_v, max_v);
#else
    assert(min < max);
    assert(axis >= 0 && axis <= 2);
    float3 min_v = m_min;
    float3 max_v = m_max;
    get(min_v, axis) = clampf(get(min_v, axis), min, max);
    get(max_v, axis) = clampf(get(max_v, axis), min, max);
    return AABB(min_v, max_v);
#endif
}

AABB AABB::clipX(float min, float max) const
{
    assert(min < max);
    float3 min_v = m_min;
    float3 max_v = m_max;
    min_v.x = clampf(min_v.x, min, max);
    max_v.x = clampf(max_v.x, min, max);
    return AABB(min_v, max_v);
}

AABB AABB::clipY(float min, float max) const
{
    assert(min < max);
    float3 min_v = m_min;
    float3 max_v = m_max;
    min_v.y = clampf(min_v.y, min, max);
    max_v.y = clampf(max_v.y, min, max);
    return AABB(min_v, max_v);
}

AABB AABB::clipZ(float min, float max) const
{
    assert(min < max);
    float3 min_v = m_min;
    float3 max_v = m_max;
    min_v.z = clampf(min_v.z, min, max);
    max_v.z = clampf(max_v.z, min, max);
    return AABB(min_v, max_v);
}

std::string AABB::toString() const
{
    std::stringstream ss;
    ss << "min(" << m_min.x << ", " << m_min.y << ", " << m_min.z << "), ";
    ss << "max(" << m_max.x << ", " << m_max.y << ", " << m_max.z << ")";
    return ss.str();
}
