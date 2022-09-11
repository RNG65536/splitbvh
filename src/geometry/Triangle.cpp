#include <iostream>
#include <sstream>
#include "Triangle.h"
#include "Logger.h"

using std::endl;

float Triangle::m_intersectionEpsilon = 1e-6f;

#define MOLLER_TRUMBORE
// returns positive or negative distance
static bool rayTriangleIntersect(const float3& orig,
                                 const float3& dir,
                                 const float3& v0,
                                 const float3& v1,
                                 const float3& v2,
                                 float&        t,
                                 float&        u,
                                 float&        v,
                                 float         kEpsilon)
{
#ifdef MOLLER_TRUMBORE
    float3 v0v1 = v1 - v0;
    float3 v0v2 = v2 - v0;
    float3 pvec = cross(dir, v0v2);
    float  det = dot(v0v1, pvec);
#ifdef CULLING
    // if the determinant is negative the triangle is backfacing
    // if the determinant is close to 0, the ray misses the triangle
    if (det < kEpsilon) return false;
#else
    // ray and triangle are parallel if det is close to 0
    if (fabs(det) < kEpsilon) return false;
#endif
    float invDet = 1 / det;

    float3 tvec = orig - v0;
    u = dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1) return false;

    float3 qvec = cross(tvec, v0v1);
    v = dot(dir, qvec) * invDet;
    if (v < 0 || u + v > 1) return false;

    t = dot(v0v2, qvec) * invDet;

    return true;
#else
#endif
}

bool Triangle::intersect(const Ray& ray, float& tNear) const
{
    float t = kInfinity;
    float u = 0, v = 0;
    bool  hit = rayTriangleIntersect(ray.origin(),
                                    ray.direction(),
                                    m_a,
                                    m_b,
                                    m_c,
                                    t,
                                    u,
                                    v,
                                    m_intersectionEpsilon);
    if (hit && t > ray.tmin() && t < ray.tmax())
    {
        tNear = t;
        return true;
    }
    else
    {
        tNear = kInfinity;
        return false;
    }
}

std::string Triangle::toString() const
{
    std::stringstream ss;
    ss << "<Triangle>" << endl;
    ss << "a = " << m_a.x << ", " << m_a.y << ", " << m_a.z << endl;
    ss << "b = " << m_b.x << ", " << m_b.y << ", " << m_b.z << endl;
    ss << "c = " << m_c.x << ", " << m_c.y << ", " << m_c.z << endl;
    ss << "bbox = " << m_aabb.toString() << endl;
    return ss.str();
}
