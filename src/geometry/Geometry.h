#pragma once

#include <string>
#include "Ray.h"

// abstraction for triangle/mesh/etc.
// intersectable by ray
class Geometry
{
public:
    // don't care about normal/etc. at the moment.
    // returns true if tNear is between ray [tmin, tmax],
    // otherwise returns false and tNear is set to infinity.
    virtual bool intersect(const Ray& ray, float& tNear) const = 0;

    virtual bool intersectBound(const Ray& ray) const
    {
        return true;
    }

    virtual std::string toString() const
    {
        return "";
    }
};

// NOTE: implementations
// Triangle
// Mesh
// AABB
// BVHLeafNode
// BVHBranchNode
// BVH
