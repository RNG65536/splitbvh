#pragma once

#include "Geometry.h"
#include "Ray.h"

struct IntersectionInfo
{

};

// Moller-Trumbore for ray-triangle intersection
// and BVH acceleration for ray-mesh intersection
class Intersection
{
public:
    virtual float intersectWith(const Ray& ray) const = 0;

    // const Geometry* m_geometry = nullptr;
};

