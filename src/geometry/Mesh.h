#pragma once

#include <vector>

#include "Geometry.h"
#include "Triangle.h"

class Mesh : public Geometry
{
public:
    Mesh();
    Mesh(const std::string& folder, const std::string& filename);

    bool intersect(const Ray& ray, float& tNear) const override;

    std::vector<Triangle>::const_iterator begin() const
    {
        return m_triangles.cbegin();
    }

    std::vector<Triangle>::const_iterator end() const
    {
        return m_triangles.cend();
    }

    const AABB& boundingBox() const { return m_boundingBox; }

private:
    void initMesh();
    void initMeshDebug();
    void initRandomMesh();

    std::vector<Triangle> m_triangles;
    AABB                  m_boundingBox;
};