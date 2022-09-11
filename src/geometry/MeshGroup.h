#pragma once

#include "LinearMath.h"
#include "Mesh.h"

class MeshInstance
{
public:
    int   mesh_id = -1;
    int   mat_id  = -1;
    mat44 xform;
};

class MeshGroup : public Geometry
{
public:
    MeshGroup();

    void addMesh(const Mesh* mesh);

    void addInstance(const MeshInstance& mesh_instance);

    bool intersect(const Ray& ray, float& tNear) const override;

    AABB boundingBox() const;

public:
    std::vector<const Mesh*>  m_meshes;
    std::vector<MeshInstance> m_instances;
};