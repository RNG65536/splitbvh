#include "MeshGroup.h"

MeshGroup::MeshGroup() {}

void MeshGroup::addMesh(const Mesh* mesh) { m_meshes.push_back(mesh); }

void MeshGroup::addInstance(const MeshInstance& mesh_instance)
{
    m_instances.push_back(mesh_instance);
}

bool MeshGroup::intersect(const Ray& ray, float& tNear) const
{
    tNear        = kInfinity;
    bool hit_any = false;

    Ray rayLocal;
    rayLocal.setTMin(ray.tmin());
    rayLocal.setTMax(ray.tmax());

    for (auto& i : m_instances)
    {
        int   mesh_id = i.mesh_id;
        int   mat_id  = i.mat_id;
        mat44 xform   = i.xform;
        xform         = inverse(xform);

        auto& mesh = *m_meshes[mesh_id];

        float3 orig = float3(xform * float4(ray.origin(), 1.0f));
        float3 dir  = float3(xform * float4(ray.direction(), 0.0f));

        rayLocal.setOrigin(orig);
        rayLocal.setDirection(dir);

        float t;
        bool  hit = mesh.intersect(rayLocal, t);
        if (hit)
        {
            tNear = fminf(t, tNear);
            //printf("hit %f\n", tNear);
        }
        else
        {
            //printf("no hit\n");
        }

        hit_any = hit_any | hit;
        //printf("hit any : %s\n", hit_any ? "true" : "false");
    }

    return hit_any;
}

AABB MeshGroup::boundingBox() const
{
    AABB bbox;
    for (auto& i : m_instances)
    {
        int   mesh_id = i.mesh_id;
        int   mat_id  = i.mat_id;
        mat44 xform   = i.xform;

        auto& mesh = *m_meshes[mesh_id];
        auto  c    = mesh.boundingBox().getCorners();
        for (auto& p : c)
        {
            p = xform * float4(p, 1.0f);
            bbox.expandBy(p);
        }
    }
    return bbox;
}
