//#include <GL/freeglut.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>

#include "BVH.h"
#include "Intersection.h"
#include "Logger.h"
#include "Math.h"
#include "Mesh.h"
#include "MeshGroup.h"
#include "Ray.h"
#include "SAHBVH.h"
#include "SplitBVH.h"
#include "Triangle.h"
#include "nvsbvh.h"
#include "rrbvh.h"

using std::endl;

// bool checkEqual(float f1, float f2) { return fabs(f1 - f2) < 1e-6f; }
bool checkEqual(float f1, float f2) { return fabs(f1 - f2) < 1e-5f; }
// bool checkEqual(float f1, float f2) { return fabs(f1 - f2) < 1e-4f; }

class IntersectionTriangle : public Intersection
{
public:
    IntersectionTriangle(const Triangle& triangle) : m_triangle(triangle) {}

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_triangle.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_triangle.intersect(ray, t);
        return t;
    }

private:
    const Triangle& m_triangle;
};

class IntersectionBoundedTriangle : public Intersection
{
public:
    IntersectionBoundedTriangle(const BoundedTriangle& triangle)
        : m_triangle(triangle)
    {
    }

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_triangle.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_triangle.intersect(ray, t);
        return t;
    }

private:
    const BoundedTriangle& m_triangle;
};

class IntersectionMesh : public Intersection
{
public:
    IntersectionMesh(const Mesh& mesh) : m_mesh(mesh) {}

    float intersectWith(const Ray& ray) const
    {
        float t = kInfinity;
        m_mesh.intersect(ray, t);
        return t;
    }

private:
    const Mesh& m_mesh;
};

class IntersectionMeshGroup : public Intersection
{
public:
    IntersectionMeshGroup(const MeshGroup& mesh_group)
        : m_mesh_group(mesh_group)
    {
    }

    float intersectWith(const Ray& ray) const
    {
        float t = kInfinity;
        m_mesh_group.intersect(ray, t);
        return t;
    }

private:
    const MeshGroup& m_mesh_group;
};

class IntersectionNaiveBVH : public Intersection
{
public:
    IntersectionNaiveBVH(const BVH& bvh) : m_bvh(bvh) {}

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_bvh.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_bvh.intersect(ray, t);
        return t;
    }

private:
    const BVH& m_bvh;
};

class IntersectionSAHBVH : public Intersection
{
public:
    IntersectionSAHBVH(const SAHBVH& bvh) : m_bvh(bvh) {}

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_bvh.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_bvh.intersect(ray, t);
        return t;
    }

private:
    const SAHBVH& m_bvh;
};

class IntersectionSplitBVH : public Intersection
{
public:
    IntersectionSplitBVH(const SplitBVH& bvh) : m_bvh(bvh) {}

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_bvh.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_bvh.intersect(ray, t);
        return t;
    }

private:
    const SplitBVH& m_bvh;
};

class IntersectionNvSplitBVH : public Intersection
{
public:
    IntersectionNvSplitBVH(const nvsbvh::NvSplitBVH& bvh) : m_bvh(bvh) {}

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_bvh.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_bvh.intersect(ray, t);
        return t;
    }

private:
    const nvsbvh::NvSplitBVH& m_bvh;
};

class IntersectionRRBVH : public Intersection
{
public:
    IntersectionRRBVH(const rrbvh::RRBVH& bvh) : m_bvh(bvh) {}

    float intersectWith(const Ray& ray) const override
    {
        // HitInfo hit_info;
        // m_bvh.intersect(ray, hit_info);
        // return hit_info.m_t;
        float t = kInfinity;
        m_bvh.intersect(ray, t);
        return t;
    }

private:
    const rrbvh::RRBVH& m_bvh;
};

//////////////////////////////////////////

class Benchmark
{
public:
    Benchmark(const Intersection& reference,
              const Intersection& subject,
              const std::string&  name)
        : m_reference(reference), m_subject(subject), m_name(name)
    {
    }

    void run()
    {
        // benchmark build performance
        // exit(0);

        // while (0)
        int test_total = 100000;
        // int test_total = 2000;
        int test_step = test_total / 100;
        for (int i = 0; i < test_total; i++)
        {
            float3 origin(randf() * 2.0f - 1.0f,
                          randf() * 2.0f - 1.0f,
                          randf() * 2.0f - 1.0f);
            float3 direction = normalize(
                float3(randf() - 0.5f, randf() - 0.5f, randf() - 0.5f));
            Ray ray(origin, direction);

            float ta = m_reference.intersectWith(ray);
            float tb = m_subject.intersectWith(ray);

            bool no_hit = std::isinf(ta) && std::isinf(tb);
            if (no_hit)
            {
            }
            else if (checkEqual(ta, tb))
            {
                // Logger::info()
                //    << "passed: ta = " << ta << ", tb = " << tb << endl;
            }
            else
            {
                Logger::info() << "failed: ta = " << ta << ", tb = " << tb
                               << ", diff = " << (ta - tb) << endl;
                Logger::info() << "ray = " << ray.toString() << endl;
                Logger::info() << endl;

                // exit(1);
            }

            if (0 == i % test_step)
            {
                Logger::info()
                    << "checking " << i / test_step << "%" << std::endl;
            }
        }

        int                 n_buffer = 10000000;
        std::vector<float3> ray_origins(n_buffer);
        std::vector<float3> ray_directions(n_buffer);
        for (int i = 0; i < n_buffer; i++)
        {
            float3 origin(randf() * 2.0f - 1.0f,
                          randf() * 2.0f - 1.0f,
                          randf() * 2.0f - 1.0f);
            float3 direction = normalize(
                float3(randf() - 0.5f, randf() - 0.5f, randf() - 0.5f));
            ray_origins[i]    = origin;
            ray_directions[i] = direction;
        }

        while (1)
        // for (int i = 0; i < 10; i++)
        {
            // int  n_rays = 1000000;
            auto begin = std::chrono::steady_clock::now();
#pragma omp parallel for
            for (int i = 0; i < m_n_rays; i++)
            {
                Ray ray(ray_origins[i % n_buffer],
                        ray_directions[i % n_buffer]);

                m_subject.intersectWith(ray);
            }
            auto end = std::chrono::steady_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                      begin);
            auto  duration_f    = float(duration.count());
            float mrays_per_sec = m_n_rays / duration_f;
            Logger::info() << m_name << " : " << mrays_per_sec
                           << " Mrays / s, with " << m_n_rays << " rays"
                           << endl;

            // update ray count to roughly specify interval
            float interval = 1.0f;
            m_n_rays       = int(mrays_per_sec * 1e6f * interval);
        }
    }

    void run_debug()
    {
// float3 origin(0.6219, 0.199112, 0.867451);
// float3 direction = normalize(float3(-0.706185, -0.0313574,
// 0.707333));
// float3 origin(-0.684774, 0.93539, 0.929777);
// float3 direction = normalize(float3(0.677735, 0.66292, 0.318138));
// float3 origin(-0.684774, 0.93539, 0.929777);
// float3 direction = normalize(float3(0.677735, 0.66292, 0.318138));
// float3 origin(0.811584, -0.729046, 0.629447);
// float3 direction = normalize(float3(0.683038, -0.543399, 0.488034));
// float3 origin(-0.0292487, -0.780276, 0.914334);
// float3 direction = normalize(float3(-0.432506, 0.639862, 0.635228));
// float3 origin(0.178132, -0.148887, 0.866126);
// float3 direction = normalize(float3(-0.71212, -0.574688, 0.40326));
#if 1
        float3 origin(0.24495, -0.22277, 0.100313);
        float3 direction = normalize(float3(-0.0795025, 0.29622, 0.951805));
#else
        float3 origin(0.524838, -0.905342, 0.927126);
        float3 direction = normalize(float3(0.927929, 0.287957, -0.236705));
#endif
        //
        Ray ray(origin, direction);

        float ta = m_reference.intersectWith(ray);
        float tb = m_subject.intersectWith(ray);

        bool no_hit = std::isinf(ta) && std::isinf(tb);
        if (no_hit)
        {
            Logger::info() << "no hit: ta = " << ta << ", tb = " << tb << endl;
        }
        else if (checkEqual(ta, tb))
        {
            Logger::info() << "passed: ta = " << ta << ", tb = " << tb << endl;
        }
        else
        {
            Logger::info() << "failed: ta = " << ta << ", tb = " << tb
                           << ", diff = " << (ta - tb) << endl;
            Logger::info() << "ray = " << ray.toString() << endl;
            Logger::info() << endl;

            // exit(1);
        }
    }

private:
    const Intersection& m_reference;
    const Intersection& m_subject;
    std::string         m_name;
    int                 m_n_rays = 10000;
};

void benchmark_linear_traversal(const Mesh& mesh, const Intersection& ref)
{
    // median split bvh
    IntersectionMesh isect3(mesh);
    Benchmark(ref, isect3, "linear").run();
}

void benchmark_naive_bvh(const Mesh& mesh, const Intersection& ref)
{
    // median split bvh
    BVH                  bvh(mesh);
    IntersectionNaiveBVH isect4(bvh);
    Benchmark(ref, isect4, "naive bvh").run();
}

void benchmark_sah_bvh(const Mesh& mesh, const Intersection& ref)
{
    // SAH bvh
    SAHBVH             sahbvh(mesh);
    IntersectionSAHBVH isect5(sahbvh);
    Benchmark(ref, isect5, "sah bvh").run();
}

void benchmark_split_bvh(const Mesh& mesh, const Intersection& ref)
{
    // split bvh
    // performance seems to be unstable ??
    SplitBVH             splitbvh(mesh);
    IntersectionSplitBVH isect6(splitbvh);
    Benchmark(ref, isect6, "split bvh").run();
}

void benchmark_split_bvh_nvsbvh(const Mesh& mesh, const Intersection& ref)
{
    // split bvh
    // performance seems to be unstable ??
    nvsbvh::NvSplitBVH     splitbvh(mesh);
    IntersectionNvSplitBVH isect6(splitbvh);
    Benchmark(ref, isect6, "split bvh").run();
    // Benchmark(ref, isect6, "split bvh").run_debug();
}

void benchmark_split_bvh_rrbvh(const MeshGroup& mesh, const Intersection& ref)
{
    // split bvh
    // performance seems to be unstable ??
    rrbvh::RRBVH      splitbvh(mesh);
    IntersectionRRBVH isect6(splitbvh);
    Benchmark(ref, isect6, "split bvh").run();
    // Benchmark(ref, isect6, "split bvh").run_debug();
}

void check_mesh_group_isect()
{
    Mesh             mesh;
    IntersectionMesh ref(mesh);
    ;
    MeshGroup mesh_group;
    mesh_group.addMesh(&mesh);
    mesh_group.addInstance(MeshInstance{0, -1, mat44()});
    IntersectionMeshGroup test(mesh_group);
    ;
    Benchmark(ref, test, "mesh group isect").run();
}

mat44 compose(const float3& rotation,
              const float3& translation,
              const float3& scaling)
{
    mat44 rx;
    rx[1][1] = cos(rotation.x);
    rx[2][1] = sin(rotation.x);
    rx[1][2] = -sin(rotation.x);
    rx[2][2] = cos(rotation.x);

    mat44 ry;
    ry[2][2] = cos(rotation.y);
    ry[0][2] = sin(rotation.y);
    ry[2][0] = -sin(rotation.y);
    ry[0][0] = cos(rotation.y);

    mat44 rz;
    ry[0][0] = cos(rotation.z);
    ry[1][0] = sin(rotation.z);
    ry[0][1] = -sin(rotation.z);
    ry[1][1] = cos(rotation.z);

    mat44 r = rz * ry * rx;

    mat44 t;
    t[3][0] = translation.x;
    t[3][1] = translation.y;
    t[3][2] = translation.z;

    mat44 s;
    s[0][0] = scaling.x;
    s[1][1] = scaling.y;
    s[2][2] = scaling.z;

    return t * r * s;
}

int main()
{
    // check_mesh_group_isect();
    // return 0;

    // naive scan
    Mesh             mesh;
    IntersectionMesh isect3(mesh);
    Logger::info() << "mesh AABB = " << mesh.boundingBox().toString() << endl;
    Logger::info() << "#######################################################"
                   << endl;

    MeshGroup mesh_group;
    mesh_group.addMesh(&mesh);
    if (1)
    {
        // identity transform
        mesh_group.addInstance(MeshInstance{
            0, -1, compose(float3(0.0f), float3(0.0f), float3(1.0f))});
    }
    if (1)
    {
        if (1)
        {
            mesh_group.addInstance(MeshInstance{
                0,
                -1,
                compose(float3(0.0f), float3(2.0f, 0.5f, 0.3f), float3(1.2f))});
        }
        if (1)
        {
            mesh_group.addInstance(MeshInstance{
                0,
                -1,
                compose(
                    float3(0.0f), float3(1.0f, -0.5f, 0.3f), float3(0.5f))});
        }
    }

    IntersectionMeshGroup isect4(mesh_group);
    Logger::info() << "mesh AABB = " << mesh_group.boundingBox().toString()
                   << endl;
    Logger::info() << "#######################################################"
                   << endl;

    // benchmark_linear_traversal(mesh, isect3);
    // benchmark_naive_bvh(mesh, isect3);
    // benchmark_sah_bvh(mesh, isect3);
    // benchmark_split_bvh(mesh, isect3);
    // benchmark_split_bvh_nvsbvh(mesh, isect3);
    // benchmark_split_bvh_rrbvh(mesh_group, isect3);
    benchmark_split_bvh_rrbvh(mesh_group, isect4);

    return 0;
}

void debug()
{
    // float3 origin(0.1f, 0.2f, 3.0f);
    float3 origin(0.0f, 0.0f, 3.0f);
    float3 direction(0.0f, 0.0f, -1.0f);
    Ray    ray(origin, direction);

    float3   va(-1.0f, -1.0f, -2.0f);
    float3   vb(1.0f, -1.0f, -2.0f);
    float3   vc(0.0f, 1.0f, -4.0f);
    Triangle triangle1(va, vb, vc);
    Triangle triangle2(va, vc, vb);

    // Intersection isect1(&triangle1);
    // float        t1 = isect1.intersectWith(ray);
    // Intersection isect2(&triangle2);
    // float        t2 = isect2.intersectWith(ray);
    // float        t3 = isect3.intersectWith(ray);
    // float        t4 = isect4.intersectWith(ray);
    // float        t5 = isect5.intersectWith(ray);
    // float t6 = isect6.intersectWith(ray);

    // Logger::info() << "isect with triangle1 = " << t1 << endl;
    // Logger::info() << "isect with triangle2 = " << t2 << endl;
    // Logger::info() << "isect with mesh = " << t3 << endl;
    // Logger::info() << "isect with mesh with bvh = " << t4 << endl;

    //{
    //    // float3 o1(-0.0482297, -0.477552, 1.78652);
    //    // float3 d1(0.485279, -0.867193, -0.111716);

    //    float3 o1(0.63693, -0.34375, 0.953973);
    //    float3 d1(-0.361296, -0.568984, 0.73873);

    //    d1 = normalize(d1);

    //    Ray ray(o1, d1);

    //    float ta = isect3.intersectWith(ray);
    //    float tb = isect4.intersectWith(ray);

    //    Logger::info() << "isect with mesh brute force = " << ta << endl;
    //    Logger::info() << "isect with mesh with bvh = " << tb << endl;
    //}
}