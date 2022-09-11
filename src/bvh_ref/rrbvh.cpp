//#include <glm/glm.hpp>
//#include <glm/gtc/matrix_inverse.hpp>
//
//#include "rrbvh.h"
//
// void testete()
//{
//
//    glm::vec4 b;
//    glm::mat4 a(b, b, b, b);
//    glm::inverse(a);
//
//    a[0];
//
//    glm::vec4 d;
//}

#include "rrbvh.h"

#define USE_COMPACT_BVH 1

namespace rrbvh
{
struct FunctionReturnGuard
{
    FunctionReturnGuard() {}
    ~FunctionReturnGuard()
    {
        for (int i = 0; i < 20; i++)
        {
            printf("\n");
        }
        getchar();
    }
};

static float int_as_float(int i) { return *reinterpret_cast<float*>(&i); }

template <typename T>
static std::vector<T> alloc_and_copy(const T* h_ptr, size_t count)
{
    //void* d_ptr = malloc(sizeof(T) * count);
    //if (d_ptr)
    //    memcpy(d_ptr, h_ptr, sizeof(T) * count);
    //else
    //    throw;
    //return reinterpret_cast<T*>(d_ptr);

    return std::vector<T>(h_ptr, h_ptr + count);
}

#if 0
static float AABBIntersect(const float3& minCorner,
                           const float3& maxCorner,
                           const Ray&    r,
                           float         ray_tmin,
                           float         ray_tmax)
{
    float3 inv_dir = 1.0f / r.direction();
    float3 f       = (maxCorner - r.origin()) * inv_dir;
    float3 n       = (minCorner - r.origin()) * inv_dir;
    float3 tmax    = max(f, n);
    float3 tmin    = min(f, n);
    // float  t1      = fminf(fminf(tmax.x, tmax.y), tmax.z);
    // float  t0      = fmaxf(fmaxf(tmin.x, tmin.y), tmin.z);
    // return (t1 >= t0) ? (t0 > 0.0f ? t0 : t1) : -1.0f;
    float t1 = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), ray_tmax);
    float t0 = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), ray_tmin);
    return (t1 >= t0) ? t0 : -1.0f;
}
#else
static float AABBIntersect(const float3& bounds_min,
                           const float3& bounds_max,
                           const Ray&    r,
                           float         ray_tmin,
                           float         ray_tmax)
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
        return -1.0f;
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
        return -1.0f;
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
    // float rayTMax = r.tmax();
    float rayTMax = ray_tmax;
    if (tmin > rayTMax)
    {
        return -1.0f;
    }
    // float rayTMin = r.tmin();
    float rayTMin = ray_tmin;
    if (tmax < rayTMin)
    {
        return -1.0f;
    }

    return fmaxf(tmin, rayTMin);  // FIXED
}
#endif

#if 0
    static bool AABBIntersect(const float3& minCorner,
                               const float3& maxCorner,
                               const Ray&    r,
        float ray_tmin, float ray_tmax,
        float& t_near)
    {
        float3 inv_dir = 1.0f / r.direction();
        float3 f       = (maxCorner - r.origin()) * inv_dir;
        float3 n       = (minCorner - r.origin()) * inv_dir;
        float3 tmax    = max(f, n);
        float3 tmin    = min(f, n);
        //float  t1      = fminf(fminf(tmax.x, tmax.y), tmax.z);
        //float  t0      = fmaxf(fmaxf(tmin.x, tmin.y), tmin.z);
        //return (t1 >= t0) ? (t0 > 0.0f ? t0 : t1) : -1.0f;
        float  t1      = fminf(fminf(fminf(tmax.x, tmax.y), tmax.z), r.tmax());
        float  t0      = fmaxf(fmaxf(fmaxf(tmin.x, tmin.y), tmin.z), r.tmin());
        if (t1 >= t0)
        {
            t_near = t0;
            return true;
        }
        else{
            t_near = kInfinity;
            return false;
        }
    }
#else
static bool AABBIntersect(const float3& bounds_min,
                          const float3& bounds_max,
                          const Ray&    r,
                          float         ray_tmin,
                          float         ray_tmax,
                          float&        t_near)
{
    t_near = kInfinity;

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
    // float rayTMax = r.tmax();
    float rayTMax = ray_tmax;
    if (tmin > rayTMax)
    {
        return false;
    }
    // float rayTMin = r.tmin();
    float rayTMin = ray_tmin;
    if (tmax < rayTMin)
    {
        return false;
    }

    t_near = fmaxf(tmin, rayTMin);  // FIXED
    return true;
}
#endif

RRBVH::RRBVH(const MeshGroup& mesh)
{
    Scene scene;
#if 0
    {
        RMesh* rmesh = new RMesh;
        for (auto& tri : mesh)
        {
            auto& a = tri.va();
            auto& b = tri.vb();
            auto& c = tri.vc();
            Vec3  n = normalize(cross(b - a, c - a));

            rmesh->verticesUVX.push_back(Vec4(a.x, a.y, a.z, 0.0f));
            rmesh->verticesUVX.push_back(Vec4(b.x, b.y, b.z, 0.0f));
            rmesh->verticesUVX.push_back(Vec4(c.x, c.y, c.z, 0.0f));
            rmesh->normalsUVY.push_back(Vec4(n.x, n.y, n.z, 0.0f));
            rmesh->normalsUVY.push_back(Vec4(n.x, n.y, n.z, 0.0f));
            rmesh->normalsUVY.push_back(Vec4(n.x, n.y, n.z, 0.0f));
        }
        scene.meshes.push_back(rmesh);

        Mat4         xform;
        MeshInstance inst("", 0, xform, -1);
        scene.AddMeshInstance(inst);
    }
#else
    for (auto& m : mesh.m_meshes)
    {
        RMesh* rmesh = new RMesh;
        for (auto& tri : *m)
        {
            auto& a = tri.va();
            auto& b = tri.vb();
            auto& c = tri.vc();
            Vec3  n = normalize(cross(b - a, c - a));

            rmesh->verticesUVX.push_back(Vec4(a.x, a.y, a.z, 0.0f));
            rmesh->verticesUVX.push_back(Vec4(b.x, b.y, b.z, 0.0f));
            rmesh->verticesUVX.push_back(Vec4(c.x, c.y, c.z, 0.0f));
            rmesh->normalsUVY.push_back(Vec4(n.x, n.y, n.z, 0.0f));
            rmesh->normalsUVY.push_back(Vec4(n.x, n.y, n.z, 0.0f));
            rmesh->normalsUVY.push_back(Vec4(n.x, n.y, n.z, 0.0f));
        }
        scene.meshes.push_back(rmesh);
    }
    for (auto& i : mesh.m_instances)
    {
        MeshInstance inst("", i.mesh_id, i.xform, i.mat_id);
        scene.AddMeshInstance(inst);
    }
#endif

    scene.ProcessScene();
    ;

    bvh_buffer = alloc_and_copy<BvhTranslator::Node>(
        scene.bvhTranslator.nodes.data(), scene.bvhTranslator.nodes.size());
    vertex_indices_buffer = alloc_and_copy<Indices>(scene.vertIndices.data(),
                                                    scene.vertIndices.size());
    vertices_buffer       = alloc_and_copy<Vec4>(scene.verticesUVX.data(),
                                           scene.verticesUVX.size());
    normals_buffer =
        alloc_and_copy<Vec4>(scene.normalsUVY.data(), scene.normalsUVY.size());

    std::vector<Mat4> inv_xforms(scene.transforms.size());
    for (int i = 0; i < scene.transforms.size(); i++)
    {
        inv_xforms[i] = inverse(scene.transforms[i]);
    }
    transforms_buffer =
        alloc_and_copy<Mat4>(inv_xforms.data(), scene.transforms.size());

    bvh_tex            = reinterpret_cast<float3*>(bvh_buffer.data());
    vertex_indices_tex = reinterpret_cast<int3*>(vertex_indices_buffer.data());
    vertices_tex       = reinterpret_cast<float4*>(vertices_buffer.data());
    normals_tex        = reinterpret_cast<float4*>(normals_buffer.data());
    transforms_tex     = reinterpret_cast<float4*>(transforms_buffer.data());
    topBVHIndex        = scene.bvhTranslator.topLevelIndex;
    num_nodes          = scene.bvhTranslator.nodes.size();

#if USE_COMPACT_BVH
    this->woopify_tri();
    tri_woop_tex       = reinterpret_cast<float4*>(tri_woop_buffer.data());
    this->convert_to_compact();
#endif
}

RRBVH::~RRBVH()
{
}

#if 1
#define check(x) void(0)
#define check2(x) void(0)
#else
#define check(x) void(0)
#define check2(x) x
#endif

void RRBVH::convert_to_compact_()
{
    int stack[64];
    int ptr        = 0;
    stack[ptr++]   = -1;
    int  index     = topBVHIndex;
    int  currMatID = 0;
    bool BLAS      = false;

    compact_bvh.clear();
    compact_bvh_top.clear();
    std::vector<int> compact_indices(num_nodes);
    compact_indices.assign(num_nodes, -1);

    while (index != -1)
    {
        float3 LRLeaf = bvh_tex[index * 3 + 2];

        int leftIndex  = int(LRLeaf.x);
        int rightIndex = int(LRLeaf.y);
        int leaf       = int(LRLeaf.z);

        if (leaf == 0)  // not leaf
        {
            check(printf("----- index = %d, leaf = %d (%s branch)\n",
                         index,
                         leaf,
                         BLAS ? "BLAS" : "TLAS"));
            // check(printf(BLAS ? "  BLAS node\n" : "  TLAS node\n"));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            if (BLAS)
            {
                compact_indices[index] = compact_bvh.size();

                float3 c0_min = bvh_tex[leftIndex * 3 + 0];
                float3 c0_max = bvh_tex[leftIndex * 3 + 1];
                float3 c1_min = bvh_tex[rightIndex * 3 + 0];
                float3 c1_max = bvh_tex[rightIndex * 3 + 1];

                CompactNode c;
                c.c0xy     = float4(c0_min.x, c0_max.x, c0_min.y, c0_max.y);
                c.c1xy     = float4(c1_min.x, c1_max.x, c1_min.y, c1_max.y);
                c.c01z     = float4(c0_min.z, c0_max.z, c1_min.z, c1_max.z);
                c.cnodes.x = leftIndex;
                c.cnodes.y = rightIndex;
                c.cnodes.z = index;
                c.cnodes.w = leaf;

                // handle leaves
                {
                    float3 left_c = bvh_tex[leftIndex * 3 + 2];
                    if (int(left_c.z) > 0)
                    {
                        int offset = left_c.x;
                        int count  = left_c.y;
                        check(printf(
                            "  BLAS left leaf: %d, %d\n", offset, count));
                        c.cnodes.x = ~(
                            offset |
                            count << 20);  // TODO encode count in vertex array
                    }

                    float3 right_c = bvh_tex[rightIndex * 3 + 2];
                    if (int(right_c.z) > 0)
                    {
                        int offset = right_c.x;
                        int count  = right_c.y;
                        check(printf(
                            "  BLAS right leaf: %d, %d\n", offset, count));
                        c.cnodes.y = ~(
                            offset |
                            count << 20);  // TODO encode count in vertex array
                    }
                }

                compact_bvh.push_back(c);
            }

            index        = leftIndex;
            stack[ptr++] = rightIndex;  // deferred
            continue;
        }
        else if (leaf < 0)  // top level leaf
        {
            check(printf(
                "----- index = %d, leaf = %d (TLAS leaf)\n", index, leaf));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            {
                CompactNode c;
                c.c0xy     = float4(std::numeric_limits<float>::quiet_NaN());
                c.c1xy     = float4(std::numeric_limits<float>::quiet_NaN());
                c.c01z     = float4(std::numeric_limits<float>::quiet_NaN());
                c.cnodes.x = leftIndex;
                c.cnodes.y = rightIndex;
                c.cnodes.z = index;
                c.cnodes.w = leaf;
                compact_bvh_top.push_back(c);
            }

            stack[ptr++] = -1;
            index        = leftIndex;
            BLAS         = true;
            currMatID    = rightIndex;

            continue;
        }
        else  // leaf > 0 // bottom level leaf
        {
            check(printf(
                "----- index = %d, leaf = %d (BLAS leaf)\n", index, leaf));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            //{
            //    CompactNode& c = compact_bvh[index];
            //    c.c0xy         =
            //    float4(std::numeric_limits<float>::infinity()); c.c1xy =
            //    float4(std::numeric_limits<float>::infinity()); c.c01z =
            //    float4(std::numeric_limits<float>::infinity()); c.cnodes.x =
            //    leftIndex; c.cnodes.y     = rightIndex; c.cnodes.z     =
            //    index; c.cnodes.w     = leaf;
            //}

            for (int i = 0; i < rightIndex; i++)
            {
                int3 vertIndices = vertex_indices_tex[leftIndex + i];

                float4 v0 = vertices_tex[vertIndices.x];
                float4 v1 = vertices_tex[vertIndices.y];
                float4 v2 = vertices_tex[vertIndices.z];
            }
        }

        index = stack[--ptr];
        check(printf("  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));

        if (BLAS && index == -1)
        {
            BLAS = false;

            index = stack[--ptr];
            check(
                printf("  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));
        }
    }

    for (auto& c : compact_bvh)
    {
        if (c.cnodes.w == 0)  // branch node
        {
            if (c.cnodes.x >= 0) c.cnodes.x = compact_indices[c.cnodes.x];
            if (c.cnodes.y >= 0) c.cnodes.y = compact_indices[c.cnodes.y];
            c.cnodes.z = compact_indices[c.cnodes.z];
        }
    }

    this->topBVHIndex = compact_bvh.size();

    // TODO handle TLAS tree
    for (auto& c : compact_bvh_top)
    {
        if (c.cnodes.w < 0)
        {
            if (c.cnodes.x >= 0) c.cnodes.x = compact_indices[c.cnodes.x];
            if (c.cnodes.y >= 0) c.cnodes.y = compact_indices[c.cnodes.y];
            c.cnodes.z = compact_bvh.size();
        }
    }

    compact_bvh.insert(
        compact_bvh.end(), compact_bvh_top.begin(), compact_bvh_top.end());

    // throw;
}

#if 1
#define check(x) void(0)
#define check2(x) void(0)
#else
#define check(x) x
#define check2(x) x
#endif

void RRBVH::convert_to_compact()
{
    compact_bvh.clear();
    compact_bvh_top.clear();
    std::vector<int> compact_indices(num_nodes);
    compact_indices.assign(num_nodes, -1);

    for (int index = 0; index < topBVHIndex; index++)
    {
        float3 LRLeaf     = bvh_tex[index * 3 + 2];
        int    leftIndex  = int(LRLeaf.x);
        int    rightIndex = int(LRLeaf.y);
        int    leaf       = int(LRLeaf.z);

        if (leaf == 0)
        {
            check(printf(
                "----- index = %d, leaf = %d (BLAS branch)\n", index, leaf));
            check(printf("  left index = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            compact_indices[index] = compact_bvh.size();

            float3 c0_min = bvh_tex[leftIndex * 3 + 0];
            float3 c0_max = bvh_tex[leftIndex * 3 + 1];
            float3 c1_min = bvh_tex[rightIndex * 3 + 0];
            float3 c1_max = bvh_tex[rightIndex * 3 + 1];

            CompactNode c;
            c.c0xy     = float4(c0_min.x, c0_max.x, c0_min.y, c0_max.y);
            c.c1xy     = float4(c1_min.x, c1_max.x, c1_min.y, c1_max.y);
            c.c01z     = float4(c0_min.z, c0_max.z, c1_min.z, c1_max.z);
            c.cnodes.x = leftIndex;
            c.cnodes.y = rightIndex;
            c.cnodes.z = index;
            c.cnodes.w = leaf;

            // handle leaves
            {
                float3 left_c = bvh_tex[leftIndex * 3 + 2];
                if (int(left_c.z) > 0)
                {
                    int offset = left_c.x;
                    int count  = left_c.y;
                    check(printf("  BLAS left leaf: %d, %d\n", offset, count));
                    c.cnodes.x =
                        ~(offset |
                          count << 20);  // TODO encode count in vertex array
                }

                float3 right_c = bvh_tex[rightIndex * 3 + 2];
                if (int(right_c.z) > 0)
                {
                    int offset = right_c.x;
                    int count  = right_c.y;
                    check(printf("  BLAS right leaf: %d, %d\n", offset, count));
                    c.cnodes.y =
                        ~(offset |
                          count << 20);  // TODO encode count in vertex array
                }
            }

            compact_bvh.push_back(c);
        }
        else if (leaf > 0)  // bottom level leaf
        {
            check(printf(
                "----- index = %d, leaf = %d (BLAS leaf)\n", index, leaf));
            check(printf("  left index (offset) = %d\n", leftIndex));
            check(printf("  right index (count) = %d\n", rightIndex));
        }
        else
        {
            throw;
        }
    }

    for (auto& c : compact_bvh)
    {
        if (c.cnodes.w == 0)  // branch node
        {
            if (c.cnodes.x >= 0) c.cnodes.x = compact_indices[c.cnodes.x];
            if (c.cnodes.y >= 0) c.cnodes.y = compact_indices[c.cnodes.y];
            c.cnodes.z = compact_indices[c.cnodes.z];
        }
    }

    // for (int i = 0; i < compact_bvh.size(); i++)
    //{
    //    if (compact_bvh[i].cnodes.z != i) throw;
    //    if (compact_bvh[i].cnodes.w != 0) throw;
    //}

    for (int index = topBVHIndex; index < num_nodes; index++)
    {
        float3 LRLeaf     = bvh_tex[index * 3 + 2];
        int    leftIndex  = int(LRLeaf.x);
        int    rightIndex = int(LRLeaf.y);
        int    leaf       = int(LRLeaf.z);

        if (leftIndex == 0 && rightIndex == 0) break;

        if (leaf == 0)
        {
            check(printf(
                "----- index = %d, leaf = %d (TLAS branch)\n", index, leaf));
            check(printf("  left index = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            compact_indices[index] = compact_bvh_top.size();

            float3 c0_min = bvh_tex[leftIndex * 3 + 0];
            float3 c0_max = bvh_tex[leftIndex * 3 + 1];
            float3 c1_min = bvh_tex[rightIndex * 3 + 0];
            float3 c1_max = bvh_tex[rightIndex * 3 + 1];

            CompactNode c;
            c.c0xy     = float4(c0_min.x, c0_max.x, c0_min.y, c0_max.y);
            c.c1xy     = float4(c1_min.x, c1_max.x, c1_min.y, c1_max.y);
            c.c01z     = float4(c0_min.z, c0_max.z, c1_min.z, c1_max.z);
            c.cnodes.x = leftIndex;
            c.cnodes.y = rightIndex;
            c.cnodes.z = index;
            c.cnodes.w = leaf;

            compact_bvh_top.push_back(c);
        }
        else if (leaf < 0)  // top level leaf
        {
            check(printf(
                "----- index = %d, leaf = %d (TLAS leaf)\n", index, leaf));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            compact_indices[index] = compact_bvh_top.size();

            float3 c0_min = bvh_tex[leftIndex * 3 + 0];
            float3 c0_max = bvh_tex[leftIndex * 3 + 1];
            float3 c1_min = float3(std::numeric_limits<float>::quiet_NaN());
            float3 c1_max = float3(std::numeric_limits<float>::quiet_NaN());

            CompactNode c;
            c.c0xy     = float4(c0_min.x, c0_max.x, c0_min.y, c0_max.y);
            c.c1xy     = float4(c1_min.x, c1_max.x, c1_min.y, c1_max.y);
            c.c01z     = float4(c0_min.z, c0_max.z, c1_min.z, c1_max.z);
            c.cnodes.x = leftIndex;
            c.cnodes.y = rightIndex;
            c.cnodes.z = index;
            c.cnodes.w = leaf;
            compact_bvh_top.push_back(c);
        }
        else
        {
            throw;
        }
    }

    this->topBVHIndex = compact_bvh.size();

    // TLAS tree after BLAS tree
    for (auto& c : compact_bvh_top)
    {
        if (c.cnodes.w == 0)  // TLAS branch
        {
            if (c.cnodes.x >= 0)
                c.cnodes.x = compact_indices[c.cnodes.x] + compact_bvh.size();
            if (c.cnodes.y >= 0)
                c.cnodes.y = compact_indices[c.cnodes.y] + compact_bvh.size();
            c.cnodes.z = compact_indices[c.cnodes.z] + compact_bvh.size();
        }
        else if (c.cnodes.w < 0)  // TLAS leaf
        {
            if (c.cnodes.x >= 0) c.cnodes.x = compact_indices[c.cnodes.x];
            if (c.cnodes.y >= 0) c.cnodes.y = compact_indices[c.cnodes.y];
            c.cnodes.z = compact_indices[c.cnodes.z] + compact_bvh.size();
        }
        else
        {
            throw;
        }
    }

    compact_bvh.insert(
        compact_bvh.end(), compact_bvh_top.begin(), compact_bvh_top.end());
}

void RRBVH::woopify_tri()
{
    int num_triangles = this->vertex_indices_buffer.size();
    std::cout << "#tri = " << num_triangles << std::endl;

    struct TriWoop
    {
        float4 v0;
        float4 v1;
        float4 v2;
    };

    this->tri_woop_buffer.resize(num_triangles * 3);
    for (int i = 0; i < num_triangles; i++)
    {
        Indices f = this->vertex_indices_buffer[i];
        float3 v0 = float3(this->vertices_buffer[f.x]);
        float3 v1 = float3(this->vertices_buffer[f.y]);
        float3 v2 = float3(this->vertices_buffer[f.z]);

        mat44 mtx;
        mtx[0] = float4(v0 - v2, 0.0f);
        mtx[1] = float4(v1 - v2, 0.0f);
        mtx[2] = float4(cross(v0 - v2, v1 - v2), 0.0f);
        mtx[3] = float4(v2, 1.0f);
        mtx = inverse(mtx);

        TriWoop woop;
        woop.v0 = float4(mtx[0][2], mtx[1][2], mtx[2][2], -mtx[3][2]); // third row
        woop.v1 = float4(mtx[0][0], mtx[1][0], mtx[2][0], mtx[3][0]); // first row
        woop.v2 = float4(mtx[0][1], mtx[1][1], mtx[2][1], mtx[3][1]); // second row
        this->tri_woop_buffer[i * 3 + 0] = woop.v0;
        this->tri_woop_buffer[i * 3 + 1] = woop.v1;
        this->tri_woop_buffer[i * 3 + 2] = woop.v2;
    }
}

#if 1
#define check(x) void(0)
#define check2(x) void(0)
#else
#define check(x) x
#define check2(x) x
#endif

bool RRBVH::intersect0(const Ray& ray, float& tNear) const
{
    // FunctionReturnGuard guard;

    check(printf("========================================\n"));
    check(printf("ray: o = %f, %f, %f, d = %f, %f, %f\n",
                 ray.origin().x,
                 ray.origin().y,
                 ray.origin().z,
                 ray.direction().x,
                 ray.direction().y,
                 ray.direction().z));
    check(printf("top bvh index = %d\n", topBVHIndex));

    ;
    float t = kInfinity;

    int stack[64];
    int ptr      = 0;
    stack[ptr++] = -1;

    int index = topBVHIndex;
    // int   index    = 0; // works if there is only one instance

    float leftHit  = 0.0;
    float rightHit = 0.0;

    int  currMatID = 0;
    bool BLAS      = false;

    int3   triID = int3(-1, -1, -1);
    mat44  transMat;
    mat44  transform;
    float3 bary;
    float4 vert0, vert1, vert2;

    Ray rTrans = ray;

    while (index != -1)
    {
        float3 LRLeaf = bvh_tex[index * 3 + 2];

        int leftIndex  = int(LRLeaf.x);
        int rightIndex = int(LRLeaf.y);
        int leaf       = int(LRLeaf.z);

        if (leaf == 0)  // not leaf
        {
            check(printf("----- index = %d, leaf = %d (%s branch)\n",
                         index,
                         leaf,
                         BLAS ? "BLAS" : "TLAS"));
            // check(printf(BLAS ? "  BLAS node\n" : "  TLAS node\n"));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            // leftHit = AABBIntersect(
            //    bvh_tex[leftIndex * 3 + 0], bvh_tex[leftIndex * 3 + 1],
            //    rTrans);
            // rightHit = AABBIntersect(bvh_tex[rightIndex * 3 + 0],
            //                         bvh_tex[rightIndex * 3 + 1],
            //                         rTrans);
            leftHit  = AABBIntersect(bvh_tex[leftIndex * 3 + 0],
                                    bvh_tex[leftIndex * 3 + 1],
                                    rTrans,
                                    ray.tmin(),
                                    fminf(ray.tmax(), t));
            rightHit = AABBIntersect(bvh_tex[rightIndex * 3 + 0],
                                     bvh_tex[rightIndex * 3 + 1],
                                     rTrans,
                                     ray.tmin(),
                                     fminf(ray.tmax(), t));

#if 1
            if (leftHit > 0.0f && rightHit > 0.0f)
            {
                check(printf("  hit both\n"));
                int deferred = -1;
                if (leftHit > rightHit)
                {
                    index    = rightIndex;
                    deferred = leftIndex;
                }
                else
                {
                    index    = leftIndex;
                    deferred = rightIndex;
                }

                stack[ptr++] = deferred;
                continue;
            }
            else if (leftHit > 0.0f)
            {
                check(printf("  hit left\n"));
                index = leftIndex;
                continue;
            }
            else if (rightHit > 0.0f)
            {
                check(printf("  hit right\n"));
                index = rightIndex;
                continue;
            }
            else
            {
                check(printf("  hit none\n"));
            }
#else
#endif
        }
        else if (leaf < 0)  // top level leaf
        {
            check(printf(
                "----- index = %d, leaf = %d (TLAS leaf)\n", index, leaf));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));
#if 1
            // this is slow
            float4 c0 = transforms_tex[(-leaf - 1) * 4 + 0];
            float4 c1 = transforms_tex[(-leaf - 1) * 4 + 1];
            float4 c2 = transforms_tex[(-leaf - 1) * 4 + 2];
            float4 c3 = transforms_tex[(-leaf - 1) * 4 + 3];

            transMat = mat44(c0, c1, c2, c3);
            rTrans.setOrigin(float3(transMat * float4(ray.origin(), 1.0f)));
            rTrans.setDirection(
                float3(transMat * float4(ray.direction(), 0.0f)));
#else
            rTrans = ray;
#endif

            check(printf("  ray: o(%f, %f, %f), d(%f, %f, %f)\n",
                         rTrans.origin().x,
                         rTrans.origin().y,
                         rTrans.origin().z,
                         rTrans.direction().x,
                         rTrans.direction().y,
                         rTrans.direction().z));

            stack[ptr++] = -1;
            index        = leftIndex;
            BLAS         = true;
            currMatID    = rightIndex;

            continue;
        }
        else  // leaf > 0 // bottom level leaf
        {
            check(printf(
                "----- index = %d, leaf = %d (BLAS leaf)\n", index, leaf));
            check(printf("  left index (offset) = %d\n", leftIndex));
            check(printf("  right index (count) = %d\n", rightIndex));

            check(printf("  ray: o(%f, %f, %f), d(%f, %f, %f)\n",
                         rTrans.origin().x,
                         rTrans.origin().y,
                         rTrans.origin().z,
                         rTrans.direction().x,
                         rTrans.direction().y,
                         rTrans.direction().z));

            for (int i = 0; i < rightIndex; i++)
            {
                int3 vertIndices = vertex_indices_tex[leftIndex + i];

                float4 v0 = vertices_tex[vertIndices.x];
                float4 v1 = vertices_tex[vertIndices.y];
                float4 v2 = vertices_tex[vertIndices.z];

                float3 e0  = float3(v1) - float3(v0);
                float3 e1  = float3(v2) - float3(v0);
                float3 pv  = cross(rTrans.direction(), e1);
                float  det = dot(e0, pv);

                float3 tv = rTrans.origin() - float3(v0);
                float3 qv = cross(tv, e0);

                float4 uvt;
                uvt.x = dot(tv, pv);
                uvt.y = dot(rTrans.direction(), qv);
                uvt.z = dot(e1, qv);
                uvt.x /= det;
                uvt.y /= det;
                uvt.z /= det;
                uvt.w = 1.0f - uvt.x - uvt.y;

                check(printf("  hit dist = %f\n", uvt.z));

                if (uvt.x >= 0 && uvt.y >= 0 && uvt.z >= 0 && uvt.w >= 0 &&
                    uvt.z < t && uvt.z > ray.tmin())
                {
                    t         = uvt.z;
                    triID     = vertIndices;
                    bary      = float3(uvt.w, uvt.x, uvt.y);
                    vert0     = v0;
                    vert1     = v1;
                    vert2     = v2;
                    transform = transMat;
                }
            }
        }

        index = stack[--ptr];
        check(printf("  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));

        if (BLAS && index == -1)
        {
            BLAS = false;

            index = stack[--ptr];
            check(
                printf("  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));

#if 1
            rTrans.setOrigin(ray.origin());
            rTrans.setDirection(ray.direction());
#else
            rTrans = ray;
#endif
        }
    }

    tNear = t;

    // if (tNear == kInfinity)
    //{
    //    return false;
    //}
    // else
    //{
    //    return true;
    //}
    return tNear < kInfinity;
}

static inline float3 get_c0_min(const RRBVH::CompactNode& c)
{
    return float3(c.c0xy.x, c.c0xy.z, c.c01z.x);
}

static inline float3 get_c0_max(const RRBVH::CompactNode& c)
{
    return float3(c.c0xy.y, c.c0xy.w, c.c01z.y);
}

static inline float3 get_c1_min(const RRBVH::CompactNode& c)
{
    return float3(c.c1xy.x, c.c1xy.z, c.c01z.z);
}

static inline float3 get_c1_max(const RRBVH::CompactNode& c)
{
    return float3(c.c1xy.y, c.c1xy.w, c.c01z.w);
}

#include "rrbvh_temp.h"
#include "rrbvh_temp2.h"
#include "rrbvh_temp3.h"
#include "rrbvh_temp4.h"
#include "rrbvh_temp5.h"

bool RRBVH::intersect(const Ray& ray, float& tNear) const
{
#if USE_COMPACT_BVH
    // requires convert_to_compact
    // return intersect1(ray, tNear);
    // return intersect2(ray, tNear);
    //return intersect3(ray, tNear);
    //return intersect4(ray, tNear);
    return intersect5(ray, tNear);
#else
    return intersect0(ray, tNear);
#endif
}
}  // namespace rrbvh