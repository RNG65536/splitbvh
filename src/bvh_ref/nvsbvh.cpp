#include "nvsbvh.h"

#include "Ray.h"
#include "nvsbvh/CudaBVH.h"

#define BLD 0

namespace nvsbvh
{
NvSplitBVH::NvSplitBVH(const Mesh& mesh)
{
    TriangleMesh trimesh;
    {
        // mesh->trimesh
        int offset = 0;
        for (auto& tri : mesh)
        {
            TriangleVertex v;

            v.position = tri.va();
            trimesh.addTriangleVertex(v);
            v.position = tri.vb();
            trimesh.addTriangleVertex(v);
            v.position = tri.vc();
            trimesh.addTriangleVertex(v);

            TriangleFace f;
            f.ia = offset;
            f.ib = offset + 1;
            f.ic = offset + 2;
            offset += 3;
            trimesh.addTriangleFace(f);
        }
    }

    auto vertices  = trimesh.getVerts();
    auto triangles = trimesh.getFaces();

    // build bvh
    // create arrays for the triangles and the vertices

    Array<Scene::Triangle> tris;
    Array<Vec3f>           verts;

    std::vector<S32> tri_mat_ids;

    // convert Triangle to Scene::Triangle
    for (unsigned int i = 0; i < triangles.size(); i++)
    {
        Scene::Triangle newtri;
        newtri.vertices =
            Vec3i(triangles[i].ia, triangles[i].ib, triangles[i].ic);
        tris.add(newtri);

        tri_mat_ids.push_back(static_cast<S32>(triangles[i].mat_id));
    }

    // fill up Array of vertices
    for (unsigned int i = 0; i < vertices.size(); i++)
    {
        verts.add(Vec3f(vertices[i].position.x,
                        vertices[i].position.y,
                        vertices[i].position.z));
    }

    std::cout << "Building a new scene\n";
    m_scene = new Scene(triangles.size(), vertices.size(), tris, verts);

    std::cout << "Building BVH with spatial splits\n";
    // create a default platform
    Platform         defaultplatform;
    BVH::BuildParams defaultparams;
    BVH::Stats       stats;
    BVH              myBVH(m_scene, defaultplatform, defaultparams);

    std::cout << "Building CudaBVH\n";
    // create CUDA friendly BVH datastructure
    m_bvh = new CudaBVH(
        myBVH, BVHLayout_Compact);  // Fermi BVH layout = compact. BVH
                                    // layout for Kepler kernel Compact2
    std::cout << "CudaBVH successfully created\n";

    std::cout << "Hi Sam!  How you doin'?" << std::endl;

    Vec4i* cpuNodePtr       = m_bvh->getGpuNodes();
    Vec4i* cpuTriWoopPtr    = m_bvh->getGpuTriWoop();
    S32*   cpuTriIndicesPtr = m_bvh->getGpuTriIndices();

    int nodeSize       = m_bvh->getGpuNodesSize();
    int triWoopSize    = m_bvh->getGpuTriWoopSize();
    int triIndicesSize = m_bvh->getGpuTriIndicesSize();
    int leafnode_count = m_bvh->getLeafnodeCount();
    int triangle_count = m_bvh->getTriCount();
    int triMatIdsSize  = tri_mat_ids.size();

    printf("node size         = %d\n", nodeSize);
    printf("tri woop size     = %d\n", triWoopSize);
    printf("tri indices size  = %d\n", triIndicesSize);
    printf("leaf node count   = %d\n", leafnode_count);
    printf("triangle count    = %d\n", triangle_count);
    printf("tri mat ids count = %d\n", triMatIdsSize);
    printf("tri woop size 2   = %d\n",
           triangle_count * 3 +
               leafnode_count);  // need additional leaf_node_count items to
                                 // mark leaf end

// allocate and copy scene databuffers to the GPU (BVH nodes, triangle
// vertices, triangle indices)
#if 1
    this->cudaNodeBuffer =
        std::vector<float4>(reinterpret_cast<float4*>(cpuNodePtr),
                            reinterpret_cast<float4*>(cpuNodePtr) + nodeSize);
    this->cudaTriWoopBuffer = std::vector<float4>(
        reinterpret_cast<float4*>(cpuTriWoopPtr),
        reinterpret_cast<float4*>(cpuTriWoopPtr) + triWoopSize);
    this->cudaTriIndicesBuffer =
        std::vector<S32>(cpuTriIndicesPtr, cpuTriIndicesPtr + triIndicesSize);
    float4* cudaNodePtr       = this->cudaNodeBuffer.data();
    float4* cudaTriWoopPtr    = this->cudaTriWoopBuffer.data();
    S32*    cudaTriIndicesPtr = this->cudaTriIndicesBuffer.data();
#else
    float4* cudaNodePtr       = reinterpret_cast<float4*>(cpuNodePtr);
    float4* cudaTriWoopPtr    = reinterpret_cast<float4*>(cpuTriWoopPtr);
    S32*    cudaTriIndicesPtr = cpuTriIndicesPtr;
#endif

    // TODO use CompactNode struct and see performance
    this->bvhNodes        = cudaNodePtr;
    this->triWoop         = cudaTriWoopPtr;
    this->triIndices      = cudaTriIndicesPtr;
    this->tri_count       = triangle_count;
    this->leaf_node_count = leafnode_count;
    this->node_size       = nodeSize;
    // this->tri_mesh        = m_mesh->getInterface();
}

//
// both work
//
#if 1
static bool rayAABBOverlap(const Ray&    r,
                           const float3& bounds_min,
                           const float3& bounds_max,
                           float&        tNear)
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tNear = kInfinity;

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

    assert(tmin <= tmax);
    // tNear = tmin;
    tNear = fmaxf(tmin, rayTMin);  // FIXED
    return true;
#endif
}
#else
static bool rayAABBOverlap(const Ray& r,
                           const float3& bounds_min,
                           const float3& bounds_max,
                           float& tNear)
{
#if 0
    constexpr float ooeps = 1e-20f;  // Avoid div by zero, returns 1/2^80,
                                     // an extremely small number
    float idirx = 1.0f / (fabsf(r.direction().x) > ooeps
                              ? r.direction().x
                              : copysignf(ooeps, r.direction().x));
    float idiry = 1.0f / (fabsf(r.direction().y) > ooeps
                              ? r.direction().y
                              : copysignf(ooeps, r.direction().y));
    float idirz = 1.0f / (fabsf(r.direction().z) > ooeps
                              ? r.direction().z
                              : copysignf(ooeps, r.direction().z));
    float oodx  = r.origin().x * idirx;
    float oody  = r.origin().y * idiry;
    float oodz  = r.origin().z * idirz;
#else
    float oodx = r.origin().x * r.directionReciprocal().x;
    float oody = r.origin().y * r.directionReciprocal().y;
    float oodz = r.origin().z * r.directionReciprocal().z;
#endif

    float c0lox = bounds_min.x * r.directionReciprocal().x - oodx;
    float c0hix = bounds_max.x * r.directionReciprocal().x - oodx;
    float c0loy = bounds_min.y * r.directionReciprocal().y - oody;
    float c0hiy = bounds_max.y * r.directionReciprocal().y - oody;
    float c0loz = bounds_min.z * r.directionReciprocal().z - oodz;
    float c0hiz = bounds_max.z * r.directionReciprocal().z - oodz;
    float c0min =
        spanBeginKepler(c0lox, c0hix, c0loy, c0hiy, c0loz, c0hiz, r.tmin());
    float c0max =
        spanEndKepler(c0lox, c0hix, c0loy, c0hiy, c0loz, c0hiz, r.tmax());
    return c0min <= c0max;
}
#endif

static void intersectBVHandTriangles_old(const float4  rayorig,
                                         const float4  raydir,
                                         const float4* gpuNodes,
                                         const float4* triWoop,
                                         const int*    triIndices,
                                         int&          hitTriIdx,
                                         float&        hitdistance,
                                         int&          debugbingo,
                                         float&        hit_u,
                                         float&        hit_v,
                                         int           leafcount,
                                         int           tricount,
                                         bool          anyHit)
{
    constexpr int STACK_SIZE = 64;

    ///////////////////////////////////////////
    //// FERMI / KEPLER KERNEL
    ///////////////////////////////////////////

    // BVH layout Compact2 for Kepler, Ccompact for Fermi (nodeOffsetSizeDiv
    // is different) void CudaBVH::createCompact(const BVH& bvh, int
    // nodeOffsetSizeDiv) createCompact(bvh,16); for Compact2
    // createCompact(bvh,1); for Compact

    int traversalStack[STACK_SIZE];

    // Live state during traversal, stored in registers.

    int   rayidx;  // not used, can be removed
    float tmin;    // t-value from which the ray starts. Usually 0.

#if BLD
    float origx, origy, origz;  // Ray origin.
    float dirx, diry, dirz;     // Ray direction.
    float idirx, idiry, idirz;  // 1 / ray direction
    float oodx, oody, oodz;     // ray origin / ray direction
#endif

    char* stackPtr;  // Current position in traversal stack.
    int   leafAddr;  // If negative, then first postponed leaf, non-negative
                     // if no leaf (innernode).
    int nodeAddr;
    int hitIndex;  // Triangle index of the closest intersection, -1 if
                   // none.
    float hitT;    // t-value of the closest intersection.
                   // Kepler kernel only
    // int     leafAddr2;              // Second postponed leaf,
    // non-negative if none. int     nodeAddr = EntrypointSentinel; //
    // Non-negative: current internal node, negative: second postponed leaf.
    // float hit_u;
    // float hit_v;

    Ray ray = Ray(float3(rayorig), float3(raydir));

    // Initialize (stores local variables in registers)
    {
#if BLD
        origx = rayorig.x;
        origy = rayorig.y;
        origz = rayorig.z;
        dirx  = raydir.x;
        diry  = raydir.y;
        dirz  = raydir.z;

        // ooeps is very small number, used instead of raydir xyz component
        // when that component is near zero 		float ooeps =
        // exp2f(-80.0f);
        // // Avoid div by zero, returns 1/2^80, an extremely small number
        constexpr float ooeps = 1e-20f;  // Avoid div by zero, returns 1/2^80,
                                         // an extremely small number
        idirx =
            1.0f / (fabsf(raydir.x) > ooeps
                        ? raydir.x
                        : copysignf(ooeps, raydir.x));  // inverse ray direction
        idiry =
            1.0f / (fabsf(raydir.y) > ooeps
                        ? raydir.y
                        : copysignf(ooeps, raydir.y));  // inverse ray direction
        idirz =
            1.0f / (fabsf(raydir.z) > ooeps
                        ? raydir.z
                        : copysignf(ooeps, raydir.z));  // inverse ray direction
        oodx = origx * idirx;  // ray origin / ray direction
        oody = origy * idiry;  // ray origin / ray direction
        oodz = origz * idirz;  // ray origin / ray direction
#endif
        tmin = rayorig.w;  // ray min

        // Setup traversal + initialisation

        traversalStack[0] =
            EntrypointSentinel;                // Bottom-most entry. 0x76543210
                                               // (1985229328 in decimal)
        stackPtr = (char*)&traversalStack[0];  // point stackPtr to bottom
                                               // of traversal stack =
                                               // EntryPointSentinel
        leafAddr = 0;                          // No postponed leaf.
        nodeAddr = 0;                          // Start from the root.
        hitIndex = -1;        // No triangle intersected so far.
        hitT     = raydir.w;  // tmax  , ray max
        hit_u    = 0.0f;
        hit_v    = 0.0f;
    }

    // Traversal loop.

    while (nodeAddr != EntrypointSentinel)
    {
        // Traverse internal nodes until all SIMD lanes have found a leaf.

        bool searchingLeaf = true;  // required for warp efficiency
        while (nodeAddr >= 0 && nodeAddr != EntrypointSentinel)
        {
            // Fetch AABBs of the two child nodes.

            // nodeAddr is an offset in number of bytes (char) in gpuNodes
            // array

            float4* ptr  = (float4*)((char*)gpuNodes + nodeAddr);
            float4  n0xy = ptr[0];  // childnode 0, xy-bounds (c0.lo.x,
                                    // c0.hi.x, c0.lo.y, c0.hi.y)
            float4 n1xy = ptr[1];   // childnode 1, xy-bounds (c1.lo.x,
                                    // c1.hi.x, c1.lo.y, c1.hi.y)
            float4 nz = ptr[2];     // childnode 0 and 1, z-bounds (c0.lo.z,
                                    // c0.hi.z, c1.lo.z, c1.hi.z)
            // ptr[3] contains indices to 2 childnodes in case of innernode,
            // see below (childindex = size of array during building, see
            // CudaBVH.cpp)

            // compute ray intersections with BVH node bounding box

            /// RAY BOX INTERSECTION
            // Intersect the ray against the child nodes.

            ray.setTMin(rayorig.w);
            ray.setTMax(hitT);  // huge improvement

#if 0
            float c0lox = n0xy.x * ray.directionReciprocal().x - oodx;
            float c0hix = n0xy.y * ray.directionReciprocal().x - oodx;
            float c0loy = n0xy.z * ray.directionReciprocal().y - oody;
            float c0hiy = n0xy.w * ray.directionReciprocal().y - oody;
            float c0loz = nz.x * ray.directionReciprocal().z - oodz;
            float c0hiz = nz.y * ray.directionReciprocal().z - oodz;
            float c0min = spanBeginKepler(
                c0lox, c0hix, c0loy, c0hiy, c0loz, c0hiz, ray.tmin());
            float c0max = spanEndKepler(
                c0lox, c0hix, c0loy, c0hiy, c0loz, c0hiz, ray.tmax());

            float c1lox = n1xy.x * ray.directionReciprocal().x - oodx;
            float c1hix = n1xy.y * ray.directionReciprocal().x - oodx;
            float c1loy = n1xy.z * ray.directionReciprocal().y - oody;
            float c1hiy = n1xy.w * ray.directionReciprocal().y - oody;
            float c1loz = nz.z * ray.directionReciprocal().z - oodz;
            float c1hiz = nz.w * ray.directionReciprocal().z - oodz;
            float c1min = spanBeginKepler(
                c1lox, c1hix, c1loy, c1hiy, c1loz, c1hiz, ray.tmin());
            float c1max = spanEndKepler(
                c1lox, c1hix, c1loy, c1hiy, c1loz, c1hiz, ray.tmax());

            // ray box intersection boundary tests:

            bool traverseChild0 = (c0min <= c0max);
            bool traverseChild1 = (c1min <= c1max);
#else
            float c0min, c1min;
            bool hit0 = rayAABBOverlap(ray,
                                       float3(n0xy.x, n0xy.z, nz.x),
                                       float3(n0xy.y, n0xy.w, nz.y),
                                       c0min);
            bool hit1 = rayAABBOverlap(ray,
                                       float3(n1xy.x, n1xy.z, nz.z),
                                       float3(n1xy.y, n1xy.w, nz.w),
                                       c1min);

            bool traverseChild0 = (hit0);
            bool traverseChild1 = (hit1);
#endif

            //
            // both work
            //
#if 1
            // Neither child was intersected => pop stack.
            if (!traverseChild0 && !traverseChild1)
            {
                nodeAddr =
                    *(int*)stackPtr;  // fetch next node by popping the stack
                stackPtr -= 4;        // popping decrements stackPtr by 4 bytes
                                      // (because stackPtr is a pointer to char)
            }

            // Otherwise, one or both children intersected => fetch child
            // pointers.

            else
            {
                int2 cnodes = *(int2*)&ptr[3];
                // set nodeAddr equal to intersected childnode index (or
                // first childnode when both children are intersected)
                nodeAddr = (traverseChild0) ? cnodes.x : cnodes.y;

                // Both children were intersected => push the farther one on
                // the stack.

                if (traverseChild0 &&
                    traverseChild1)  // store closest child in nodeAddr,
                                     // swap if necessary
                {
#if 1
                    // TODO benchmark performance improvement (and on GPU too)
                    if (c1min < c0min) swap2(nodeAddr, cnodes.y);
#endif

                    stackPtr += 4;  // pushing increments stack by 4 bytes
                                    // (stackPtr is a pointer to char)
                    *(int*)stackPtr =
                        cnodes.y;  // push furthest node on the stack
                }
            }
#else
            int2 cnodes = *(int2*)&ptr[3];
            if (traverseChild0)
            {
                stackPtr += 4;  // pushing increments stack by 4 bytes
                                // (stackPtr is a pointer to char)
                *(int*)stackPtr = cnodes.x;  // push furthest node on the stack
            }
            if (traverseChild1)
            {
                stackPtr += 4;  // pushing increments stack by 4 bytes
                                // (stackPtr is a pointer to char)
                *(int*)stackPtr = cnodes.y;  // push furthest node on the stack
            }
            nodeAddr = *(int*)stackPtr;  // fetch next node by popping the stack
            stackPtr -= 4;  // popping decrements stackPtr by 4 bytes
#endif

            // First leaf => postpone and continue traversal.
            // leafnodes have a negative index to distinguish them from
            // inner nodes if nodeAddr less than 0 -> nodeAddr is a leaf
            if (nodeAddr < 0 && leafAddr >= 0)
            {
                searchingLeaf = false;  // required for warp efficiency
                leafAddr      = nodeAddr;
                nodeAddr      = *(int*)stackPtr;  // pops next node from stack
                stackPtr -= 4;  // decrements stackptr by 4 bytes (because
                                // stackPtr is a pointer to char)
                break;
            }
        }

        ///////////////////////////////////////////
        /// TRIANGLE INTERSECTION
        //////////////////////////////////////

        // Process postponed leaf nodes.

        while (leafAddr <
               0)  /// if leafAddr is negative, it points to an actual
                   /// leafnode (when positive or 0 it's an innernode)
        {
            // Intersect the ray against each triangle using Sven Woop's
            // algorithm. Woop ray triangle intersection: Woop triangles are
            // unit triangles. Each ray must be transformed to "unit
            // triangle space", before testing for intersection

            for (int triAddr = ~leafAddr;;
                 triAddr += 3)  // triAddr is index in triWoop array (and
                                // bitwise complement of leafAddr)
            {  // no defined upper limit for loop, continues until leaf
               // terminator code 0x80000000 is encountered

                if (triAddr == debugbingo) continue;  // avoid self intersection

                // Read first 16 bytes of the triangle.
                // fetch first precomputed triangle edge
                float4 v00 = triWoop[triAddr];

                // End marker 0x80000000 (negative zero) => all triangles in
                // leaf processed --> terminate
                if (float_as_int(v00.x) == 0x80000000) break;

                    // Compute and check intersection t-value (hit distance
                    // along ray).
#if BLD
                float Oz = v00.w - origx * v00.x - origy * v00.y -
                           origz * v00.z;  // Origin z
                float invDz = 1.0f / (dirx * v00.x + diry * v00.y +
                                      dirz * v00.z);  // inverse Direction z
#else
                float Oz = v00.w - ray.origin().x * v00.x -
                           ray.origin().y * v00.y -
                           ray.origin().z * v00.z;  // Origin z
                float invDz =
                    1.0f /
                    (ray.direction().x * v00.x + ray.direction().y * v00.y +
                     ray.direction().z * v00.z);  // inverse Direction z
#endif

                float t = Oz * invDz;

                if (t > tmin && t < hitT)
                {
                    // Compute and check barycentric u.

                    // fetch second precomputed triangle edge
                    float4 v11 = triWoop[triAddr + 1];
#if BLD
                    float Ox = v11.w + origx * v11.x + origy * v11.y +
                               origz * v11.z;  // Origin.x
                    float Dx = dirx * v11.x + diry * v11.y +
                               dirz * v11.z;  // Direction.x
#else
                    float Ox = v11.w + ray.origin().x * v11.x +
                               ray.origin().y * v11.y +
                               ray.origin().z * v11.z;  // Origin.x
                    float Dx = ray.direction().x * v11.x +
                               ray.direction().y * v11.y +
                               ray.direction().z * v11.z;  // Direction.x
#endif
                    float u = Ox + t * Dx;  /// parametric equation of a ray
                                            /// (intersection point)

                    if (u >= 0.0f && u <= 1.0f)
                    {
                        // Compute and check barycentric v.

                        // fetch third precomputed triangle edge
                        float4 v22 = triWoop[triAddr + 2];
#if BLD
                        float Oy = v22.w + origx * v22.x + origy * v22.y +
                                   origz * v22.z;
                        float Dy = dirx * v22.x + diry * v22.y + dirz * v22.z;
#else
                        float Oy = v22.w + ray.origin().x * v22.x +
                                   ray.origin().y * v22.y +
                                   ray.origin().z * v22.z;
                        float Dy = ray.direction().x * v22.x +
                                   ray.direction().y * v22.y +
                                   ray.direction().z * v22.z;
#endif
                        float v = Oy + t * Dy;

                        if (v >= 0.0f && u + v <= 1.0f)
                        {
                            // We've got a hit!
                            // Record intersection.

                            hitT     = t;
                            hitIndex = triAddr;  // store triangle index for
                                                 // shading
                            hit_u = u;
                            hit_v = v;

                            // Closest intersection not required =>
                            // terminate.
                            if (anyHit)  // only true for shadow rays
                            {
                                nodeAddr = EntrypointSentinel;
                                break;
                            }
                        }
                    }
                }
            }  // end triangle intersection

            // Another leaf was postponed => process it as well.

            leafAddr = nodeAddr;
            if (nodeAddr < 0)  // nodeAddr is an actual leaf when < 0
            {
                nodeAddr = *(int*)stackPtr;  // pop stack
                stackPtr -= 4;  // decrement with 4 bytes to get the next
                                // int (stackPtr is char*)
            }
        }  // end leaf/triangle intersection loop
    }      // end traversal loop (AABB and triangle intersection)

    // Remap intersected triangle index, and store the result.

    debugbingo = hitIndex;  // hit tri addr or -1

    if (hitIndex != -1)
    {
        // float4 a    = tex1Dfetch<float4>(triNormalsTexture, hitIndex);
        // float4 b    = tex1Dfetch<float4>(triNormalsTexture, hitIndex +
        // 1); float4 c    = tex1Dfetch<float4>(triNormalsTexture, hitIndex
        // + 2); shadenormal = normalize(Vector3(a.x, a.y, a.z) * hit_u +
        // Vector3(b.x, b.y, b.z) * hit_v +
        //                        Vector3(c.x, c.y, c.z) * (1 - hit_u -
        //                        hit_v));

        hitIndex = triIndices[hitIndex];
        // remapping tri indices delayed until this point for performance
        // reasons (slow texture memory lookup in de triIndicesTexture)
        // because multiple triangles per node can potentially be hit
    }

    hitTriIdx   = hitIndex;
    hitdistance = hitT;
}

struct NvsbvhCompactNode
{
    float4 c0xy;
    float4 c1xy;
    float4 c01z;
    int4   cnodes;
};

static void intersectBVHandTriangles(const float4  rayorig,
                                     const float4  raydir,
                                     const float4* gpuNodes,
                                     const float4* triWoop,
                                     const int*    triIndices,
                                     int&          hitTriIdx,
                                     float&        hitdistance,
                                     int&          debugbingo,
                                     float&        hit_u,
                                     float&        hit_v,
                                     int           leafcount,
                                     int           tricount,
                                     bool          anyHit)
{
    constexpr int            STACK_SIZE = 64;
    const NvsbvhCompactNode* compact_bvh =
        reinterpret_cast<const NvsbvhCompactNode*>(gpuNodes);

    float tmin;
    float hitT;
    int   traversalStack[STACK_SIZE];
    int   ptr = 0;
    int   nodeAddr;
    int   hitIndex;
    int   leafAddr;  // If negative, then first postponed leaf, non-negative
                     // if no leaf (innernode).

    Ray ray = Ray(float3(rayorig), float3(raydir));

    // Initialize (stores local variables in registers)
    {
        tmin                  = rayorig.w;  // ray min
        hitT                  = raydir.w;   // tmax  , ray max
        traversalStack[ptr++] = EntrypointSentinel;
        leafAddr              = 0;   // No postponed leaf.
        nodeAddr              = 0;   // Start from the root.
        hitIndex              = -1;  // No triangle intersected so far.
        hit_u                 = 0.0f;
        hit_v                 = 0.0f;
    }

    // Traversal loop.
    while (nodeAddr != EntrypointSentinel)
    {
        while (nodeAddr >= 0 && nodeAddr != EntrypointSentinel)
        {
            auto& node = compact_bvh[nodeAddr / 64];

            ray.setTMin(rayorig.w);
            ray.setTMax(hitT);  // huge improvement

            float c0min, c1min;
            bool  hit0 =
                rayAABBOverlap(ray,
                               float3(node.c0xy.x, node.c0xy.z, node.c01z.x),
                               float3(node.c0xy.y, node.c0xy.w, node.c01z.y),
                               c0min);
            bool hit1 =
                rayAABBOverlap(ray,
                               float3(node.c1xy.x, node.c1xy.z, node.c01z.z),
                               float3(node.c1xy.y, node.c1xy.w, node.c01z.w),
                               c1min);

            bool traverseChild0 = (hit0);
            bool traverseChild1 = (hit1);

            //
            // both work
            //
            // Neither child was intersected => pop stack.
            if (!traverseChild0 && !traverseChild1)
            {
                nodeAddr = traversalStack[--ptr];
            }

            // Otherwise, one or both children intersected => fetch child
            // pointers.
            else
            {
                int4 cnodes = node.cnodes;
                // set nodeAddr equal to intersected childnode index (or
                // first childnode when both children are intersected)
                nodeAddr = (traverseChild0) ? cnodes.x : cnodes.y;

                if (traverseChild0 && traverseChild1)
                {
                    // TODO benchmark performance improvement (and on GPU too)
                    if (c1min < c0min) swap2(nodeAddr, cnodes.y);

                    traversalStack[ptr++] = cnodes.y;
                }
            }

            // First leaf => postpone and continue traversal.
            // leafnodes have a negative index to distinguish them from
            // inner nodes if nodeAddr less than 0 -> nodeAddr is a leaf
            if (nodeAddr < 0 && leafAddr >= 0)
            {
                leafAddr = nodeAddr;
                nodeAddr = traversalStack[--ptr];
                break;
            }
        }

        ///////////////////////////////////////////
        /// TRIANGLE INTERSECTION
        //////////////////////////////////////

        // Process postponed leaf nodes.

        while (leafAddr <
               0)  /// if leafAddr is negative, it points to an actual
                   /// leafnode (when positive or 0 it's an innernode)
        {
            // Intersect the ray against each triangle using Sven Woop's
            // algorithm. Woop ray triangle intersection: Woop triangles are
            // unit triangles. Each ray must be transformed to "unit
            // triangle space", before testing for intersection

            for (int triAddr = ~leafAddr;;
                 triAddr += 3)  // triAddr is index in triWoop array (and
                                // bitwise complement of leafAddr)
            {  // no defined upper limit for loop, continues until leaf
               // terminator code 0x80000000 is encountered

                if (triAddr == debugbingo) continue;  // avoid self intersection

                // Read first 16 bytes of the triangle.
                // fetch first precomputed triangle edge
                float4 v00 = triWoop[triAddr];

                // End marker 0x80000000 (negative zero) => all triangles in
                // leaf processed --> terminate
                if (float_as_int(v00.x) == 0x80000000) break;

                // Compute and check intersection t-value (hit distance
                // along ray).
                float Oz = v00.w - ray.origin().x * v00.x -
                           ray.origin().y * v00.y -
                           ray.origin().z * v00.z;  // Origin z
                float invDz =
                    1.0f /
                    (ray.direction().x * v00.x + ray.direction().y * v00.y +
                     ray.direction().z * v00.z);  // inverse Direction z

                float t = Oz * invDz;

                if (t > tmin && t < hitT)
                {
                    // Compute and check barycentric u.

                    // fetch second precomputed triangle edge
                    float4 v11 = triWoop[triAddr + 1];
                    float  Ox  = v11.w + ray.origin().x * v11.x +
                               ray.origin().y * v11.y +
                               ray.origin().z * v11.z;  // Origin.x
                    float Dx = ray.direction().x * v11.x +
                               ray.direction().y * v11.y +
                               ray.direction().z * v11.z;  // Direction.x
                    float u = Ox + t * Dx;  /// parametric equation of a ray
                                            /// (intersection point)

                    if (u >= 0.0f && u <= 1.0f)
                    {
                        // Compute and check barycentric v.

                        // fetch third precomputed triangle edge
                        float4 v22 = triWoop[triAddr + 2];
                        float  Oy  = v22.w + ray.origin().x * v22.x +
                                   ray.origin().y * v22.y +
                                   ray.origin().z * v22.z;
                        float Dy = ray.direction().x * v22.x +
                                   ray.direction().y * v22.y +
                                   ray.direction().z * v22.z;
                        float v = Oy + t * Dy;

                        if (v >= 0.0f && u + v <= 1.0f)
                        {
                            // We've got a hit!
                            // Record intersection.

                            hitT     = t;
                            hitIndex = triAddr;  // store triangle index for
                                                 // shading
                            hit_u = u;
                            hit_v = v;

                            // Closest intersection not required =>
                            // terminate.
                            if (anyHit)  // only true for shadow rays
                            {
                                nodeAddr = EntrypointSentinel;
                                break;
                            }
                        }
                    }
                }
            }  // end triangle intersection

            // Another leaf was postponed => process it as well.
            leafAddr = nodeAddr;
            if (nodeAddr < 0)  // nodeAddr is an actual leaf when < 0
            {
                nodeAddr = traversalStack[--ptr];
            }
        }
    }

    if (hitIndex != -1)
    {
        debugbingo = hitIndex;
        hitIndex   = triIndices[hitIndex];
    }

    hitTriIdx   = hitIndex;
    hitdistance = hitT;
}

NvSplitBVH::~NvSplitBVH()
{
    delete m_bvh;
    delete m_scene;
}

bool NvSplitBVH::intersect(const Ray& ray, float& tNear) const
{
    const float3& rayorig = ray.origin();
    const float3& raydir  = ray.direction();

    int   bestTriIdx  = -1;
    int   geomtype    = -1;
    float hitDistance = kInfinity;
    float hit_u;
    float hit_v;
    int   debugbingo = -1;
    intersectBVHandTriangles(
        float4(rayorig.x, rayorig.y, rayorig.z, ray.tmin()),
        float4(raydir.x, raydir.y, raydir.z, ray.tmax()),
        bvhNodes,
        triWoop,
        triIndices,
        bestTriIdx,
        hitDistance,
        debugbingo,
        hit_u,
        hit_v,
        leaf_node_count,
        tri_count,
        false);

    if (hitDistance < ray.tmax() && hitDistance > ray.tmin())
    {
        tNear = hitDistance;
        return true;
    }
    else
    {
        tNear = kInfinity;
        return false;
    }
}

}  // namespace nvsbvh
