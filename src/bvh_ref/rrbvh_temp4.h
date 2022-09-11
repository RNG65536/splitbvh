#if 1
#define check(x) void(0)
#define check2(x) void(0)
#else
#define check(x) x
#define check2(x) x
#endif

#define BLAS_MARKER -2
#define TLAS_MARKER -1

bool RRBVH::intersect4(const Ray& ray, float& tNear) const
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
    // float t = kInfinity;
    float t = ray.tmax();

    int stack[64];
    int ptr      = 0;
    stack[ptr++] = TLAS_MARKER;

    int index = topBVHIndex;
    // int   index    = 0; // works if there is only one instance

    int  currMatID = 0;
    bool BLAS      = false;

    int3   triID = int3(-1, -1, -1);
    mat44  transMat;
    float3 bary;
    float4 vert0, vert1, vert2;

    Ray rTrans = ray;

    while (index != TLAS_MARKER)
    {
        auto& node = this->compact_bvh[index];

        int leaf       = node.cnodes.w;
        int leftIndex  = node.cnodes.x;
        int rightIndex = node.cnodes.y;

        if (leaf == 0)  // not leaf
        {
            check(printf("----- index = %d, leaf = %d (%s branch)\n",
                         index,
                         leaf,
                         BLAS ? "BLAS" : "TLAS"));
            // check(printf(BLAS ? "  BLAS node\n" : "  TLAS node\n"));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            float c0min, c1min;
            bool  hit0 = AABBIntersect(get_c0_min(node),
                                      get_c0_max(node),
                                      rTrans,
                                      ray.tmin(),
                                      t,
                                      c0min);
            bool  hit1 = AABBIntersect(get_c1_min(node),
                                      get_c1_max(node),
                                      rTrans,
                                      ray.tmin(),
                                      t,
                                      c1min);

            if (!hit0 && !hit1)
            {
                index = stack[--ptr];
                check(printf(
                    "  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));
            }
            else
            {
                index = hit0 ? leftIndex : rightIndex;

                if (hit0 && hit1)
                {
                    check(printf("  hit both\n"));
                    if (c1min < c0min)
                    {
                        std::swap(index, rightIndex);
                    }

                    stack[ptr++] = rightIndex;
                }

                if (index >= 0)
                {
                    continue;
                }
            }
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

            stack[ptr++] = BLAS_MARKER;
            index        = leftIndex;
            BLAS         = true;
            currMatID    = rightIndex;

            continue;
        }
        else  // leaf > 0 // bottom level leaf
        {
            throw;
        }

        if (BLAS)
        {
            while (index < 0 && index != BLAS_MARKER)
            {
                int tmp    = ~index;
                int offset = tmp & 0xFFFFF;
                int count  = tmp >> 20;
                check(printf("  intersect leaf (%d, %d)\n", offset, count));
                check(printf("  ray: o(%f, %f, %f), d(%f, %f, %f)\n",
                             rTrans.origin().x,
                             rTrans.origin().y,
                             rTrans.origin().z,
                             rTrans.direction().x,
                             rTrans.direction().y,
                             rTrans.direction().z));

                for (int i = 0; i < count; i++)
                {
#if 0
                int3 vertIndices = vertex_indices_tex[offset + i];
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
                    t     = uvt.z;
                    //triID = vertIndices;
                    bary  = float3(uvt.w, uvt.x, uvt.y);
                    vert0 = v0;
                    vert1 = v1;
                    vert2 = v2;
                }
#else
                    float4 v00 = this->tri_woop_buffer[(offset + i) * 3 + 0];

                    float Oz = v00.w - rTrans.origin().x * v00.x -
                               rTrans.origin().y * v00.y -
                               rTrans.origin().z * v00.z;
                    float invDz = 1.0f / (rTrans.direction().x * v00.x +
                                          rTrans.direction().y * v00.y +
                                          rTrans.direction().z * v00.z);
                    float d     = Oz * invDz;

                    if (d > ray.tmin() && d < t)
                    {
                        float4 v11 =
                            this->tri_woop_buffer[(offset + i) * 3 + 1];
                        float Ox = v11.w + rTrans.origin().x * v11.x +
                                   rTrans.origin().y * v11.y +
                                   rTrans.origin().z * v11.z;
                        float Dx = rTrans.direction().x * v11.x +
                                   rTrans.direction().y * v11.y +
                                   rTrans.direction().z * v11.z;
                        float u = Ox + d * Dx;

                        if (u >= 0.0f && u <= 1.0f)
                        {
                            float4 v22 =
                                this->tri_woop_buffer[(offset + i) * 3 + 2];
                            float Oy = v22.w + rTrans.origin().x * v22.x +
                                       rTrans.origin().y * v22.y +
                                       rTrans.origin().z * v22.z;
                            float Dy = rTrans.direction().x * v22.x +
                                       rTrans.direction().y * v22.y +
                                       rTrans.direction().z * v22.z;
                            float v = Oy + d * Dy;

                            if (v >= 0.0f && u + v <= 1.0f)
                            {
                                t    = d;
                                bary = float3(u, v, 1.0f - u - v);
                            }
                        }
                    }
#endif
                }

                index = stack[--ptr];
            }

            if (index == BLAS_MARKER)
            {
                BLAS = false;

                index = stack[--ptr];
                check(printf(
                    "  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));

                // rTrans = ray;
                rTrans.setOrigin(ray.origin());
                rTrans.setDirection(ray.direction());
            }
        }
    }

    tNear = t;

    return tNear < kInfinity;
}
