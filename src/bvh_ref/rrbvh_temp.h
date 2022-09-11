#if 1
#define check(x) void(0)
#define check2(x) void(0)
#else
#define check(x) x
#define check2(x) x
#endif

// not working
bool RRBVH::intersect1(const Ray& ray, float& tNear) const
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

    int stack2[2];
    int ptr2 = 0;

    int index = topBVHIndex;
    // int index = 0;  // works if there is only one instance

    int  currMatID = 0;
    bool BLAS      = false;

    int3   triID = int3(-1, -1, -1);
    mat44  transMat;
    float3 bary;
    float4 vert0, vert1, vert2;

    Ray rTrans = ray;

    while (index != -1)
    {
        auto& node = this->compact_bvh[index];

        int leaf       = node.cnodes.w;
        int leftIndex  = node.cnodes.x;
        int rightIndex = node.cnodes.y;

        if (leaf == 0)  // not leaf
        {
            check2(printf("----- index = %d, leaf = %d (%s branch)\n",
                          index,
                          leaf,
                          BLAS ? "BLAS" : "TLAS"));
            // check(printf(BLAS ? "  BLAS node\n" : "  TLAS node\n"));
            check(printf("  left index  = %d\n", leftIndex));
            check(printf("  right index = %d\n", rightIndex));

            if (leftIndex == -1) throw;
            if (rightIndex == -1) throw;

            rTrans.setTMax(t);  // huge improvement

            float c0min, c1min;
            bool  hit0 = AABBIntersect(get_c0_min(node),
                                      get_c0_max(node),
                                      rTrans,
                                      ray.tmin(),
                                      fminf(ray.tmax(), t),
                                      c0min);
            bool  hit1 = AABBIntersect(get_c1_min(node),
                                      get_c1_max(node),
                                      rTrans,
                                      ray.tmin(),
                                      fminf(ray.tmax(), t),
                                      c1min);

#if 1

            if (BLAS)
            {
                if (!hit0 && !hit1)
                {
                    check(printf("  hit none\n"));
                    index = stack[--ptr];
                    check(printf("  pop %s index = %d\n",
                                 BLAS ? "BLAS" : "TLAS",
                                 index));
                }

                else
                {
                    int firstIndex  = hit0 ? leftIndex : rightIndex;
                    int secondIndex = hit0 ? rightIndex : leftIndex;

                    if (hit0 && hit1)
                    {
                        if (c1min < c0min)
                        {
                            std::swap(firstIndex, secondIndex);
                        }

                        if (secondIndex < 0)
                        {
                            stack2[ptr2++] = secondIndex;
                        }
                        else
                        {
                            stack[ptr++] = secondIndex;
                        }
                    }

                    if (firstIndex < 0)
                    {
                        stack2[ptr2++] = firstIndex;
                        index          = stack[--ptr];
                    }
                    else
                    {
                        index = firstIndex;
                    }

                    //if (BLAS && index == -1)
                    //{
                    //    BLAS = false;

                    //    index = stack[--ptr];
                    //    check(printf("  pop %s index = %d\n",
                    //                 BLAS ? "BLAS" : "TLAS",
                    //                 index));
                    //    rTrans = ray;
                    //}
                }

                if (BLAS && index == -1)
                {
                    BLAS = false;

                    index = stack[--ptr];
                    check(printf("  pop %s index = %d\n",
                                 BLAS ? "BLAS" : "TLAS",
                                 index));
                    rTrans = ray;
                }
            }
            else
            {
                // TODO : TLAS branch when there are multiple instances
                // throw;

                bool hit_any = false;

                if (hit0)
                {
                    hit_any = true;

                    check(printf("  hit left\n"));
                    if (node.cnodes.x < 0)
                    {
                        throw;
                    }
                    else
                    {
                        check(printf("  left is branch\n"));
                        stack[ptr++] = leftIndex;
                    }
                }

                if (hit1)
                {
                    hit_any = true;

                    check(printf("  hit right\n"));
                    if (node.cnodes.y < 0)
                    {
                        throw;
                    }
                    else
                    {
                        check(printf("  right is branch\n"));
                        stack[ptr++] = rightIndex;
                    }
                }

                index = stack[--ptr];
                check(printf(
                    "  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));

//                if (!hit_any)
//                {
//                    if (BLAS && index == -1)
//                    {
//                        BLAS = false;
//
//                        index = stack[--ptr];
//                        check(printf("  pop %s index = %d\n",
//                                     BLAS ? "BLAS" : "TLAS",
//                                     index));
//#if 1
//                        rTrans.setOrigin(ray.origin());
//                        rTrans.setDirection(ray.direction());
//#else
//                        rTrans = ray;
//#endif
//                    }
//                }
            }

#else
            bool hit_any = false;

            if (BLAS)
            {
                if (leftHit > 0.0f)
                {
                    hit_any = true;

                    check(printf("  hit left\n"));
                    if (node.cnodes.x < 0)
                    {
                        stack2[ptr2++] = node.cnodes.x;
                    }
                    else
                    {
                        check(printf("  left is branch\n"));
                        stack[ptr++] = leftIndex;
                    }
                }

                if (rightHit > 0.0f)
                {
                    hit_any = true;

                    check(printf("  hit right\n"));
                    if (node.cnodes.y < 0)
                    {
                        stack2[ptr2++] = node.cnodes.y;
                    }
                    else
                    {
                        check(printf("  right is branch\n"));
                        stack[ptr++] = rightIndex;
                    }
                }
            }
            else
            {
                // TODO : TLAS branch when there are multiple instances
                throw;

                if (leftHit > 0.0f)
                {
                    hit_any = true;

                    check(printf("  hit left\n"));
                    if (node.cnodes.x < 0)
                    {
                    }
                    else
                    {
                        check(printf("  left is branch\n"));
                        stack[ptr++] = leftIndex;
                    }
                }

                if (rightHit > 0.0f)
                {
                    hit_any = true;

                    check(printf("  hit right\n"));
                    if (node.cnodes.y < 0)
                    {
                    }
                    else
                    {
                        check(printf("  right is branch\n"));
                        stack[ptr++] = rightIndex;
                    }
                }
            }

            index = stack[--ptr];
            check(
                printf("  pop %s index = %d\n", BLAS ? "BLAS" : "TLAS", index));

            if (!hit_any)
            {
                if (BLAS && index == -1)
                {
                    BLAS = false;

                    index = stack[--ptr];
                    check(printf("  pop %s index = %d\n",
                                 BLAS ? "BLAS" : "TLAS",
                                 index));
                    rTrans = ray;
                }
            }
#endif
        }
        else if (leaf < 0)  // top level leaf
        {
            check2(printf(
                "----- index = %d, leaf = %d (TLAS leaf)\n", index, leaf));
            check2(printf("  left index  = %d\n", leftIndex));
            check2(printf("  right index = %d\n", rightIndex));

            // rTrans = ray;

            float4 c0 = transforms_tex[(-leaf - 1) * 4 + 0];
            float4 c1 = transforms_tex[(-leaf - 1) * 4 + 1];
            float4 c2 = transforms_tex[(-leaf - 1) * 4 + 2];
            float4 c3 = transforms_tex[(-leaf - 1) * 4 + 3];

            transMat = Mat4(c0, c1, c2, c3);
            rTrans.setOrigin(float3(transMat * float4(ray.origin(), 1.0f)));
            rTrans.setDirection(
                float3(transMat * float4(ray.direction(), 0.0f)));
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
        }
        else
        {
            throw;
        }

        while (ptr2 > 0)
        {
            int tmp    = ~stack2[--ptr2];
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
                    triID = vertIndices;
                    bary  = float3(uvt.w, uvt.x, uvt.y);
                    vert0 = v0;
                    vert1 = v1;
                    vert2 = v2;
                }
            }
        }
    }

    tNear = t;

    return tNear < kInfinity;
}
