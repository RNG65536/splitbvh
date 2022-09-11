bool RRBVH::intersect2(const Ray& ray, float& tNear) const
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

    int stack2[10];
    int ptr2 = 0;

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

            // leftHit = AABBIntersect(
            //    bvh_tex[leftIndex * 3 + 0], bvh_tex[leftIndex * 3 + 1],
            //    rTrans);
            // rightHit = AABBIntersect(bvh_tex[rightIndex * 3 + 0],
            //                         bvh_tex[rightIndex * 3 + 1],
            //                         rTrans);
            leftHit  = AABBIntersect(get_c0_min(node),
                                    get_c0_max(node),
                                    rTrans,
                                    ray.tmin(),
                                    fminf(ray.tmax(), t));
            rightHit = AABBIntersect(get_c1_min(node),
                                     get_c1_max(node),
                                     rTrans,
                                     ray.tmin(),
                                     fminf(ray.tmax(), t));

            if (BLAS)
            {
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

                    if (deferred < 0)
                    {
                        stack2[ptr2++] = deferred;
                    }
                    else
                    {
                        stack[ptr++] = deferred;
                    }

                    if (index >= 0)
                    {
                        continue;
                    }
                    else
                    {
                        stack2[ptr2++] = index;
                    }
                }
                else if (leftHit > 0.0f)
                {
                    check(printf("  hit left\n"));
                    index = leftIndex;

                    if (index >= 0)
                    {
                        continue;
                    }
                    else
                    {
                        stack2[ptr2++] = index;
                    }
                }
                else if (rightHit > 0.0f)
                {
                    check(printf("  hit right\n"));
                    index = rightIndex;

                    if (index >= 0)
                    {
                        continue;
                    }
                    else
                    {
                        stack2[ptr2++] = index;
                    }
                }
                else
                {
                    check(printf("  hit none\n"));
                }
            }
            else  // TLAS
            {
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

            stack[ptr++] = -1;
            index        = leftIndex;
            BLAS         = true;
            currMatID    = rightIndex;

            continue;
        }
        else  // leaf > 0 // bottom level leaf
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
