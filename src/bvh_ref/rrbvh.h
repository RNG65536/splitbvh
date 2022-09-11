//#include <cuda_runtime.h>
//#include <helper_cuda.h>
//
//#include "Material.h"
//#include "Scene.h"
//#include "bvh/triangle.h"
//#include "core/constants.h"
//#include "core/vector.h"

#include "LinearMath.h"
#include "Ray.h"
#include "rrbvh/rr_scene.h"

namespace rrbvh
{
// struct mat4
//{
//    float m[4][4];
//
//    mat4() {}
//    mat4(const float4& c0, const float4& c1, const float4& c2, const float4&
//    c3)
//    {
//        m[0][0] = c0.x;
//        m[0][1] = c0.y;
//        m[0][2] = c0.z;
//        m[0][3] = c0.w;
//        m[1][0] = c1.x;
//        m[1][1] = c1.y;
//        m[1][2] = c1.z;
//        m[1][3] = c1.w;
//        m[2][0] = c2.x;
//        m[2][1] = c2.y;
//        m[2][2] = c2.z;
//        m[2][3] = c2.w;
//        m[3][0] = c3.x;
//        m[3][1] = c3.y;
//        m[3][2] = c3.z;
//        m[3][3] = c3.w;
//    }
//
//    float4 col(int idx) const { return make_float4(m[idx][0], m[idx][1],
//    m[idx][2], m[idx][3]); }
//
//    mat4 operator*(float a) const
//    {
//        float4 c0 = make_float4(m[0][0], m[0][1], m[0][2], m[0][3]);
//        float4 c1 = make_float4(m[1][0], m[1][1], m[1][2], m[1][3]);
//        float4 c2 = make_float4(m[2][0], m[2][1], m[2][2], m[2][3]);
//        float4 c3 = make_float4(m[3][0], m[3][1], m[3][2], m[3][3]);
//        return mat4(c0, c1, c2, c3);
//    }
//
//    float4 operator*(const float4& v) const
//    {
//        float4 r;
//        r.x = m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z + m[3][0] * v.w;
//        r.y = m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z + m[3][1] * v.w;
//        r.z = m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z + m[3][2] * v.w;
//        r.w = m[0][3] * v.x + m[1][3] * v.y + m[2][3] * v.z + m[3][3] * v.w;
//        return r;
//    }
//};

class RRBVH : public Geometry
{
public:
    RRBVH(const MeshGroup& mesh);

    ~RRBVH();

    void convert_to_compact_();
    void convert_to_compact();
    void woopify_tri();

    bool intersect0(const Ray& ray, float& tNear) const;
    bool intersect1(const Ray& ray, float& tNear) const;
    bool intersect2(const Ray& ray, float& tNear) const;
    bool intersect3(const Ray& ray, float& tNear) const;
    bool intersect4(const Ray& ray, float& tNear) const;
    bool intersect5(const Ray& ray, float& tNear) const;
    bool intersect(const Ray& ray, float& tNear) const override;

    std::vector<BvhTranslator::Node> bvh_buffer;
    std::vector<Indices>             vertex_indices_buffer;
    std::vector<Vec4>                vertices_buffer;
    std::vector<Vec4>                normals_buffer;
    std::vector<Mat4>                transforms_buffer;
    std::vector<float4>              tri_woop_buffer;

    const float3* bvh_tex            = 0;  // f32 rgb
    const int3*   vertex_indices_tex = 0;  // i32 rgb
    const float4* vertices_tex       = 0;  // f32 rgba
    const float4* normals_tex        = 0;  // f32 rgba
    const float4* transforms_tex     = 0;  // f32 rgba
    const float4* tri_woop_tex       = 0;
    int           topBVHIndex;
    int           num_nodes;

    struct CompactNode
    {
        float4 c0xy;
        float4 c1xy;
        float4 c01z;
        int4   cnodes;
    };

    std::vector<CompactNode> compact_bvh;
    std::vector<CompactNode> compact_bvh_top;
};
}  // namespace rrbvh