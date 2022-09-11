#pragma once

#include "Geometry.h"
#include "LinearMath.h"
#include "Mesh.h"
#include "nvsbvh/CudaRenderKernel.h"

namespace nvsbvh
{
struct TriangleVertex
{
    float3 position;
};

struct TriangleFace
{
    uint32_t ia, ib, ic, mat_id;
};

// in order to be passed to the kernel
class TriangleMeshInterface
{
private:
    const TriangleFace*   faces = nullptr;
    const TriangleVertex* verts = nullptr;
    const uint32_t n_faces = 0;  // todo : remove field as it's not required
    const uint32_t n_verts = 0;

public:
    TriangleMeshInterface() = delete;
    TriangleMeshInterface(const TriangleFace*   faces_,
                          const TriangleVertex* verts_,
                          uint32_t              n_faces_,
                          uint32_t              n_verts_)
        : faces(faces_), verts(verts_), n_faces(n_faces_), n_verts(n_verts_)
    {
    }

    const TriangleFace*   getFaces() const { return faces; }
    const TriangleVertex* getVerts() const { return verts; }
    const uint32_t        getNumFaces() const { return n_faces; }
    const uint32_t        getNumVerts() const { return n_verts; }
};

class TriangleMesh
{
private:
    std::vector<TriangleFace>   faces;
    std::vector<TriangleVertex> verts;
    std::vector<AABB>           aabbs;
    bool                        is_aabb_built = false;

public:
    uint32_t                           numFaces() const { return faces.size(); }
    uint32_t                           numVerts() const { return verts.size(); }
    const std::vector<TriangleFace>&   getFaces() const { return faces; }
    const std::vector<TriangleVertex>& getVerts() const { return verts; }

    float get_area(const uint32_t triangle_idx) const
    {
        const auto& f = faces[triangle_idx];
        const auto& a = verts[f.ia].position;
        const auto& b = verts[f.ib].position;
        const auto& c = verts[f.ic].position;
        float3      d = cross((b - a), (c - a));
        return length(d) * 0.5f;
    }

    void addTriangleVertex(const TriangleVertex& vert)
    {
        verts.push_back(vert);
    }
    void addTriangleVertex(TriangleVertex&& vert)
    {
        verts.push_back(std::move(vert));
    }
    void addTriangleFace(const TriangleFace& face) { faces.push_back(face); }
    void addTriangleFace(TriangleFace&& face)
    {
        faces.push_back(std::move(face));
    }

    void append(const TriangleMesh& other)
    {
        auto& verts = other.getVerts();
        auto& faces = other.getFaces();

        uint32_t vertex_offset = this->numVerts();
        for (auto& v : verts)
        {
            this->addTriangleVertex(v);
        }
        for (auto f : faces)
        {
            f.ia += vertex_offset;
            f.ib += vertex_offset;
            f.ic += vertex_offset;
            this->addTriangleFace(f);
        }
    }

    void append(const TriangleMesh& other, const mat44& xform)
    {
        auto& verts = other.getVerts();
        auto& faces = other.getFaces();

        uint32_t vertex_offset = this->numVerts();
        for (auto v : verts)
        {
            glm::vec4 p(v.position.x, v.position.y, v.position.z, 1.0f);
            p = xform * p;

            v.position = float3(p.x, p.y, p.z);

            this->addTriangleVertex(v);
        }
        for (auto f : faces)
        {
            f.ia += vertex_offset;
            f.ib += vertex_offset;
            f.ic += vertex_offset;
            this->addTriangleFace(f);
        }
    }
};

class CudaBVH;
class Scene;

class TriangleMeshCUDA
{
public:
    TriangleMeshCUDA(const TriangleMesh& mesh)
    {
        verts = mesh.getVerts();
        faces = mesh.getFaces();
    }

    TriangleMeshInterface getInterface() const
    {
        TriangleMeshInterface ret(
            faces.data(), verts.data(), faces.size(), verts.size());
        return ret;
    }

private:
    std::vector<TriangleFace>   faces;
    std::vector<TriangleVertex> verts;
};

//#define EntrypointSentinel 0x76543210

inline float max_of(const float& x, const float& y, const float& z)
{
    return fmaxf(fmaxf(x, y), z);
}

inline float min_of(const float& x, const float& y, const float& z)
{
    return fminf(fminf(x, y), z);
}

inline float spanBeginKepler(
    float a0, float a1, float b0, float b1, float c0, float c1, float d)
{
    return max_of(fminf(a0, a1), fminf(b0, b1), fmaxf(fminf(c0, c1), d));
}
inline float spanEndKepler(
    float a0, float a1, float b0, float b1, float c0, float c1, float d)
{
    return min_of(fmaxf(a0, a1), fmaxf(b0, b1), fminf(fmaxf(c0, c1), d));
}

inline void swap2(int& a, int& b)
{
    int temp = a;
    a        = b;
    b        = temp;
}

inline int float_as_int(const float& f)
{
    return *reinterpret_cast<const int*>(&f);
}

class NvSplitBVH : public Geometry
{
public:
    NvSplitBVH(const Mesh& mesh);

    ~NvSplitBVH();

    bool intersect(const Ray& ray, float& tNear) const override;

private:
    CudaBVH* m_bvh   = nullptr;
    Scene*   m_scene = nullptr;
    float4*  bvhNodes;
    float4*  triWoop;
    int*     triIndices;
    uint32_t tri_count;
    uint32_t leaf_node_count;
    uint32_t node_size;

    std::vector<float4> cudaNodeBuffer;
    std::vector<float4> cudaTriWoopBuffer;
    std::vector<S32>    cudaTriIndicesBuffer;
};
}  // namespace nvsbvh
