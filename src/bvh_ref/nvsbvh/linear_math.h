#pragma once

#include <math.h>

#include <cassert>
#include <cstdint>
#include <glm/glm.hpp>

namespace nvsbvh
{
#undef min
#undef max

#define FW_ASSERT assert

typedef uint8_t  U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;
typedef int8_t   S8;
typedef int16_t  S16;
typedef int32_t  S32;
typedef int64_t  S64;
typedef float    F32;
typedef double   F64;

typedef glm::ivec2 int2;
typedef glm::ivec3 int3;
typedef glm::ivec4 int4;
typedef glm::vec2  float2;
typedef glm::vec3  float3;
typedef glm::vec4  float4;
typedef glm::mat4  mat44;

inline float max1f(const float& a, const float& b) { return (a < b) ? b : a; }
inline float min1f(const float& a, const float& b) { return (a > b) ? b : a; }

class Vec2f : public float2
{
public:
    Vec2f()
    {
        x = 0;
        y = 0;
    }
    explicit Vec2f(float a) { x = y = a; }
    Vec2f(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
    Vec2f(const float2& v) : float2(v) {}
    Vec2f& operator=(const float2& v)
    {
        x = v.x;
        y = v.y;
        return *this;
    }
    float&       operator[](int n) { return (&x)[n]; }
    const float& operator[](int n) const { return (&x)[n]; }

    bool  operator==(const Vec2f& v) { return x == v.x && y == v.y; }
    Vec2f operator+(const Vec2f& v) const { return Vec2f(x + v.x, y + v.y); }
    Vec2f operator-(const Vec2f& v) const { return Vec2f(x - v.x, y - v.y); }
    Vec2f operator*(const Vec2f& v) const { return Vec2f(x * v.x, y * v.y); }
};

struct Vec2i
{
    /// int x, y;
    union
    {
        struct
        {
            int x, y;
        };
        int _v[2];
    };

    Vec2i(int _x = 0, int _y = 0) : x(_x), y(_y) {}
    Vec2i(const Vec2i& v) : x(v.x), y(v.y) {}

    bool operator==(const Vec2i& v) { return x == v.x && y == v.y; }
};

// float max1f(const float& a, const float& b)
//{
//    return (a < b) ? b : a;
//}
// float min1f(const float& a, const float& b)
//{
//    return (a > b) ? b : a;
//}

class Vec3f : public float3
{
public:
    Vec3f()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    explicit Vec3f(float a) { x = y = z = a; }
    Vec3f(const Vec2f& v, float z_)
    {
        x = v.x;
        y = v.y;
        z = z_;
    }
    Vec3f(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    Vec3f(const float3& v) : float3(v) {}
    Vec3f& operator=(const float3& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }
    float&       operator[](int n) { return (&x)[n]; }
    const float& operator[](int n) const { return (&x)[n]; }

    float length() { return sqrtf(x * x + y * y + z * z); }
    // sometimes we dont need the sqrt, we are just comparing one length with
    // another
    float lengthsq() { return x * x + y * y + z * z; }
    float max() { return max1f(max1f(x, y), z); }
    float min() { return min1f(min1f(x, y), z); }
    Vec3f normalize()
    {
        float norm = 1.0f / sqrtf(x * x + y * y + z * z);
        x *= norm;
        y *= norm;
        z *= norm;
        return Vec3f(x, y, z);
    }
    Vec3f normalized()
    {
        float norm = 1.0f / sqrtf(x * x + y * y + z * z);
        return Vec3f(x * norm, y * norm, z * norm);
    }
    Vec3f& operator+=(const Vec3f& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    Vec3f& operator-=(const Vec3f& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    Vec3f& operator*=(const float& a)
    {
        x *= a;
        y *= a;
        z *= a;
        return *this;
    }
    Vec3f& operator*=(const Vec3f& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }
    Vec3f operator*(float a) const { return Vec3f(x * a, y * a, z * a); }
    Vec3f operator/(float a) const { return Vec3f(x / a, y / a, z / a); }
    Vec3f operator*(const Vec3f& v) const
    {
        return Vec3f(x * v.x, y * v.y, z * v.z);
    }
    Vec3f operator+(const Vec3f& v) const
    {
        return Vec3f(x + v.x, y + v.y, z + v.z);
    }
    Vec3f operator-(const Vec3f& v) const
    {
        return Vec3f(x - v.x, y - v.y, z - v.z);
    }
    Vec3f operator/(const Vec3f& v) const
    {
        return Vec3f(x / v.x, y / v.y, z / v.z);
    }
    Vec3f& operator/=(const float& a)
    {
        x /= a;
        y /= a;
        z /= a;
        return *this;
    }
    bool operator!=(const Vec3f& v) { return x != v.x || y != v.y || z != v.z; }
    bool operator==(const Vec3f& v) { return x == v.x && y == v.y && z == v.z; }
};

struct Vec3i
{
    union
    {
        struct
        {
            int x, y, z;
        };
        int _v[3];
    };
    /// int x, y, z;

    Vec3i(int _x = 0, int _y = 0, int _z = 0) : x(_x), y(_y), z(_z) {}
    Vec3i(const Vec3i& v) : x(v.x), y(v.y), z(v.z) {}
    Vec3i(const Vec3f& vf) : x(vf.x), y(vf.y), z(vf.z) {}

    bool operator==(const Vec3i& v) { return x == v.x && y == v.y && z == v.z; }
};

struct Vec4f
{
    union
    {
        struct
        {
            float x, y, z, w;
        };
        float _v[4];
    };
    /// float x, y, z, w;

    Vec4f(float _x = 0, float _y = 0, float _z = 0, float _w = 0)
        : x(_x), y(_y), z(_z), w(_w)
    {
    }
    Vec4f(const Vec4f& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
    Vec4f(const Vec3f& v, const float a) : x(v.x), y(v.y), z(v.z), w(a) {}

    Vec4f& operator+=(const Vec4f& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
        return *this;
    }
    Vec4f& operator*=(const Vec4f& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        w *= v.w;
        return *this;
    }
};

struct Vec4i
{
    int x, y, z, w;

    Vec4i(int _x = 0, int _y = 0, int _z = 0, int _w = 0)
        : x(_x), y(_y), z(_z), w(_w)
    {
    }
    Vec4i(const Vec4i& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
    Vec4i(const Vec3i& v, const int a) : x(v.x), y(v.y), z(v.z), w(a) {}
};

inline Vec3f min3f(const Vec3f& v1, const Vec3f& v2)
{
    return Vec3f(v1.x < v2.x ? v1.x : v2.x,
                 v1.y < v2.y ? v1.y : v2.y,
                 v1.z < v2.z ? v1.z : v2.z);
}
inline Vec3f max3f(const Vec3f& v1, const Vec3f& v2)
{
    return Vec3f(v1.x > v2.x ? v1.x : v2.x,
                 v1.y > v2.y ? v1.y : v2.y,
                 v1.z > v2.z ? v1.z : v2.z);
}

//-------------------------------------------------------------------------------------------------

//----------------------------------
// Matrix algebra
//----------------------------------

// based on Aila Laine framework , without templates

class Mat3f
{
public:
    inline void set(const float* ptr)
    {
        FW_ASSERT(ptr);
        for (int i = 0; i < 3 * 3; i++) get(i) = ptr[i];
    }
    inline void set(const float& a)
    {
        for (int i = 0; i < 3 * 3; i++) get(i) = a;
    }
    inline void setZero(void) { set((float)0); }
    inline void setIdentity(void)
    {
        setZero();
        for (int i = 0; i < 3; i++) get(i, i) = (float)1;
    }
    inline const float& get(int idx) const
    {
        FW_ASSERT(idx >= 0 && idx < 3 * 3);
        return getPtr()[idx];
    }
    inline float& get(int idx)
    {
        FW_ASSERT(idx >= 0 && idx < 3 * 3);
        return getPtr()[idx];
    }
    inline const float& get(int r, int c) const
    {
        FW_ASSERT(r >= 0 && r < 3 && c >= 0 && c < 3);
        return getPtr()[r + c * 3];
    }
    inline float& get(int r, int c)
    {
        FW_ASSERT(r >= 0 && r < 3 && c >= 0 && c < 3);
        return getPtr()[r + c * 3];
    }
    inline void   set(const Mat3f& v) { set(v.getPtr()); }
    inline Mat3f& operator=(const float& a)
    {
        set(a);
        return *(Mat3f*)this;
    }
    inline const float& operator()(int r, int c) const { return get(r, c); }
    inline float&       operator()(int r, int c) { return get(r, c); }
    inline float        det(void) const;

    inline Mat3f(void) { setIdentity(); }
    inline explicit Mat3f(F32 a) { set(a); }

    inline const F32*   getPtr(void) const { return &m00; }
    inline F32*         getPtr(void) { return &m00; }
    static inline Mat3f fromPtr(const F32* ptr)
    {
        Mat3f v;
        v.set(ptr);
        return v;
    }

    inline Mat3f(const Mat3f& v) { set(v); }
    inline Mat3f& operator=(const Mat3f& v)
    {
        set(v);
        return *this;
    }

    //#if !FW_CUDA
    //    static Mat3f rotation(const Vec3f& axis,
    //                          F32 angle);  // Rotation of "angle" radians
    //                          around
    //                                       // "axis". Axis must be unit!
    //#endif

public:
    F32 m00, m10, m20;
    F32 m01, m11, m21;
    F32 m02, m12, m22;
};

class Mat4f
{
public:
    inline const F32*   getPtr(void) const { return &m00; }
    inline F32*         getPtr(void) { return &m00; }
    inline const Vec4f& col(int c) const
    {
        FW_ASSERT(c >= 0 && c < 4);
        return *(const Vec4f*)(getPtr() + c * 4);
    }
    inline Vec4f& col(int c)
    {
        FW_ASSERT(c >= 0 && c < 4);
        return *(Vec4f*)(getPtr() + c * 4);
    }
    inline const Vec4f& getCol(int c) const { return col(c); }
    inline Vec4f        getCol0() const
    {
        Vec4f col;
        col.x = m00;
        col.y = m01;
        col.z = m02;
        col.w = m03;
        return col;
    }
    inline Vec4f getCol1() const
    {
        Vec4f col;
        col.x = m10;
        col.y = m11;
        col.z = m12;
        col.w = m13;
        return col;
    }
    inline Vec4f getCol2() const
    {
        Vec4f col;
        col.x = m20;
        col.y = m21;
        col.z = m22;
        col.w = m23;
        return col;
    }
    inline Vec4f getCol3() const
    {
        Vec4f col;
        col.x = m30;
        col.y = m31;
        col.z = m32;
        col.w = m33;
        return col;
    }
    inline Vec4f        getRow(int r) const;
    inline Mat4f        inverted4x4(void);
    inline void         invert(void) { set(inverted4x4()); }
    inline const float& get(int idx) const
    {
        FW_ASSERT(idx >= 0 && idx < 4 * 4);
        return getPtr()[idx];
    }
    inline float& get(int idx)
    {
        FW_ASSERT(idx >= 0 && idx < 4 * 4);
        return getPtr()[idx];
    }
    inline const float& get(int r, int c) const
    {
        FW_ASSERT(r >= 0 && r < 4 && c >= 0 && c < 4);
        return getPtr()[r + c * 4];
    }
    inline float& get(int r, int c)
    {
        FW_ASSERT(r >= 0 && r < 4 && c >= 0 && c < 4);
        return getPtr()[r + c * 4];
    }
    inline void set(const float& a)
    {
        for (int i = 0; i < 4 * 4; i++) get(i) = a;
    }
    inline void set(const float* ptr)
    {
        FW_ASSERT(ptr);
        for (int i = 0; i < 4 * 4; i++) get(i) = ptr[i];
    }
    inline void setZero(void) { set((float)0); }
    inline void setIdentity(void)
    {
        setZero();
        for (int i = 0; i < 4; i++) get(i, i) = (float)1;
    }
    inline void setCol(int c, const Vec4f& v) { col(c) = v; }
    inline void setCol0(const Vec4f& v)
    {
        m00 = v.x;
        m01 = v.y;
        m02 = v.z;
        m03 = v.w;
    }
    inline void setCol1(const Vec4f& v)
    {
        m10 = v.x;
        m11 = v.y;
        m12 = v.z;
        m13 = v.w;
    }
    inline void setCol2(const Vec4f& v)
    {
        m20 = v.x;
        m21 = v.y;
        m22 = v.z;
        m23 = v.w;
    }
    inline void setCol3(const Vec4f& v)
    {
        m30 = v.x;
        m31 = v.y;
        m32 = v.z;
        m33 = v.w;
    }
    inline void   setRow(int r, const Vec4f& v);
    inline void   set(const Mat4f& v) { set(v.getPtr()); }
    inline Mat4f& operator=(const float& a)
    {
        set(a);
        return *(Mat4f*)this;
    }
    inline Mat4f operator*(const float& a) const
    {
        Mat4f r;
        for (int i = 0; i < 4 * 4; i++) r.get(i) = get(i) * a;
        return r;
    }
    inline const float& operator()(int r, int c) const { return get(r, c); }
    inline float&       operator()(int r, int c) { return get(r, c); }

    inline Mat4f(void) { setIdentity(); }
    inline explicit Mat4f(F32 a) { set(a); }
    static inline Mat4f fromPtr(const F32* ptr)
    {
        Mat4f v;
        v.set(ptr);
        return v;
    }

    inline Mat4f(const Mat4f& v) { set(v); }
    inline Mat4f& operator=(const Mat4f& v)
    {
        set(v);
        return *this;
    }

public:
    F32 m00, m10, m20, m30;
    F32 m01, m11, m21, m31;
    F32 m02, m12, m22, m32;
    F32 m03, m13, m23, m33;
};

inline Mat4f invert(Mat4f& v) { return v.inverted4x4(); }

static const int c_popc8LUT[] =  // CUDA CONST
    {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4,
        2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4,
        2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
        4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5,
        3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6,
        4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};

inline int popc8(U32 mask) { return c_popc8LUT[mask & 0xFFu]; }

inline int popc16(U32 mask)
{
    return c_popc8LUT[mask & 0xFFu] + c_popc8LUT[(mask >> 8) & 0xFFu];
}

inline int popc32(U32 mask)
{
    int result = c_popc8LUT[mask & 0xFFu];
    result += c_popc8LUT[(mask >> 8) & 0xFFu];
    result += c_popc8LUT[(mask >> 16) & 0xFFu];
    result += c_popc8LUT[mask >> 24];
    return result;
}

inline int popc64(U64 mask)
{
    U32 lo     = (U32)mask;
    U32 hi     = (U32)(mask >> 32);
    int result = c_popc8LUT[lo & 0xffu] + c_popc8LUT[hi & 0xffu];
    result += c_popc8LUT[(lo >> 8) & 0xffu] + c_popc8LUT[(hi >> 8) & 0xffu];
    result += c_popc8LUT[(lo >> 16) & 0xffu] + c_popc8LUT[(hi >> 16) & 0xffu];
    result += c_popc8LUT[lo >> 24] + c_popc8LUT[hi >> 24];
    return result;
}

//------------------------------------------------------------------------

Vec4f Mat4f::getRow(int idx) const
{
    Vec4f r;
    for (int i = 0; i < 4; i++) r._v[i] = get(idx, i);
    return r;
}

void Mat4f::setRow(int idx, const Vec4f& v)
{
    for (int i = 0; i < 4; i++) get(idx, i) = v._v[i];
}

inline float detImpl(const Mat3f& v)
{
    return v(0, 0) * v(1, 1) * v(2, 2) - v(0, 0) * v(1, 2) * v(2, 1) +
           v(1, 0) * v(2, 1) * v(0, 2) - v(1, 0) * v(2, 2) * v(0, 1) +
           v(2, 0) * v(0, 1) * v(1, 2) - v(2, 0) * v(0, 2) * v(1, 1);
}

float Mat3f::det(void) const { return detImpl(*this); }

// efficient column major matrix inversion function from
// http://rodolphe-vaillant.fr/?e=7

Mat4f Mat4f::inverted4x4(void)
{
    float inv[16];
    float m[16] = {m00,
                   m10,
                   m20,
                   m30,
                   m01,
                   m11,
                   m21,
                   m31,
                   m02,
                   m12,
                   m22,
                   m32,
                   m03,
                   m13,
                   m23,
                   m33};

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
             m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] +
             m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] +
             m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
             m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] +
              m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] +
              m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] +
             m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] +
             m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
             m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
             m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
              m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
             m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
             m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
              m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
              m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
             m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
              m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0) return Mat4f();

    det = 1.f / det;
    Mat4f inverse;
    for (int i = 0; i < 16; i++) inverse.get(i) = inv[i] * det;

    return inverse;
}
}  // namespace nvsbvh
