#ifndef util_h__
#define util_h__

#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <memory>
#include <functional>
#include <cstdarg>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using std::cout;
using std::endl;

typedef glm::vec2 gVec2;
typedef glm::vec3 gVec3;
typedef glm::vec4 gVec4;
typedef glm::mat4x4 gMat4;
typedef glm::mat3x3 gMat3;
typedef glm::ivec3 gInt3;
typedef glm::quat gQuaternion;

// const float M_PI = 3.14159265358979323846f;
//float randf();
//
//// output
//class Logger
//{
//    std::ofstream output;
//public:
//    Logger(std::string logfile, bool override = true);
//    ~Logger();
//    template<class T> Logger& operator << (const T& v);
//};
//
//void lprintf(const char *sformat, ...);

void Abort(unsigned int err);

std::string printToString(const char *format, ...);

template <typename T>
void print(const std::vector<T> &x)
{
    cout << "[ ";
    for (int n = 0; n < x.size() - 1; n++)
    {
        cout << x[n] << ", ";
    }
    cout << x[x.size() - 1] << " ]" << endl;
}

inline std::ostream& operator<<(std::ostream& out, const gMat4& mat)
{
    out << mat[0][0] << ", " << mat[1][0] << ", " << mat[2][0] << ", " << mat[3][0] << endl;
    out << mat[0][1] << ", " << mat[1][1] << ", " << mat[2][1] << ", " << mat[3][1] << endl;
    out << mat[0][2] << ", " << mat[1][2] << ", " << mat[2][2] << ", " << mat[3][2] << endl;
    out << mat[0][3] << ", " << mat[1][3] << ", " << mat[2][3] << ", " << mat[3][3] << endl;

    return out;
}

inline std::ostream& operator<<(std::ostream& out, const gQuaternion& q)
{
    out << q.w << ", " << q.x << ", " << q.y << ", " << q.z;

    return out;
}

inline std::ostream& operator<<(std::ostream& out, const gVec3& v)
{
    out << v.x << ", " << v.y << ", " << v.z;

    return out;
}

/// image loaders
enum
{
    FLIP_UP_DOWN = 1,
    FLIP_LEFT_RIGHT = 2,
};

#endif // util_h__