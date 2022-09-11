#include "Ray.h"
#include <sstream>

using std::endl;

float Ray::m_rayEpsilon = 1e-4f;

std::string Ray::toString() const
{
    std::stringstream ss;
    ss << "o(" << m_origin.x << ", " << m_origin.y << ", " << m_origin.z
       << "), ";
    ss << "d(" << m_direction.x << ", " << m_direction.y << ", "
       << m_direction.z << ")";
    ss << "t[" << m_tMin << ", " << m_tMax << "]";
    return ss.str();
}
