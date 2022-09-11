#pragma once

#include <algorithm>
#include <glm/glm.hpp>
#include <limits>

typedef glm::ivec2 int2;
typedef glm::ivec3 int3;
typedef glm::vec3  float3;
typedef glm::vec4  float4;
typedef glm::mat4  mat44;

// TODO : use max() instead ??
static constexpr float kInfinity = std::numeric_limits<float>::infinity();
static constexpr float kEpsilon  = std::numeric_limits<float>::epsilon();

bool isNormalized(const float3& vec);

float clampf(float x, float a, float b);

float randf();
float randfFast();

// TODO : implement as member
const float& get(const float3& v, int i);
float&       get(float3& v, int i);
