#include "LinearMath.h"

#include <random>

bool isNormalized(const float3& vec)
{
    return fabs(length(vec) - 1.0f) < 1e-6f;
}

float clampf(float x, float a, float b) { return x < a ? a : x > b ? b : x; }

std::default_random_engine            rng;
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

float randf() { return dist(rng); }

constexpr float k_norm = 1.0f / (float)RAND_MAX;
float           randfFast() { return (rand() + 0.5f) * k_norm; }

const float& get(const float3& v, int i) { return (&v.x)[i]; }

float& get(float3& v, int i) { return (&v.x)[i]; }
