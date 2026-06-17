// Small 2D/3D vector helpers. Header-only; everything is constexpr/inline so the
// optimiser folds it away. Replaces V1's loose tuple maths in misc_functions.py.
#pragma once

#include <cmath>

namespace koda {

struct Vec2 {
  float x = 0.0f;
  float y = 0.0f;

  constexpr Vec2() = default;
  constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

  constexpr Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  constexpr Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  constexpr Vec2 operator*(float s) const { return {x * s, y * s}; }

  float length() const { return std::hypot(x, y); }
  float dist(const Vec2& o) const { return std::hypot(x - o.x, y - o.y); }
  float dot(const Vec2& o) const { return x * o.x + y * o.y; }
  // 2D scalar cross product (z of the 3D cross) — handy for torque/moment sums.
  float cross(const Vec2& o) const { return x * o.y - y * o.x; }
};

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Vec3() = default;
  constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

  float length() const { return std::sqrt(x * x + y * y + z * z); }
};

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

// map_value() from V1 misc_functions.py
inline float map_range(float v, float in_lo, float in_hi, float out_lo, float out_hi) {
  return (v - in_lo) * (out_hi - out_lo) / (in_hi - in_lo) + out_lo;
}

}  // namespace koda
