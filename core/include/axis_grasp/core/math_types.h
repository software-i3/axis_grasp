#ifndef AXIS_GRASP_CORE_MATH_TYPES_H_
#define AXIS_GRASP_CORE_MATH_TYPES_H_

#include <array>
#include <cmath>

namespace axis_grasp {

struct Vec2d {
  double x = 0.0;
  double y = 0.0;
};

struct Vec2i {
  int x = 0;
  int y = 0;
};

struct Vec3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Vec3d operator+(const Vec3d& a, const Vec3d& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3d operator-(const Vec3d& a, const Vec3d& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3d operator-(const Vec3d& a) { return {-a.x, -a.y, -a.z}; }
inline Vec3d operator*(const Vec3d& a, double scale) {
  return {a.x * scale, a.y * scale, a.z * scale};
}
inline Vec3d operator*(double scale, const Vec3d& a) { return a * scale; }
inline Vec3d operator/(const Vec3d& a, double scale) {
  return {a.x / scale, a.y / scale, a.z / scale};
}
inline Vec3d& operator+=(Vec3d& a, const Vec3d& b) {
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
  return a;
}
inline double Dot(const Vec3d& a, const Vec3d& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3d Cross(const Vec3d& a, const Vec3d& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
inline double Norm(const Vec3d& value) { return std::sqrt(Dot(value, value)); }
inline bool IsFinite(const Vec3d& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}
inline Vec3d Normalize(const Vec3d& value, double epsilon = 1e-12) {
  return value / (Norm(value) + epsilon);
}

// Row-major storage. Grasp rotation columns have the semantic order
// approach, jaw-length/tangent, closing axis.
struct Mat3d {
  std::array<double, 9> values{1.0, 0.0, 0.0, 0.0, 1.0,
                               0.0, 0.0, 0.0, 1.0};

  double& operator()(int row, int column) { return values[row * 3 + column]; }
  double operator()(int row, int column) const {
    return values[row * 3 + column];
  }
  Vec3d column(int index) const {
    return {(*this)(0, index), (*this)(1, index), (*this)(2, index)};
  }
  void set_column(int index, const Vec3d& value) {
    (*this)(0, index) = value.x;
    (*this)(1, index) = value.y;
    (*this)(2, index) = value.z;
  }
};

struct Quaterniond {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_MATH_TYPES_H_
