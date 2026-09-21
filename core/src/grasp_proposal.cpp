#include "axis_grasp/core/grasp_proposal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

#include "axis_grasp/core/component_filter.h"
#include "internal/image_ops.h"

namespace axis_grasp {
namespace {

// NumPy's default_rng uses PCG64. RANSAC must draw the same three indices as
// numpy.random.Generator.choice(..., 3, replace=False), otherwise two valid
// but different consensus sets can rotate the refined post-skeleton plane.
// This small implementation mirrors NumPy's SeedSequence, PCG64 XSL-RR, the
// bounded uint32 Lemire sampler, and the three-element Floyd sample used by
// choice. It stays private to the algorithm and requires no runtime library.
struct Uint128 {
  std::uint64_t high = 0;
  std::uint64_t low = 0;
};

std::uint64_t MultiplyHigh64(std::uint64_t x, std::uint64_t y) {
  constexpr std::uint64_t kMask32 = 0xFFFFFFFFULL;
  const std::uint64_t x0 = x & kMask32;
  const std::uint64_t x1 = x >> 32;
  const std::uint64_t y0 = y & kMask32;
  const std::uint64_t y1 = y >> 32;
  const std::uint64_t w0 = x0 * y0;
  const std::uint64_t t = x1 * y0 + (w0 >> 32);
  std::uint64_t w1 = t & kMask32;
  const std::uint64_t w2 = t >> 32;
  w1 += x0 * y1;
  return x1 * y1 + w2 + (w1 >> 32);
}

Uint128 Add128(Uint128 a, Uint128 b) {
  Uint128 result;
  result.low = a.low + b.low;
  result.high = a.high + b.high + (result.low < b.low ? 1 : 0);
  return result;
}

Uint128 Multiply128(Uint128 a, Uint128 b) {
  return {MultiplyHigh64(a.low, b.low) + a.high * b.low + a.low * b.high,
          a.low * b.low};
}

Uint128 ShiftLeftOne(Uint128 value) {
  return {(value.high << 1) | (value.low >> 63), value.low << 1};
}

std::uint64_t RotateRight64(std::uint64_t value, unsigned int rotation) {
  return (value >> rotation) | (value << ((-rotation) & 63));
}

std::uint32_t HashMix(std::uint32_t value, std::uint32_t* hash_constant) {
  constexpr std::uint32_t kMultiplier = 0x931e8875U;
  value ^= *hash_constant;
  *hash_constant *= kMultiplier;
  value *= *hash_constant;
  value ^= value >> 16;
  return value;
}

std::uint32_t Mix(std::uint32_t x, std::uint32_t y) {
  constexpr std::uint32_t kMultiplierLeft = 0xca01f9ddU;
  constexpr std::uint32_t kMultiplierRight = 0x4973f715U;
  std::uint32_t result = kMultiplierLeft * x - kMultiplierRight * y;
  result ^= result >> 16;
  return result;
}

std::array<std::uint64_t, 4> NumpySeedSequence(std::uint32_t seed) {
  std::array<std::uint32_t, 4> pool{};
  std::uint32_t hash_constant = 0x43b0d7e5U;
  for (std::size_t i = 0; i < pool.size(); ++i) {
    pool[i] = HashMix(i == 0 ? seed : 0U, &hash_constant);
  }
  for (std::size_t source = 0; source < pool.size(); ++source) {
    for (std::size_t destination = 0; destination < pool.size();
         ++destination) {
      if (source != destination) {
        pool[destination] =
            Mix(pool[destination], HashMix(pool[source], &hash_constant));
      }
    }
  }

  std::array<std::uint32_t, 8> words{};
  hash_constant = 0x8b51f9ddU;
  constexpr std::uint32_t kOutputMultiplier = 0x58f38dedU;
  for (std::size_t i = 0; i < words.size(); ++i) {
    std::uint32_t value = pool[i % pool.size()];
    value ^= hash_constant;
    hash_constant *= kOutputMultiplier;
    value *= hash_constant;
    value ^= value >> 16;
    words[i] = value;
  }

  std::array<std::uint64_t, 4> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = static_cast<std::uint64_t>(words[2 * i]) |
                (static_cast<std::uint64_t>(words[2 * i + 1]) << 32);
  }
  return result;
}

class NumpyPcg64 {
 public:
  explicit NumpyPcg64(std::uint32_t seed) {
    const std::array<std::uint64_t, 4> generated = NumpySeedSequence(seed);
    const Uint128 initial_state{generated[0], generated[1]};
    const Uint128 initial_sequence{generated[2], generated[3]};
    increment_ = ShiftLeftOne(initial_sequence);
    increment_.low |= 1U;
    Step();
    state_ = Add128(state_, initial_state);
    Step();
  }

  std::uint32_t BoundedUint32(std::uint32_t inclusive_maximum) {
    if (inclusive_maximum == 0) return 0;
    const std::uint32_t range = inclusive_maximum + 1U;
    std::uint64_t product =
        static_cast<std::uint64_t>(NextUint32()) * range;
    std::uint32_t leftover = static_cast<std::uint32_t>(product);
    if (leftover < range) {
      const std::uint32_t threshold =
          (std::numeric_limits<std::uint32_t>::max() - inclusive_maximum) %
          range;
      while (leftover < threshold) {
        product = static_cast<std::uint64_t>(NextUint32()) * range;
        leftover = static_cast<std::uint32_t>(product);
      }
    }
    return static_cast<std::uint32_t>(product >> 32);
  }

 private:
  void Step() {
    constexpr Uint128 kMultiplier{2549297995355413924ULL,
                                  4865540595714422341ULL};
    state_ = Add128(Multiply128(state_, kMultiplier), increment_);
  }

  std::uint64_t NextUint64() {
    Step();
    const unsigned int rotation = static_cast<unsigned int>(state_.high >> 58);
    return RotateRight64(state_.high ^ state_.low, rotation);
  }

  std::uint32_t NextUint32() {
    if (has_buffered_uint32_) {
      has_buffered_uint32_ = false;
      return buffered_uint32_;
    }
    const std::uint64_t value = NextUint64();
    buffered_uint32_ = static_cast<std::uint32_t>(value >> 32);
    has_buffered_uint32_ = true;
    return static_cast<std::uint32_t>(value);
  }

  Uint128 state_{};
  Uint128 increment_{};
  bool has_buffered_uint32_ = false;
  std::uint32_t buffered_uint32_ = 0;
};

std::array<std::size_t, 3> NumpyChoiceThree(std::size_t population,
                                            NumpyPcg64* generator) {
  std::array<std::size_t, 3> result{};
  for (std::size_t output = 0; output < result.size(); ++output) {
    const std::size_t j = population - result.size() + output;
    std::size_t value = generator->BoundedUint32(
        static_cast<std::uint32_t>(j));
    if (std::find(result.begin(), result.begin() + output, value) !=
        result.begin() + output) {
      value = j;
    }
    result[output] = value;
  }
  for (std::size_t i = result.size() - 1; i > 0; --i) {
    const std::size_t j = generator->BoundedUint32(
        static_cast<std::uint32_t>(i));
    std::swap(result[i], result[j]);
  }
  return result;
}

Vec3d SmallestCovarianceEigenvector(const std::vector<Vec3d>& points,
                                    const Vec3d& centroid) {
  double matrix[3][3] = {};
  for (const Vec3d& point : points) {
    const Vec3d delta = point - centroid;
    matrix[0][0] += delta.x * delta.x;
    matrix[0][1] += delta.x * delta.y;
    matrix[0][2] += delta.x * delta.z;
    matrix[1][1] += delta.y * delta.y;
    matrix[1][2] += delta.y * delta.z;
    matrix[2][2] += delta.z * delta.z;
  }
  matrix[1][0] = matrix[0][1];
  matrix[2][0] = matrix[0][2];
  matrix[2][1] = matrix[1][2];
  double vectors[3][3] = {{1.0, 0.0, 0.0},
                          {0.0, 1.0, 0.0},
                          {0.0, 0.0, 1.0}};

  for (int iteration = 0; iteration < 32; ++iteration) {
    int p = 0;
    int q = 1;
    double maximum = std::abs(matrix[p][q]);
    for (int row = 0; row < 3; ++row) {
      for (int column = row + 1; column < 3; ++column) {
        if (std::abs(matrix[row][column]) > maximum) {
          maximum = std::abs(matrix[row][column]);
          p = row;
          q = column;
        }
      }
    }
    if (maximum < 1e-15) break;
    const double angle =
        0.5 * std::atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);

    const double app = matrix[p][p];
    const double aqq = matrix[q][q];
    const double apq = matrix[p][q];
    matrix[p][p] = cosine * cosine * app - 2.0 * sine * cosine * apq +
                   sine * sine * aqq;
    matrix[q][q] = sine * sine * app + 2.0 * sine * cosine * apq +
                   cosine * cosine * aqq;
    matrix[p][q] = 0.0;
    matrix[q][p] = 0.0;
    for (int index = 0; index < 3; ++index) {
      if (index == p || index == q) continue;
      const double aip = matrix[index][p];
      const double aiq = matrix[index][q];
      matrix[index][p] = cosine * aip - sine * aiq;
      matrix[p][index] = matrix[index][p];
      matrix[index][q] = sine * aip + cosine * aiq;
      matrix[q][index] = matrix[index][q];
    }
    for (int row = 0; row < 3; ++row) {
      const double vip = vectors[row][p];
      const double viq = vectors[row][q];
      vectors[row][p] = cosine * vip - sine * viq;
      vectors[row][q] = sine * vip + cosine * viq;
    }
  }
  int smallest = 0;
  if (matrix[1][1] < matrix[smallest][smallest]) smallest = 1;
  if (matrix[2][2] < matrix[smallest][smallest]) smallest = 2;
  return Normalize(
      {vectors[0][smallest], vectors[1][smallest], vectors[2][smallest]});
}

Vec3d Mean(const std::vector<Vec3d>& points) {
  Vec3d sum;
  for (const Vec3d& point : points) sum += point;
  return sum / static_cast<double>(points.size());
}

std::optional<Vec3d> PostSkeletonNormal(
    const Image<std::uint8_t>& skeleton, const Image<Vec3d>& points,
    const GraspConfig& config) {
  const internal::ConnectedComponents chains =
      internal::LabelConnected(skeleton, 8);
  if (chains.area.size() <= 1) return std::nullopt;
  int dominant_label = 1;
  for (int label = 2; label < static_cast<int>(chains.area.size()); ++label) {
    if (chains.area[label] > chains.area[dominant_label]) {
      dominant_label = label;
    }
  }
  const int dilation = config.post_skeleton_dilation_pixels;
  const Image<std::uint8_t> band =
      internal::Dilate(skeleton, 2 * dilation + 1);
  std::vector<Vec2i> skeleton_points;
  skeleton_points.reserve(skeleton.size());
  for (int y = 0; y < skeleton.height(); ++y) {
    for (int x = 0; x < skeleton.width(); ++x) {
      if (skeleton(y, x) != 0) skeleton_points.push_back({x, y});
    }
  }

  std::vector<Vec3d> fit_points;
  for (int y = 0; y < band.height(); ++y) {
    for (int x = 0; x < band.width(); ++x) {
      if (band(y, x) == 0 || !IsFinite(points(y, x))) continue;
      int nearest_label = 0;
      int nearest_distance_sq = std::numeric_limits<int>::max();
      // The band was formed with a square dilation of this radius, so its
      // nearest skeleton sample is guaranteed to lie in this local window.
      for (const Vec2i& candidate : skeleton_points) {
        if (std::abs(candidate.x - x) > dilation ||
            std::abs(candidate.y - y) > dilation) {
          continue;
        }
        const int dx = candidate.x - x;
        const int dy = candidate.y - y;
        const int distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_distance_sq) {
          nearest_distance_sq = distance_sq;
          nearest_label = chains.labels(candidate.y, candidate.x);
        }
      }
      if (nearest_label == dominant_label) fit_points.push_back(points(y, x));
    }
  }
  if (fit_points.size() < 5) return std::nullopt;
  const std::optional<PlaneFit> plane = FitPlaneRansac(
      fit_points, config.ransac_distance_m, config.ransac_iterations,
      config.ransac_min_inliers, config.ransac_min_inlier_fraction,
      config.ransac_seed);
  if (!plane.has_value()) return std::nullopt;
  return OrientNormalTowardCamera(plane->normal, plane->inlier_centroid);
}

std::vector<Vec2d> EqualTargets(const std::vector<Vec3d>& positions,
                                double spacing_m,
                                std::vector<std::size_t>* rotation_indices) {
  std::vector<double> cumulative(positions.size(), 0.0);
  for (std::size_t i = 1; i < positions.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + Norm(positions[i] - positions[i - 1]);
  }
  const double total = cumulative.back();
  std::vector<Vec2d> brackets;
  if (total <= 0.0) {
    rotation_indices->push_back(0);
    brackets.push_back({0.0, 0.0});
    return brackets;
  }
  const int count = std::max(
      1, static_cast<int>(std::nearbyint(total / spacing_m)) + 1);
  brackets.reserve(count);
  rotation_indices->reserve(count);
  for (int index = 0; index < count; ++index) {
    const double target = count == 1
                              ? 0.0
                              : total * index / static_cast<double>(count - 1);
    const auto it = std::lower_bound(cumulative.begin(), cumulative.end(), target);
    const std::size_t upper = static_cast<std::size_t>(
        std::distance(cumulative.begin(), it));
    rotation_indices->push_back(std::min(upper, positions.size() - 1));
    if (upper == 0) {
      brackets.push_back({0.0, 0.0});
    } else {
      const double denominator =
          std::max(cumulative[upper] - cumulative[upper - 1], 1e-12);
      const double fraction = std::clamp(
          (target - cumulative[upper - 1]) / denominator, 0.0, 1.0);
      brackets.push_back({static_cast<double>(upper - 1), fraction});
    }
  }
  return brackets;
}

void ReorientChain(std::vector<Mat3d>* rotations) {
  for (std::size_t i = 1; i < rotations->size(); ++i) {
    const Mat3d& previous = (*rotations)[i - 1];
    Mat3d& current = (*rotations)[i];
    const double keep_score = Dot(current.column(0), previous.column(0)) +
                              Dot(current.column(1), previous.column(1));
    const double flip_score = Dot(current.column(0), previous.column(0)) -
                              Dot(current.column(1), previous.column(1));
    if (flip_score > keep_score) {
      current.set_column(1, -current.column(1));
      current.set_column(2, -current.column(2));
    }
  }
}

}  // namespace

Image<Vec3d> ReprojectDisparity(const Image<float>& disparity,
                               const CameraIntrinsics& intrinsics) {
  Image<Vec3d> points(disparity.height(), disparity.width());
  for (int y = 0; y < disparity.height(); ++y) {
    for (int x = 0; x < disparity.width(); ++x) {
      const double t = intrinsics.baseline_m / disparity(y, x);
      points(y, x) = {t * (x - intrinsics.cx),
                      -t * (y - intrinsics.cy), -t * intrinsics.fx};
    }
  }
  return points;
}

std::vector<Vec2i> OrderSkeletonPoints(
    const Image<std::uint8_t>& skeleton) {
  std::vector<Vec2i> points;
  for (int y = 0; y < skeleton.height(); ++y) {
    for (int x = 0; x < skeleton.width(); ++x) {
      if (skeleton(y, x) != 0) points.push_back({x, y});
    }
  }
  if (points.empty()) return points;

  std::size_t start = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    int degree = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int neighbor_x = points[i].x + dx;
        const int neighbor_y = points[i].y + dy;
        if ((dx != 0 || dy != 0) && neighbor_x >= 0 &&
            neighbor_x < skeleton.width() && neighbor_y >= 0 &&
            neighbor_y < skeleton.height() &&
            skeleton(neighbor_y, neighbor_x) != 0) {
          ++degree;
        }
      }
    }
    if (degree == 1) {
      start = i;
      break;
    }
  }
  std::vector<bool> visited(points.size(), false);
  std::vector<Vec2i> ordered;
  ordered.reserve(points.size());
  std::size_t current = start;
  visited[current] = true;
  ordered.push_back(points[current]);
  while (ordered.size() < points.size()) {
    std::size_t best = points.size();
    long long best_distance = std::numeric_limits<long long>::max();
    for (std::size_t candidate = 0; candidate < points.size(); ++candidate) {
      if (visited[candidate]) continue;
      const long long dx = points[candidate].x - points[current].x;
      const long long dy = points[candidate].y - points[current].y;
      const long long distance = dx * dx + dy * dy;
      if (distance < best_distance) {
        best_distance = distance;
        best = candidate;
      }
    }
    if (best == points.size()) break;
    visited[best] = true;
    ordered.push_back(points[best]);
    current = best;
  }
  return ordered;
}

std::vector<Vec2d> ResamplePolyline(const std::vector<Vec2i>& points,
                                    double spacing_pixels) {
  std::vector<Vec2d> result;
  if (points.size() < 2) {
    for (const Vec2i& point : points) {
      result.push_back({static_cast<double>(point.x),
                        static_cast<double>(point.y)});
    }
    return result;
  }
  std::vector<double> cumulative(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    cumulative[i] = cumulative[i - 1] +
                    std::hypot(points[i].x - points[i - 1].x,
                               points[i].y - points[i - 1].y);
  }
  const double total = cumulative.back();
  const int count =
      std::max(2, static_cast<int>(std::floor(total / spacing_pixels)) + 1);
  result.reserve(count);
  for (int i = 0; i < count; ++i) {
    const double target = total * i / static_cast<double>(count - 1);
    auto upper_it = std::lower_bound(cumulative.begin(), cumulative.end(), target);
    std::size_t upper =
        static_cast<std::size_t>(std::distance(cumulative.begin(), upper_it));
    if (upper == 0) {
      result.push_back({static_cast<double>(points[0].x),
                        static_cast<double>(points[0].y)});
      continue;
    }
    upper = std::min(upper, points.size() - 1);
    const double segment = cumulative[upper] - cumulative[upper - 1];
    const double fraction = segment <= 0.0
                                ? 0.0
                                : (target - cumulative[upper - 1]) / segment;
    result.push_back(
        {(1.0 - fraction) * points[upper - 1].x + fraction * points[upper].x,
         (1.0 - fraction) * points[upper - 1].y + fraction * points[upper].y});
  }
  return result;
}

std::optional<Mat3d> BuildGraspFrame(const Vec3d& tangent,
                                     const Vec3d& surface_normal) {
  const Vec3d approach = -surface_normal / (Norm(surface_normal) + 1e-12);
  Vec3d jaw_axis = tangent - approach * Dot(tangent, approach);
  const double jaw_norm = Norm(jaw_axis);
  if (jaw_norm < 1e-8) return std::nullopt;
  jaw_axis = jaw_axis / jaw_norm;
  Vec3d closing_axis = Cross(approach, jaw_axis);
  const double closing_norm = Norm(closing_axis);
  if (closing_norm < 1e-8) return std::nullopt;
  closing_axis = closing_axis / closing_norm;
  jaw_axis = Cross(closing_axis, approach);
  Mat3d rotation;
  rotation.set_column(0, approach);
  rotation.set_column(1, jaw_axis);
  rotation.set_column(2, closing_axis);
  return rotation;
}

std::optional<PlaneFit> FitPlaneRansac(
    const std::vector<Vec3d>& points, double distance_threshold, int iterations,
    int min_inliers, double min_inlier_fraction, std::uint32_t seed) {
  if (points.size() < 3) return std::nullopt;
  if (points.size() > std::numeric_limits<std::uint32_t>::max()) {
    return std::nullopt;
  }
  NumpyPcg64 generator(seed);
  int best_count = -1;
  Vec3d best_normal;
  Vec3d best_anchor;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const std::array<std::size_t, 3> sample =
        NumpyChoiceThree(points.size(), &generator);
    const std::size_t ia = sample[0];
    const std::size_t ib = sample[1];
    const std::size_t ic = sample[2];
    const Vec3d& a = points[ia];
    const Vec3d candidate_cross = Cross(points[ib] - a, points[ic] - a);
    const double denominator = Norm(candidate_cross);
    if (denominator < 1e-12) continue;
    const Vec3d normal = candidate_cross / denominator;
    int count = 0;
    for (const Vec3d& point : points) {
      if (std::abs(Dot(point - a, normal)) <= distance_threshold) ++count;
    }
    if (count > best_count) {
      best_count = count;
      best_normal = normal;
      best_anchor = a;
    }
  }
  if (best_count < min_inliers ||
      static_cast<double>(best_count) / points.size() < min_inlier_fraction) {
    return std::nullopt;
  }
  std::vector<Vec3d> inliers;
  inliers.reserve(best_count);
  for (const Vec3d& point : points) {
    if (std::abs(Dot(point - best_anchor, best_normal)) <= distance_threshold) {
      inliers.push_back(point);
    }
  }
  const Vec3d centroid = Mean(inliers);
  return PlaneFit{SmallestCovarianceEigenvector(inliers, centroid), centroid};
}

Vec3d OrientNormalTowardCamera(const Vec3d& normal, const Vec3d& anchor) {
  return Dot(normal, -anchor) < 0.0 ? -normal : normal;
}

Quaterniond RotationToQuaternion(const Mat3d& matrix) {
  Quaterniond quaternion;
  const double trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * scale;
    quaternion.x = (matrix(2, 1) - matrix(1, 2)) / scale;
    quaternion.y = (matrix(0, 2) - matrix(2, 0)) / scale;
    quaternion.z = (matrix(1, 0) - matrix(0, 1)) / scale;
  } else if (matrix(0, 0) > matrix(1, 1) &&
             matrix(0, 0) > matrix(2, 2)) {
    const double scale =
        std::sqrt(1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2)) * 2.0;
    quaternion.w = (matrix(2, 1) - matrix(1, 2)) / scale;
    quaternion.x = 0.25 * scale;
    quaternion.y = (matrix(0, 1) + matrix(1, 0)) / scale;
    quaternion.z = (matrix(0, 2) + matrix(2, 0)) / scale;
  } else if (matrix(1, 1) > matrix(2, 2)) {
    const double scale =
        std::sqrt(1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2)) * 2.0;
    quaternion.w = (matrix(0, 2) - matrix(2, 0)) / scale;
    quaternion.x = (matrix(0, 1) + matrix(1, 0)) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (matrix(1, 2) + matrix(2, 1)) / scale;
  } else {
    const double scale =
        std::sqrt(1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1)) * 2.0;
    quaternion.w = (matrix(1, 0) - matrix(0, 1)) / scale;
    quaternion.x = (matrix(0, 2) + matrix(2, 0)) / scale;
    quaternion.y = (matrix(1, 2) + matrix(2, 1)) / scale;
    quaternion.z = 0.25 * scale;
  }
  const double norm = std::sqrt(
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

Result<std::vector<GraspArc>> ProposeGrasps(
    const Image<Vec3d>& points, const Image<std::uint8_t>& closed_mask,
    const Image<int>& component_labels, const std::vector<int>& kept_labels,
    const GraspConfig& config) {
  if (points.height() != closed_mask.height() ||
      points.width() != closed_mask.width() ||
      points.height() != component_labels.height() ||
      points.width() != component_labels.width()) {
    return Status::Error(ErrorCode::kShapeMismatch,
                         "Point grid and component images have different shapes");
  }
  std::vector<GraspArc> arcs;
  for (int label : kept_labels) {
    Image<std::uint8_t> component(points.height(), points.width(), 0);
    bool has_pixel = false;
    for (std::size_t i = 0; i < component.size(); ++i) {
      if (closed_mask[i] != 0 && component_labels[i] == label) {
        component[i] = 1;
        has_pixel = true;
      }
    }
    if (!has_pixel) continue;
    const Image<std::uint8_t> skeleton = Skeletonize(component);
    const std::vector<Vec2i> ordered = OrderSkeletonPoints(skeleton);
    if (ordered.empty()) continue;
    const std::vector<Vec2d> dense =
        ResamplePolyline(ordered, config.dense_spacing_pixels);

    std::optional<Vec3d> normal;
    if (config.strategy == GraspStrategy::kCameraNormal) {
      if (Norm(config.camera_approach_axis) <= 1e-12) {
        return Status::Error(ErrorCode::kInvalidArgument,
                             "Camera approach axis has zero length");
      }
      normal = -Normalize(config.camera_approach_axis);
    } else {
      normal = PostSkeletonNormal(skeleton, points, config);
    }
    if (!normal.has_value()) continue;

    std::vector<Vec3d> positions;
    std::vector<Mat3d> rotations;
    std::vector<GraspPoint> dense_points;
    for (std::size_t i = 0; i < dense.size(); ++i) {
      const int x = static_cast<int>(std::nearbyint(dense[i].x));
      const int y = static_cast<int>(std::nearbyint(dense[i].y));
      if (x < config.edge_margin_pixels ||
          x >= points.width() - config.edge_margin_pixels ||
          y < config.edge_margin_pixels ||
          y >= points.height() - config.edge_margin_pixels) {
        continue;
      }
      const Vec3d position = points(y, x);
      if (!IsFinite(position)) continue;
      const std::size_t previous = i == 0 ? 0 : i - 1;
      const std::size_t next = std::min(i + 1, dense.size() - 1);
      const int x0 = std::clamp(
          static_cast<int>(std::nearbyint(dense[previous].x)), 0,
          points.width() - 1);
      const int y0 = std::clamp(
          static_cast<int>(std::nearbyint(dense[previous].y)), 0,
          points.height() - 1);
      const int x1 = std::clamp(
          static_cast<int>(std::nearbyint(dense[next].x)), 0,
          points.width() - 1);
      const int y1 = std::clamp(
          static_cast<int>(std::nearbyint(dense[next].y)), 0,
          points.height() - 1);
      if (!IsFinite(points(y0, x0)) || !IsFinite(points(y1, x1))) continue;
      const Vec3d tangent = points(y1, x1) - points(y0, x0);
      if (Norm(tangent) < 1e-9) continue;
      std::optional<Mat3d> rotation = BuildGraspFrame(tangent, *normal);
      if (!rotation.has_value()) continue;
      positions.push_back(position);
      rotations.push_back(*rotation);
      dense_points.push_back({{x, y}, position});
    }
    if (positions.empty()) continue;

    double arc_length = 0.0;
    for (std::size_t i = 1; i < positions.size(); ++i) {
      arc_length += Norm(positions[i] - positions[i - 1]);
    }
    std::vector<std::size_t> rotation_indices;
    const std::vector<Vec2d> brackets =
        EqualTargets(positions, config.output_spacing_m, &rotation_indices);
    std::vector<Vec3d> equal_positions;
    std::vector<Mat3d> equal_rotations;
    equal_positions.reserve(brackets.size());
    equal_rotations.reserve(brackets.size());
    for (std::size_t i = 0; i < brackets.size(); ++i) {
      const std::size_t lower = static_cast<std::size_t>(brackets[i].x);
      const double fraction = brackets[i].y;
      if (lower == 0 && fraction == 0.0) {
        equal_positions.push_back(positions[0]);
      } else {
        equal_positions.push_back(positions[lower] * (1.0 - fraction) +
                                  positions[lower + 1] * fraction);
      }
      equal_rotations.push_back(rotations[rotation_indices[i]]);
    }
    ReorientChain(&equal_rotations);

    GraspArc arc;
    arc.source_component = label;
    arc.skeleton_pixels.reserve(ordered.size());
    for (const Vec2i& point : ordered) {
      arc.skeleton_pixels.push_back(
          {static_cast<double>(point.x), static_cast<double>(point.y)});
    }
    arc.dense_points = std::move(dense_points);
    arc.arc_length_m = arc_length;
    for (std::size_t i = 0; i < equal_positions.size(); ++i) {
      arc.poses.push_back({equal_positions[i], equal_rotations[i],
                           RotationToQuaternion(equal_rotations[i])});
    }
    arcs.push_back(std::move(arc));
  }
  if (arcs.empty()) {
    return Status::Error(
        ErrorCode::kNoValidGrasps,
        "Centerline components were found but produced no valid grasp poses");
  }
  return arcs;
}

}  // namespace axis_grasp
