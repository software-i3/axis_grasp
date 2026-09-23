#include "axis_grasp/core/component_filter.h"

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

#include "internal/image_ops.h"

namespace axis_grasp {
namespace {

// skimage.morphology.skeletonize(method="zhang") classifies every possible
// 8-neighborhood with one lookup value: class 1 deletes in subiteration 1,
// class 2 in subiteration 2, and class 3 in both. This compatibility table is
// deliberately retained as data because scikit-image's optimized table is not
// identical to independently re-evaluating the textbook predicates for all 25
// asymmetric edge cases. See THIRD_PARTY_NOTICES.md.
//
// Neighborhood bit order matches scikit-image's _fast_skeletonize:
//
//   NW  N NE       1   2   4
//    W  .  E     128   .   8
//   SW  S SE      64  32  16
constexpr std::array<std::uint8_t, 256> kZhangSuenLut{
    0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0, 1, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 3, 0, 3, 3,
    0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 2, 2,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    2, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0,
    3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 2, 0,
    0, 0, 3, 1, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    2, 3, 1, 3, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    3, 3, 0, 1, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0};

std::uint8_t NeighborhoodCode(const Image<std::uint8_t>& image, int y,
                              int x) {
  return static_cast<std::uint8_t>(
      ((image(y - 1, x - 1) != 0) ? 1U : 0U) |
      ((image(y - 1, x) != 0) ? 2U : 0U) |
      ((image(y - 1, x + 1) != 0) ? 4U : 0U) |
      ((image(y, x + 1) != 0) ? 8U : 0U) |
      ((image(y + 1, x + 1) != 0) ? 16U : 0U) |
      ((image(y + 1, x) != 0) ? 32U : 0U) |
      ((image(y + 1, x - 1) != 0) ? 64U : 0U) |
      ((image(y, x - 1) != 0) ? 128U : 0U));
}

std::vector<Vec2i> PixelsForLabel(const Image<int>& labels, int label,
                                  int x0, int y0, int width, int height) {
  std::vector<Vec2i> points;
  for (int y = y0; y < y0 + height; ++y) {
    for (int x = x0; x < x0 + width; ++x) {
      if (labels(y, x) == label) points.push_back({x - x0, y - y0});
    }
  }
  return points;
}

// Deterministic generator for the RANSAC line fit. The plane fit in
// grasp_proposal.cpp carries a numpy-compatible PCG64 because it has to match a
// Python reference bit for bit; this filter deliberately departs from its
// Python original, so all it needs is a reproducible stream from a configured
// seed.
class SplitMix64 {
 public:
  explicit SplitMix64(std::uint32_t seed)
      : state_(static_cast<std::uint64_t>(seed) + 0x9e3779b97f4a7c15ULL) {}

  std::uint64_t Next() {
    state_ += 0x9e3779b97f4a7c15ULL;
    std::uint64_t value = state_;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
  }

  // Uniform over [0, exclusive_maximum); the rejection loop removes the bias a
  // bare remainder would carry.
  std::uint32_t Bounded(std::uint32_t exclusive_maximum) {
    if (exclusive_maximum <= 1) return 0;
    const std::uint32_t limit =
        std::numeric_limits<std::uint32_t>::max() -
        (std::numeric_limits<std::uint32_t>::max() % exclusive_maximum);
    std::uint32_t value = static_cast<std::uint32_t>(Next());
    while (value >= limit) value = static_cast<std::uint32_t>(Next());
    return value % exclusive_maximum;
  }

 private:
  std::uint64_t state_;
};

// Two distinct indices drawn uniformly without replacement. There is no
// permutation step here, unlike NumpyChoiceThree: a line through two points
// does not care which point is visited first.
std::array<std::size_t, 2> SampleTwoIndices(std::size_t population,
                                            SplitMix64* generator) {
  std::array<std::size_t, 2> result{};
  result[0] = generator->Bounded(static_cast<std::uint32_t>(population - 1));
  std::size_t second =
      generator->Bounded(static_cast<std::uint32_t>(population - 2));
  if (second >= result[0]) ++second;
  result[1] = second;
  return result;
}

struct Pca2d {
  double mean_x = 0.0;
  double mean_y = 0.0;
  double lambda_max = 0.0;
  double lambda_min = 0.0;
  double direction_x = 0.0;
  double direction_y = 0.0;
};

// Covariance eigen-decomposition of a 2-D point set, normalized by the point
// count. Shared by the elongation gate, which reads the eigenvalue ratio, and
// the RANSAC refit, which reads the major-axis direction.
Pca2d ComputePca2d(const std::vector<Vec2i>& points) {
  Pca2d result;
  if (points.empty()) return result;
  const double count = static_cast<double>(points.size());
  for (const Vec2i& point : points) {
    result.mean_x += point.x;
    result.mean_y += point.y;
  }
  result.mean_x /= count;
  result.mean_y /= count;
  double cxx = 0.0;
  double cxy = 0.0;
  double cyy = 0.0;
  for (const Vec2i& point : points) {
    const double dx = point.x - result.mean_x;
    const double dy = point.y - result.mean_y;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }
  cxx /= count;
  cxy /= count;
  cyy /= count;
  const double trace = cxx + cyy;
  const double discriminant =
      std::sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy);
  result.lambda_max = 0.5 * (trace + discriminant);
  result.lambda_min = 0.5 * (trace - discriminant);
  if (cxy != 0.0) {
    result.direction_x = result.lambda_max - cyy;
    result.direction_y = cxy;
  } else if (cxx >= cyy) {
    result.direction_x = 1.0;
  } else {
    result.direction_y = 1.0;
  }
  const double norm = std::hypot(result.direction_x, result.direction_y);
  if (norm > 0.0) {
    result.direction_x /= norm;
    result.direction_y /= norm;
  }
  return result;
}

double PerpendicularDistance(const Pca2d& line, double x, double y) {
  return std::abs(line.direction_x * (y - line.mean_y) -
                  line.direction_y * (x - line.mean_x));
}

struct LineFit {
  Pca2d line;
  std::vector<Vec2i> inliers;
  double inlier_fraction = 0.0;
};

// RANSAC over 2-point line hypotheses, shaped like FitPlaneRansac in
// grasp_proposal.cpp: sample, score, keep the best consensus, then refit the
// direction by PCA over the winning inliers. Returns nullopt when the consensus
// covers less than min_inlier_fraction of the points.
std::optional<LineFit> FitLineRansac(const std::vector<Vec2i>& points,
                                     double distance_threshold, int iterations,
                                     double min_inlier_fraction,
                                     std::uint32_t seed) {
  if (points.size() < 2) return std::nullopt;
  if (points.size() > std::numeric_limits<std::uint32_t>::max()) {
    return std::nullopt;
  }
  SplitMix64 generator(seed);
  std::size_t best_count = 0;
  std::size_t best_first = 0;
  std::size_t best_second = 0;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const std::array<std::size_t, 2> sample =
        SampleTwoIndices(points.size(), &generator);
    const Vec2i& a = points[sample[0]];
    const Vec2i& b = points[sample[1]];
    const double dx = static_cast<double>(b.x - a.x);
    const double dy = static_cast<double>(b.y - a.y);
    const double norm = std::hypot(dx, dy);
    if (norm <= 0.0) continue;
    const double vx = dx / norm;
    const double vy = dy / norm;
    std::size_t count = 0;
    for (const Vec2i& point : points) {
      if (std::abs(vx * (point.y - a.y) - vy * (point.x - a.x)) <=
          distance_threshold) {
        ++count;
      }
    }
    if (count > best_count) {
      best_count = count;
      best_first = sample[0];
      best_second = sample[1];
    }
  }
  if (best_count < 2) return std::nullopt;
  const Vec2i& anchor = points[best_first];
  const Vec2i& second = points[best_second];
  const double dx = static_cast<double>(second.x - anchor.x);
  const double dy = static_cast<double>(second.y - anchor.y);
  const double norm = std::hypot(dx, dy);
  if (norm <= 0.0) return std::nullopt;
  const double vx = dx / norm;
  const double vy = dy / norm;
  std::vector<Vec2i> inliers;
  inliers.reserve(best_count);
  for (const Vec2i& point : points) {
    if (std::abs(vx * (point.y - anchor.y) - vy * (point.x - anchor.x)) <=
        distance_threshold) {
      inliers.push_back(point);
    }
  }
  const double fraction =
      static_cast<double>(inliers.size()) / static_cast<double>(points.size());
  if (inliers.size() < 2 || fraction < min_inlier_fraction) {
    return std::nullopt;
  }
  return LineFit{ComputePca2d(inliers), std::move(inliers), fraction};
}

}  // namespace

Image<std::uint8_t> Skeletonize(const Image<std::uint8_t>& mask) {
  // The one-pixel zero border is observable: unlike an implementation that
  // simply skips image-edge pixels, it lets foreground touching an edge thin
  // exactly as if the image were surrounded by background. This is also how
  // scikit-image handles 2-D inputs.
  Image<std::uint8_t> padded(mask.height() + 2, mask.width() + 2, 0);
  for (int y = 0; y < mask.height(); ++y) {
    for (int x = 0; x < mask.width(); ++x) {
      padded(y + 1, x + 1) = mask(y, x) != 0;
    }
  }

  bool changed = true;
  std::vector<Vec2i> remove;
  while (changed) {
    changed = false;
    for (std::uint8_t pass = 1; pass <= 2; ++pass) {
      remove.clear();
      for (int y = 1; y <= mask.height(); ++y) {
        for (int x = 1; x <= mask.width(); ++x) {
          if (padded(y, x) == 0) continue;
          const std::uint8_t classification =
              kZhangSuenLut[NeighborhoodCode(padded, y, x)];
          if (classification == pass || classification == 3) {
            remove.push_back({x, y});
          }
        }
      }
      if (!remove.empty()) changed = true;
      for (const Vec2i& point : remove) padded(point.y, point.x) = 0;
    }
  }

  Image<std::uint8_t> skeleton(mask.height(), mask.width(), 0);
  for (int y = 0; y < mask.height(); ++y) {
    for (int x = 0; x < mask.width(); ++x) {
      skeleton(y, x) = padded(y + 1, x + 1);
    }
  }
  return skeleton;
}

Result<Image<float>> RemoveStraightComponents(
    const Image<float>& coherent, int close_kernel,
    const RopeFilterConfig& config) {
  std::vector<float> positive;
  positive.reserve(coherent.size());
  for (float value : coherent.data()) {
    if (value > 0.0F) positive.push_back(value);
  }
  if (positive.empty()) return Image<float>(coherent);

  const double threshold = internal::Percentile(positive, config.percentile);
  Image<std::uint8_t> detection(coherent.height(), coherent.width(), 0);
  Image<std::uint8_t> positive_mask(coherent.height(), coherent.width(), 0);
  for (std::size_t i = 0; i < coherent.size(); ++i) {
    detection[i] = coherent[i] >= threshold ? 1 : 0;
    positive_mask[i] = coherent[i] > 0.0F ? 1 : 0;
  }
  const Image<std::uint8_t> closed =
      close_kernel > 1 ? internal::Close(detection, close_kernel) : detection;
  const internal::ConnectedComponents detected =
      internal::LabelConnected(closed, 8);
  const internal::ConnectedComponents positive_components =
      internal::LabelConnected(positive_mask, 8);
  Image<float> output = coherent;

  for (int label = 1; label < static_cast<int>(detected.area.size()); ++label) {
    if (detected.area[label] < config.min_pixels) continue;
    const int x0 = detected.left[label];
    const int y0 = detected.top[label];
    const int width = detected.width[label];
    const int height = detected.height[label];
    const std::vector<Vec2i> band =
        PixelsForLabel(detected.labels, label, x0, y0, width, height);
    if (band.empty()) continue;

    const Pca2d band_pca = ComputePca2d(band);
    if (band_pca.lambda_min <= 1e-12 ||
        band_pca.lambda_max / band_pca.lambda_min < config.elongation_ratio) {
      continue;
    }

    constexpr int kPadding = 2;
    Image<std::uint8_t> crop(height + 2 * kPadding,
                             width + 2 * kPadding, 0);
    for (const Vec2i& point : band) {
      crop(point.y + kPadding, point.x + kPadding) = 1;
    }
    const Image<std::uint8_t> skeleton = Skeletonize(crop);
    std::vector<Vec2i> skeleton_points;
    for (int y = 0; y < skeleton.height(); ++y) {
      for (int x = 0; x < skeleton.width(); ++x) {
        if (skeleton(y, x) != 0) skeleton_points.push_back({x, y});
      }
    }
    // A short skeleton is the chord of something curved, not a rope line: a
    // curve is straight over a short enough span, so these fragments used to be
    // deleted as rope along with the real thing.
    if (skeleton_points.size() <
        static_cast<std::size_t>(config.min_skeleton_pixels)) {
      continue;
    }

    const std::optional<LineFit> fit =
        FitLineRansac(skeleton_points, config.ransac_distance_pixels,
                      config.ransac_iterations,
                      config.ransac_min_inlier_fraction, config.ransac_seed);
    if (!fit.has_value()) continue;

    // Both scales are measured from the consensus set alone, so a fragment
    // merged into the band cannot widen the tolerance that is supposed to
    // describe the rope.
    std::vector<double> skeleton_distance;
    skeleton_distance.reserve(fit->inliers.size());
    for (const Vec2i& point : fit->inliers) {
      skeleton_distance.push_back(
          PerpendicularDistance(fit->line, point.x, point.y));
    }
    const double residual_95 =
        internal::Percentile(std::move(skeleton_distance), 95.0);
    std::vector<double> band_distance;
    band_distance.reserve(band.size());
    for (const Vec2i& point : band) {
      const double distance = PerpendicularDistance(
          fit->line, point.x + kPadding, point.y + kPadding);
      if (distance <= config.ransac_distance_pixels) {
        band_distance.push_back(distance);
      }
    }
    if (band_distance.empty()) continue;
    const double band_width =
        internal::Percentile(std::move(band_distance), 95.0);
    if (residual_95 >
        std::max(config.residual_pixels,
                 config.residual_fraction * band_width)) {
      continue;
    }

    // Zero every positive-mask component the band touches, not just the one
    // holding band.front(). A component that is correctly classified as
    // straight can still arrive as several raw coherent>0 fragments -- a
    // low-value gap near an attachment point splits it, and only the closing
    // step bridges them for classification. Clearing the first raster-scanned
    // fragment left the rest of the visible rope in the output.
    std::vector<std::uint8_t> cleared(positive_components.area.size(), 0);
    for (const Vec2i& point : band) {
      const int full_x = x0 + point.x;
      const int full_y = y0 + point.y;
      const int positive_label = positive_components.labels(full_y, full_x);
      if (positive_label <= 0 || cleared[positive_label] != 0) continue;
      cleared[positive_label] = 1;
      for (std::size_t i = 0; i < output.size(); ++i) {
        if (positive_components.labels[i] == positive_label) output[i] = 0.0F;
      }
    }
  }
  return output;
}

Result<ComponentOutput> FilterAccumulator(const Image<float>& accumulator,
                                          const ComponentConfig& config) {
  if (config.close_kernel < 1 || config.top_components < 1 ||
      config.accumulator_percentile < 0.0 ||
      config.accumulator_percentile > 100.0) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Invalid accumulator filtering configuration");
  }
  std::vector<float> positive;
  positive.reserve(accumulator.size());
  for (float value : accumulator.data()) {
    if (value > 0.0F) positive.push_back(value);
  }
  if (positive.empty()) {
    return Status::Error(ErrorCode::kNoVotes,
                         "Directional accumulator contains no positive votes");
  }
  const double threshold =
      internal::Percentile(positive, config.accumulator_percentile);
  ComponentOutput output;
  output.threshold_mask =
      Image<std::uint8_t>(accumulator.height(), accumulator.width(), 0);
  for (std::size_t i = 0; i < accumulator.size(); ++i) {
    output.threshold_mask[i] = accumulator[i] > threshold ? 1 : 0;
  }
  output.closed_mask =
      internal::Close(output.threshold_mask, config.close_kernel);
  internal::ConnectedComponents components =
      internal::LabelConnected(output.closed_mask, 8);
  output.labels = std::move(components.labels);
  output.weights.assign(components.area.size(), 0.0);
  for (std::size_t i = 0; i < accumulator.size(); ++i) {
    output.weights[output.labels[i]] += accumulator[i];
  }
  if (output.weights.size() <= 1) {
    return Status::Error(ErrorCode::kNoComponents,
                         "No connected accumulator component survived filtering");
  }
  std::vector<int> labels(output.weights.size() - 1);
  std::iota(labels.begin(), labels.end(), 1);
  std::stable_sort(labels.begin(), labels.end(), [&](int a, int b) {
    return output.weights[a] > output.weights[b];
  });
  const std::size_t keep_count = std::min(
      labels.size(), static_cast<std::size_t>(config.top_components));
  output.kept_labels.assign(labels.begin(), labels.begin() + keep_count);
  return output;
}

}  // namespace axis_grasp
