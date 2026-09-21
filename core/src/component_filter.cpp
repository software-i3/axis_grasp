#include "axis_grasp/core/component_filter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
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

    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const Vec2i& point : band) {
      mean_x += point.x;
      mean_y += point.y;
    }
    mean_x /= band.size();
    mean_y /= band.size();
    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;
    for (const Vec2i& point : band) {
      const double dx = point.x - mean_x;
      const double dy = point.y - mean_y;
      cxx += dx * dx;
      cxy += dx * dy;
      cyy += dy * dy;
    }
    cxx /= band.size();
    cxy /= band.size();
    cyy /= band.size();
    const double trace = cxx + cyy;
    const double discriminant =
        std::sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy);
    const double lambda_max = 0.5 * (trace + discriminant);
    const double lambda_min = 0.5 * (trace - discriminant);
    if (lambda_min <= 1e-12 ||
        lambda_max / lambda_min < config.elongation_ratio) {
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
    if (skeleton_points.size() < 8) continue;

    double skeleton_mean_x = 0.0;
    double skeleton_mean_y = 0.0;
    for (const Vec2i& point : skeleton_points) {
      skeleton_mean_x += point.x;
      skeleton_mean_y += point.y;
    }
    skeleton_mean_x /= skeleton_points.size();
    skeleton_mean_y /= skeleton_points.size();
    double ssxx = 0.0;
    double ssxy = 0.0;
    double ssyy = 0.0;
    for (const Vec2i& point : skeleton_points) {
      const double dx = point.x - skeleton_mean_x;
      const double dy = point.y - skeleton_mean_y;
      ssxx += dx * dx;
      ssxy += dx * dy;
      ssyy += dy * dy;
    }
    const double skeleton_trace = ssxx + ssyy;
    const double skeleton_discriminant =
        std::sqrt((ssxx - ssyy) * (ssxx - ssyy) + 4.0 * ssxy * ssxy);
    const double skeleton_lambda_max =
        0.5 * (skeleton_trace + skeleton_discriminant);
    double vx = 0.0;
    double vy = 0.0;
    if (ssxy != 0.0) {
      vx = skeleton_lambda_max - ssyy;
      vy = ssxy;
    } else if (ssxx >= ssyy) {
      vx = 1.0;
    } else {
      vy = 1.0;
    }
    const double axis_norm = std::hypot(vx, vy);
    vx /= axis_norm;
    vy /= axis_norm;

    std::vector<double> skeleton_distance;
    skeleton_distance.reserve(skeleton_points.size());
    for (const Vec2i& point : skeleton_points) {
      skeleton_distance.push_back(std::abs(
          vx * (point.y - skeleton_mean_y) -
          vy * (point.x - skeleton_mean_x)));
    }
    const double residual_95 =
        internal::Percentile(std::move(skeleton_distance), 95.0);
    std::vector<double> band_distance;
    band_distance.reserve(band.size());
    for (const Vec2i& point : band) {
      const double px = point.x + kPadding;
      const double py = point.y + kPadding;
      band_distance.push_back(std::abs(
          vx * (py - skeleton_mean_y) - vy * (px - skeleton_mean_x)));
    }
    const double band_width = internal::Percentile(std::move(band_distance), 95.0);
    if (residual_95 >
        std::max(config.residual_pixels,
                 config.residual_fraction * band_width)) {
      continue;
    }

    const Vec2i first = band.front();
    const int full_x = x0 + first.x;
    const int full_y = y0 + first.y;
    const int positive_label = positive_components.labels(full_y, full_x);
    if (positive_label > 0) {
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
