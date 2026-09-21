#include "axis_grasp/core/axis_voting.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "axis_grasp/core/component_filter.h"
#include "internal/image_ops.h"

namespace axis_grasp {

Result<VotingOutput> AccumulateDirectionalVotes(
    const Image<float>& disparity, const Image<std::uint8_t>* roi,
    const VotingConfig& config) {
  if (disparity.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Disparity image is empty");
  }
  if (roi != nullptr &&
      (roi->height() != disparity.height() || roi->width() != disparity.width())) {
    return Status::Error(ErrorCode::kShapeMismatch,
                         "ROI and disparity shapes differ");
  }
  if (config.radius_min < 0 || config.radius_max < config.radius_min ||
      config.direction_bins < 2 || config.direction_bins % 2 != 0 ||
      config.magnitude_percentile < 0.0 ||
      config.magnitude_percentile > 100.0) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Invalid directional voting configuration");
  }

  const int height = disparity.height();
  const int width = disparity.width();
  VotingOutput output;
  output.filtered_disparity = internal::BilateralFilter(
      disparity, config.bilateral_diameter, config.bilateral_sigma_color,
      config.bilateral_sigma_space);
  Image<float> gradient_x;
  Image<float> gradient_y;
  internal::Sobel(output.filtered_disparity, &gradient_x, &gradient_y);
  Image<float> magnitude(height, width, 0.0F);
  std::vector<float> roi_magnitudes;
  roi_magnitudes.reserve(disparity.size());
  for (std::size_t i = 0; i < disparity.size(); ++i) {
    magnitude[i] = std::hypot(gradient_x[i], gradient_y[i]);
    if (roi == nullptr || (*roi)[i] != 0) roi_magnitudes.push_back(magnitude[i]);
  }
  if (roi_magnitudes.empty()) {
    return Status::Error(ErrorCode::kEmptyRoi,
                         "ROI does not contain any pixels");
  }
  const double magnitude_threshold =
      internal::Percentile(roi_magnitudes, config.magnitude_percentile);

  struct Candidate {
    int x;
    int y;
    int bin;
    float weight;
    float direction_x;
    float direction_y;
  };
  std::vector<Candidate> candidates;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if ((roi != nullptr && (*roi)(y, x) == 0) ||
          magnitude(y, x) <= magnitude_threshold) {
        continue;
      }
      const float mag = magnitude(y, x);
      const float gx = gradient_x(y, x) / (mag + 1e-8F);
      const float gy = gradient_y(y, x) / (mag + 1e-8F);
      const double angle = std::atan2(gy, gx);
      constexpr double kPi = 3.141592653589793238462643383279502884;
      int bin = static_cast<int>(std::floor(
          (angle + kPi) / (2.0 * kPi) * config.direction_bins));
      bin %= config.direction_bins;
      if (bin < 0) bin += config.direction_bins;
      candidates.push_back({x, y, bin, mag, gx, gy});
    }
  }

  const std::size_t pixel_count = disparity.size();
  std::vector<float> directional(
      static_cast<std::size_t>(config.direction_bins) * pixel_count, 0.0F);
  for (const Candidate& candidate : candidates) {
    for (int radius = config.radius_min; radius <= config.radius_max; ++radius) {
      // nearbyint uses round-to-nearest-even under the default floating-point
      // environment, matching numpy.round for half-way cases.
      const int target_x = static_cast<int>(std::nearbyint(
          candidate.x + radius * candidate.direction_x));
      const int target_y = static_cast<int>(std::nearbyint(
          candidate.y + radius * candidate.direction_y));
      if (target_x < 0 || target_x >= width || target_y < 0 ||
          target_y >= height) {
        continue;
      }
      const std::size_t index =
          static_cast<std::size_t>(candidate.bin) * pixel_count +
          static_cast<std::size_t>(target_y) * width + target_x;
      directional[index] += candidate.weight;
    }
  }

  Image<float> coherent(height, width, 0.0F);
  const int half = config.direction_bins / 2;
  for (int bin = 0; bin < half; ++bin) {
    const std::size_t offset_a = static_cast<std::size_t>(bin) * pixel_count;
    const std::size_t offset_b =
        static_cast<std::size_t>(bin + half) * pixel_count;
    for (std::size_t i = 0; i < pixel_count; ++i) {
      coherent[i] = std::max(
          coherent[i], std::min(directional[offset_a + i],
                                directional[offset_b + i]));
    }
  }

  if (config.remove_straight_rope) {
    Result<Image<float>> filtered = RemoveStraightComponents(
        coherent, config.close_kernel, config.rope);
    if (!filtered.ok()) return filtered.status();
    coherent = std::move(filtered).value();
  }

  std::vector<float> positive;
  positive.reserve(coherent.size());
  for (float value : coherent.data()) {
    if (value > 0.0F) positive.push_back(value);
  }
  const double cap = positive.empty()
                         ? 0.0
                         : internal::Percentile(std::move(positive), 99.5);
  output.accumulator = Image<float>(height, width, 0.0F);
  if (cap > 0.0) {
    for (std::size_t i = 0; i < coherent.size(); ++i) {
      output.accumulator[i] = static_cast<float>(
          std::clamp(coherent[i] / (cap + 1e-8), 0.0, 1.0));
    }
  }
  return output;
}

}  // namespace axis_grasp
