#ifndef AXIS_GRASP_CORE_CONFIG_H_
#define AXIS_GRASP_CORE_CONFIG_H_

#include <cstdint>

#include "axis_grasp/core/math_types.h"

namespace axis_grasp {

enum class GraspStrategy {
  kCameraNormal,
  kPostSkeletonComponentSvd,
};

struct RopeFilterConfig {
  double percentile = 80.0;
  int min_pixels = 60;
  double elongation_ratio = 9.0;
  double residual_pixels = 2.0;
  double residual_fraction = 0.7;
  // The straightness test is a RANSAC line fit over the component's skeleton
  // rather than a whole-population PCA: closing can bridge a rope into nearby
  // clutter, and a merged fragment must not disqualify an otherwise straight
  // component. min_skeleton_pixels is the matching floor, because a short
  // chord of a curved feature is straight by construction and would otherwise
  // be deleted as if it were rope.
  double ransac_distance_pixels = 2.0;
  int ransac_iterations = 300;
  double ransac_min_inlier_fraction = 0.7;
  int min_skeleton_pixels = 80;
  std::uint32_t ransac_seed = 0;
};

struct VotingConfig {
  int radius_min = 2;
  int radius_max = 14;
  double magnitude_percentile = 70.0;
  int direction_bins = 8;
  bool remove_straight_rope = true;
  int close_kernel = 9;
  int bilateral_diameter = 9;
  double bilateral_sigma_color = 75.0;
  double bilateral_sigma_space = 75.0;
  RopeFilterConfig rope;
};

struct ComponentConfig {
  double accumulator_percentile = 30.0;
  int close_kernel = 9;
  int top_components = 3;
};

struct GraspConfig {
  GraspStrategy strategy = GraspStrategy::kCameraNormal;
  double dense_spacing_pixels = 3.5;
  int edge_margin_pixels = 6;
  double output_spacing_m = 0.005;
  Vec3d camera_approach_axis{0.0, 0.0, -1.0};
  int post_skeleton_dilation_pixels = 2;
  double ransac_distance_m = 0.005;
  int ransac_iterations = 50;
  int ransac_min_inliers = 5;
  double ransac_min_inlier_fraction = 0.3;
  std::uint32_t ransac_seed = 0;
};

struct PipelineConfig {
  VotingConfig voting;
  ComponentConfig components;
  GraspConfig grasp;
  bool enable_roi_crop = true;
  // Negative selects a conservative automatically computed halo.
  int roi_crop_padding = -1;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_CONFIG_H_
