#include "axis_grasp/core/pipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include "axis_grasp/core/axis_voting.h"
#include "axis_grasp/core/component_filter.h"
#include "axis_grasp/core/grasp_proposal.h"

namespace axis_grasp {
namespace {

using Clock = std::chrono::steady_clock;

double Milliseconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

Image<float> CropFloat(const Image<float>& source, int x0, int y0,
                       int width, int height) {
  Image<float> result(height, width, 0.0F);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) result(y, x) = source(y + y0, x + x0);
  }
  return result;
}

Image<std::uint8_t> CropByte(const Image<std::uint8_t>& source, int x0,
                             int y0, int width, int height) {
  Image<std::uint8_t> result(height, width, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) result(y, x) = source(y + y0, x + x0);
  }
  return result;
}

}  // namespace

Pipeline::Pipeline(PipelineConfig config, CameraIntrinsics intrinsics,
                   Logger* logger)
    : config_(std::move(config)),
      intrinsics_(intrinsics),
      logger_(logger) {}

Status ValidateConfig(const PipelineConfig& config,
                      const CameraIntrinsics& intrinsics) {
  if (!(intrinsics.fx > 0.0) || !(intrinsics.fy > 0.0) ||
      !(intrinsics.baseline_m > 0.0)) {
    return Status::Error(
        ErrorCode::kInvalidArgument,
        "fx, fy, and stereo baseline must all be positive");
  }
  if (config.grasp.dense_spacing_pixels <= 0.0 ||
      config.grasp.output_spacing_m <= 0.0 ||
      config.grasp.edge_margin_pixels < 0 ||
      config.grasp.post_skeleton_dilation_pixels < 0 ||
      config.grasp.ransac_distance_m <= 0.0 ||
      config.grasp.ransac_iterations < 1 ||
      config.grasp.ransac_min_inliers < 3 ||
      config.grasp.ransac_min_inlier_fraction < 0.0 ||
      config.grasp.ransac_min_inlier_fraction > 1.0) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Invalid grasp proposal configuration");
  }
  if (config.roi_crop_padding < -1) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "roi_crop_padding must be -1 or non-negative");
  }
  return Status::Ok();
}

Result<PipelineOutput> Pipeline::Process(const FrameInput& frame) const {
  const Status config_status = ValidateConfig(config_, intrinsics_);
  if (!config_status.ok()) return config_status;
  if (frame.disparity.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Input disparity frame is empty");
  }
  if (frame.has_labels &&
      (frame.labels.height() != frame.disparity.height() ||
       frame.labels.width() != frame.disparity.width())) {
    return Status::Error(ErrorCode::kShapeMismatch,
                         "Label mask and disparity map shapes differ");
  }

  // The ROI crop is deliberately applied around the complete pipeline rather
  // than just around voting. The conservative halo contains every bilateral
  // and Sobel input, every vote target, morphology support, and grasp band.
  // Shifting the principal point preserves the exact camera reprojection.
  if (config_.enable_roi_crop && frame.has_labels) {
    int min_x = frame.labels.width();
    int min_y = frame.labels.height();
    int max_x = -1;
    int max_y = -1;
    for (int y = 0; y < frame.labels.height(); ++y) {
      for (int x = 0; x < frame.labels.width(); ++x) {
        if (frame.labels(y, x) == 0) continue;
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
      }
    }
    if (max_x >= min_x && max_y >= min_y) {
      const int automatic_padding =
          config_.voting.radius_max + config_.voting.bilateral_diameter / 2 +
          1 + 2 * std::max(config_.voting.close_kernel,
                           config_.components.close_kernel) +
          config_.grasp.post_skeleton_dilation_pixels +
          config_.grasp.edge_margin_pixels + 2;
      const int padding = config_.roi_crop_padding >= 0
                              ? config_.roi_crop_padding
                              : automatic_padding;
      const int x0 = std::max(0, min_x - padding);
      const int y0 = std::max(0, min_y - padding);
      const int x1 = std::min(frame.disparity.width(), max_x + padding + 1);
      const int y1 = std::min(frame.disparity.height(), max_y + padding + 1);
      if (x0 != 0 || y0 != 0 || x1 != frame.disparity.width() ||
          y1 != frame.disparity.height()) {
        FrameInput cropped = frame;
        cropped.disparity = CropFloat(frame.disparity, x0, y0, x1 - x0,
                                      y1 - y0);
        cropped.labels = CropByte(frame.labels, x0, y0, x1 - x0, y1 - y0);
        PipelineConfig cropped_config = config_;
        cropped_config.enable_roi_crop = false;
        CameraIntrinsics cropped_intrinsics = intrinsics_;
        cropped_intrinsics.cx -= x0;
        cropped_intrinsics.cy -= y0;
        Pipeline cropped_pipeline(cropped_config, cropped_intrinsics, logger_);
        Result<PipelineOutput> result = cropped_pipeline.Process(cropped);
        if (!result.ok()) return result.status();
        for (GraspArc& arc : result.value().arcs) {
          for (Vec2d& pixel : arc.skeleton_pixels) {
            pixel.x += x0;
            pixel.y += y0;
          }
          for (GraspPoint& point : arc.dense_points) {
            point.pixel.x += x0;
            point.pixel.y += y0;
          }
        }
        return result;
      }
    }
  }
  Logger* active_logger = logger_ == nullptr ? &null_logger_ : logger_;
  const Clock::time_point total_start = Clock::now();

  // Match np.nan_to_num(...).astype(np.float32): NaN becomes zero and signed
  // infinities become the corresponding finite float extrema.
  Image<float> disparity = frame.disparity;
  for (float& value : disparity.data()) {
    if (std::isnan(value)) {
      value = 0.0F;
    } else if (std::isinf(value)) {
      value = std::signbit(value) ? -std::numeric_limits<float>::max()
                                  : std::numeric_limits<float>::max();
    }
  }

  const Clock::time_point voting_start = Clock::now();
  const Image<std::uint8_t>* roi = frame.has_labels ? &frame.labels : nullptr;
  Result<VotingOutput> voting =
      AccumulateDirectionalVotes(disparity, roi, config_.voting);
  if (!voting.ok()) return voting.status();
  const Clock::time_point voting_end = Clock::now();

  const Clock::time_point filtering_start = Clock::now();
  Result<ComponentOutput> components =
      FilterAccumulator(voting.value().accumulator, config_.components);
  if (!components.ok()) return components.status();
  const Clock::time_point filtering_end = Clock::now();

  const Clock::time_point reprojection_start = Clock::now();
  const Image<Vec3d> points = ReprojectDisparity(disparity, intrinsics_);
  const Clock::time_point reprojection_end = Clock::now();

  const Clock::time_point grasp_start = Clock::now();
  Result<std::vector<GraspArc>> arcs = ProposeGrasps(
      points, components.value().closed_mask, components.value().labels,
      components.value().kept_labels, config_.grasp);
  if (!arcs.ok()) return arcs.status();
  const Clock::time_point grasp_end = Clock::now();

  PipelineOutput output;
  output.frame_name = frame.name;
  output.frame_id = frame.frame_id;
  output.timestamp_ns = frame.timestamp_ns;
  output.arcs = std::move(arcs).value();
  output.timings.voting_ms = Milliseconds(voting_start, voting_end);
  output.timings.filtering_ms = Milliseconds(filtering_start, filtering_end);
  output.timings.reprojection_ms =
      Milliseconds(reprojection_start, reprojection_end);
  output.timings.grasp_ms = Milliseconds(grasp_start, grasp_end);
  output.timings.total_ms = Milliseconds(total_start, grasp_end);

  std::ostringstream message;
  message << output.frame_name << " timings_ms: voting="
          << output.timings.voting_ms
          << " filtering=" << output.timings.filtering_ms
          << " reprojection=" << output.timings.reprojection_ms
          << " grasp=" << output.timings.grasp_ms
          << " total=" << output.timings.total_ms;
  active_logger->Log(LogLevel::kInfo, message.str());
  return output;
}

}  // namespace axis_grasp
