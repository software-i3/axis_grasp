#include "axis_grasp/adapters/depth_to_disparity.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace axis_grasp {
namespace {

// The one conversion core, in metres. Both payload shapes funnel through here,
// so the hole/band/finiteness policy is written once and cannot drift between
// them: a millimetre image and a metre image differ only in the unit their
// samples arrive in.
//
// Returns false -- leaving *disparity_px untouched -- for a hole or an
// out-of-band sample. A depth of 0 is a missing reading, not a 1 mm one;
// converting it would emit a huge disparity and a spurious near-field vote.
bool MetresToDisparity(double depth_m, double fx, double baseline_m,
                       const DepthBand& band, float* disparity_px) {
  if (disparity_px == nullptr) return false;
  if (!std::isfinite(depth_m) || depth_m <= 0.0) return false;
  if (depth_m < band.min_m || depth_m > band.max_m) return false;
  const double disparity = fx * baseline_m / depth_m;
  if (!std::isfinite(disparity)) return false;
  *disparity_px = static_cast<float>(disparity);
  return true;
}

// How each payload's sample spells metres. Overloads rather than a runtime
// branch, so the image's element type picks the unit at compile time and no
// caller can get it wrong by passing the wrong enum.
double SampleToMetres(std::uint16_t raw_mm) {
  if (raw_mm == 0) return 0.0;  // Hole, not a 1 mm reading.
  return static_cast<double>(raw_mm) / 1000.0;
}

// A float metric payload is metres by construction -- the ROS depth-image
// convention, and what the live explore3d depth carries. 0, negative and
// non-finite samples are holes, handled by MetresToDisparity.
double SampleToMetres(float depth_m) { return static_cast<double>(depth_m); }

// Shared body of both public overloads: validate the arguments, then convert
// every sample and count the holes.
template <typename Sample>
Result<Image<float>> ConvertDepth(const Image<Sample>& depth,
                                  const CameraIntrinsics& intrinsics,
                                  const DepthBand& band,
                                  std::size_t* hole_count) {
  if (hole_count != nullptr) *hole_count = 0;
  if (depth.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "depth image is empty");
  }
  if (!std::isfinite(intrinsics.fx) || intrinsics.fx <= 0.0 ||
      !std::isfinite(intrinsics.baseline_m) || intrinsics.baseline_m <= 0.0) {
    return Status::Error(
        ErrorCode::kInvalidArgument,
        "depth conversion needs finite fx > 0 and baseline_m > 0, got fx=" +
            std::to_string(intrinsics.fx) +
            " baseline_m=" + std::to_string(intrinsics.baseline_m));
  }
  if (!std::isfinite(band.min_m) || !std::isfinite(band.max_m) ||
      band.min_m > band.max_m) {
    return Status::Error(
        ErrorCode::kInvalidArgument,
        "depth band is not a finite ascending interval, got min_m=" +
            std::to_string(band.min_m) + " max_m=" + std::to_string(band.max_m));
  }
  Image<float> disparity(depth.height(), depth.width(), 0.0F);
  std::size_t holes = 0;
  for (std::size_t i = 0; i < depth.size(); ++i) {
    float value = 0.0F;
    if (MetresToDisparity(SampleToMetres(depth[i]), intrinsics.fx,
                          intrinsics.baseline_m, band, &value)) {
      disparity[i] = value;
    } else {
      disparity[i] = 0.0F;
      ++holes;
    }
  }
  if (hole_count != nullptr) *hole_count = holes;
  return disparity;
}

}  // namespace

bool DepthSampleToDisparity(std::uint16_t raw_mm, double fx,
                            double baseline_m, const DepthBand& band,
                            float* disparity_px) {
  if (raw_mm == 0) return false;  // Hole, not a 1 mm reading.
  return MetresToDisparity(SampleToMetres(raw_mm), fx, baseline_m, band,
                           disparity_px);
}

Result<Image<float>> DepthToDisparity(const Image<std::uint16_t>& depth_mm,
                                      const CameraIntrinsics& intrinsics,
                                      const DepthBand& band,
                                      std::size_t* hole_count) {
  return ConvertDepth(depth_mm, intrinsics, band, hole_count);
}

Result<Image<float>> DepthToDisparity(const Image<float>& depth_m,
                                      const CameraIntrinsics& intrinsics,
                                      const DepthBand& band,
                                      std::size_t* hole_count) {
  return ConvertDepth(depth_m, intrinsics, band, hole_count);
}

}  // namespace axis_grasp
