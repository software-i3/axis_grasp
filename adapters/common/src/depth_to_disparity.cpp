#include "axis_grasp/adapters/depth_to_disparity.h"

#include <cmath>
#include <string>

namespace axis_grasp {

bool DepthSampleToDisparity(std::uint16_t raw_mm, double fx,
                            double baseline_m, const DepthBand& band,
                            float* disparity_px) {
  if (disparity_px == nullptr) return false;
  if (raw_mm == 0) return false;  // Hole, not a 1 mm reading.
  const double depth_m = static_cast<double>(raw_mm) / 1000.0;
  if (!std::isfinite(depth_m) || depth_m < band.min_m || depth_m > band.max_m) {
    return false;
  }
  const double disparity = fx * baseline_m / depth_m;
  if (!std::isfinite(disparity)) return false;
  *disparity_px = static_cast<float>(disparity);
  return true;
}

Result<Image<float>> DepthToDisparity(const Image<std::uint16_t>& depth_mm,
                                      const CameraIntrinsics& intrinsics,
                                      const DepthBand& band,
                                      std::size_t* hole_count) {
  if (hole_count != nullptr) *hole_count = 0;
  if (depth_mm.empty()) {
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
  Image<float> disparity(depth_mm.height(), depth_mm.width(), 0.0F);
  std::size_t holes = 0;
  for (std::size_t i = 0; i < depth_mm.size(); ++i) {
    float value = 0.0F;
    if (DepthSampleToDisparity(depth_mm[i], intrinsics.fx,
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

}  // namespace axis_grasp
