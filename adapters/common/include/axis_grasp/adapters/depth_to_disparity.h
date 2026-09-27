#ifndef AXIS_GRASP_ADAPTERS_DEPTH_TO_DISPARITY_H_
#define AXIS_GRASP_ADAPTERS_DEPTH_TO_DISPARITY_H_

#include <cstddef>
#include <cstdint>

#include "axis_grasp/core/image.h"
#include "axis_grasp/core/status.h"
#include "axis_grasp/core/types.h"

namespace axis_grasp {

// Depth -> disparity conversion shared by every adapter that has to feed the
// pipeline a range image from a depth sensor. ROS-free so the dataset and ROS
// paths can be tested against the same code.
//
// This is an exact reparametrization, not an approximation: reprojection is
// standard stereo (Z = -fx * baseline / d), so d = fx * baseline / Z returns the
// original depth exactly. The pipeline's voting and grasp-proposal stages see
// the geometry they already expect; only the sensor that produced it differs.

// Depth range accepted from the sensor, in metres. Samples outside it are
// treated as holes. Mirrors the defaults in mine_centering's calibration.
//
// The floor is what the defaults actually enforce: a near-field sample converts
// to a huge disparity, which would vote as a large object. The ceiling rejects
// far readings at the disparity noise floor, but a uint16-millimetre payload
// tops out at 65.535 m, so 100.0 is unreachable until it is lowered.
struct DepthBand {
  double min_m = 0.05;
  double max_m = 100.0;
};

// Convert one 16-bit millimetre depth sample to pixel disparity:
// d = fx * baseline_m / (raw_mm / 1000).
//
// Returns false — leaving *disparity_px untouched — for a hole or an
// out-of-band sample, where a hole is raw_mm == 0, a non-finite depth, or a
// depth outside `band` (endpoints inclusive). A depth of 0 mm is a missing
// reading, not a 1 mm one; converting it would emit a huge disparity and a
// spurious near-field vote.
//
// Never emits NaN or infinity: the bilateral filter and Sobel inside the voting
// stage run on this image before any sanitizing.
bool DepthSampleToDisparity(std::uint16_t raw_mm, double fx,
                            double baseline_m, const DepthBand& band,
                            float* disparity_px);

// Convert a full 16UC1 millimetre depth image to pixel disparity. Invalid
// samples become 0.0F, which is how an invalid stereo pixel already reads.
//
// When `hole_count` is non-null it receives the number of samples that became
// 0.0F, so a caller can warn about a frame that is mostly holes.
//
// Returns kInvalidArgument when the image is empty, when fx <= 0 or
// baseline_m <= 0 (both of which the pipeline itself requires), or when the
// band is inverted or holds a non-finite bound.
Result<Image<float>> DepthToDisparity(const Image<std::uint16_t>& depth_mm,
                                      const CameraIntrinsics& intrinsics,
                                      const DepthBand& band,
                                      std::size_t* hole_count = nullptr);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_DEPTH_TO_DISPARITY_H_
