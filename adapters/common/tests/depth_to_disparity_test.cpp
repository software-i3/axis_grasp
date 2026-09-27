#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "axis_grasp/adapters/depth_to_disparity.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/status.h"
#include "axis_grasp/core/types.h"

namespace axis_grasp {
namespace {

// The geometry the pipeline's own reprojection assumes: Z = fx * baseline / d.
double DepthFromDisparity(double disparity_px, const CameraIntrinsics& k) {
  return k.fx * k.baseline_m / disparity_px;
}

CameraIntrinsics TestIntrinsics() {
  CameraIntrinsics intrinsics;
  intrinsics.fx = 610.0;
  intrinsics.fy = 610.0;
  intrinsics.cx = 400.0;
  intrinsics.cy = 300.0;
  intrinsics.baseline_m = 0.06;
  return intrinsics;
}

TEST(DepthSampleToDisparityTest, ConvertsMillimetresToPixelDisparity) {
  const CameraIntrinsics k = TestIntrinsics();
  float disparity = 0.0F;
  ASSERT_TRUE(DepthSampleToDisparity(3000, k.fx, k.baseline_m, DepthBand(),
                                     &disparity));
  EXPECT_FLOAT_EQ(disparity, 12.2F);  // 610 * 0.06 / 3.0
}

TEST(DepthSampleToDisparityTest, RoundTripsThroughThePipelineReprojection) {
  const CameraIntrinsics k = TestIntrinsics();
  for (const double depth_m : {0.05, 0.5, 1.0, 3.0, 12.75, 100.0}) {
    const std::uint16_t raw_mm = static_cast<std::uint16_t>(
        std::llround(depth_m * 1000.0));
    float disparity = 0.0F;
    ASSERT_TRUE(DepthSampleToDisparity(raw_mm, k.fx, k.baseline_m, DepthBand(),
                                       &disparity))
        << "depth " << depth_m;
    // Quantization to whole millimetres bounds the round trip; nothing else
    // should perturb it, because the conversion is the exact inverse.
    const double recovered = DepthFromDisparity(disparity, k);
    EXPECT_NEAR(recovered, raw_mm / 1000.0, 0.001) << "depth " << depth_m;
  }
}

TEST(DepthSampleToDisparityTest, ZeroMillimetresIsAHoleNotTheNearPlane) {
  const CameraIntrinsics k = TestIntrinsics();
  float disparity = -1.0F;
  EXPECT_FALSE(DepthSampleToDisparity(0, k.fx, k.baseline_m, DepthBand(),
                                      &disparity));
  EXPECT_FLOAT_EQ(disparity, -1.0F);  // Untouched on rejection.
}

TEST(DepthSampleToDisparityTest, RejectsSamplesOutsideTheBand) {
  const CameraIntrinsics k = TestIntrinsics();
  DepthBand band;
  band.min_m = 0.25;
  band.max_m = 5.0;
  float disparity = 0.0F;
  // Endpoints are inclusive.
  EXPECT_TRUE(DepthSampleToDisparity(250, k.fx, k.baseline_m, band, &disparity));
  EXPECT_TRUE(DepthSampleToDisparity(5000, k.fx, k.baseline_m, band, &disparity));
  EXPECT_FALSE(
      DepthSampleToDisparity(249, k.fx, k.baseline_m, band, &disparity));
  EXPECT_FALSE(
      DepthSampleToDisparity(5001, k.fx, k.baseline_m, band, &disparity));
}

TEST(DepthSampleToDisparityTest, AcceptsTheWholeRepresentableUint16Range) {
  const CameraIntrinsics k = TestIntrinsics();
  float disparity = 0.0F;
  // 65535 mm is the largest representable sample and is 65.535 m, still inside
  // the 100 m default ceiling: a uint16-millimetre payload cannot reach it, so
  // the default upper bound is only ever active when it is lowered. The lower
  // bound is the one the default band actually enforces, because 1 mm rounds to
  // a near-field disparity that would be a huge spurious vote.
  EXPECT_TRUE(
      DepthSampleToDisparity(65535, k.fx, k.baseline_m, DepthBand(), &disparity));
  EXPECT_FALSE(
      DepthSampleToDisparity(1, k.fx, k.baseline_m, DepthBand(), &disparity));
}

TEST(DepthToDisparityTest, ConvertsEverySampleAndCountsHoles) {
  const CameraIntrinsics k = TestIntrinsics();
  Image<std::uint16_t> depth(2, 3, 0);
  depth(0, 0) = 3000;  // 12.2 px
  depth(0, 1) = 0;     // hole
  depth(0, 2) = 6000;  // 6.1 px
  depth(1, 0) = 1000;  // 36.6 px
  depth(1, 1) = 0;     // hole
  depth(1, 2) = 3000;  // 12.2 px

  std::size_t holes = 0;
  const Result<Image<float>> disparity =
      DepthToDisparity(depth, k, DepthBand(), &holes);
  ASSERT_TRUE(disparity.ok()) << disparity.status().message;
  EXPECT_EQ(disparity.value().height(), 2);
  EXPECT_EQ(disparity.value().width(), 3);
  EXPECT_FLOAT_EQ(disparity.value()(0, 0), 12.2F);
  EXPECT_FLOAT_EQ(disparity.value()(0, 1), 0.0F);
  EXPECT_FLOAT_EQ(disparity.value()(0, 2), 6.1F);
  EXPECT_FLOAT_EQ(disparity.value()(1, 0), 36.6F);
  EXPECT_FLOAT_EQ(disparity.value()(1, 1), 0.0F);
  EXPECT_FLOAT_EQ(disparity.value()(1, 2), 12.2F);
  EXPECT_EQ(holes, 2U);
}

TEST(DepthToDisparityTest, NeverEmitsNaNOrInfinity) {
  const CameraIntrinsics k = TestIntrinsics();
  // 1 mm is below the default 0.05 m floor and 65535 mm above a narrowed
  // ceiling, so both become holes; neither may become an enormous or infinite
  // disparity.
  DepthBand band;
  band.min_m = 0.05;
  band.max_m = 10.0;
  Image<std::uint16_t> depth(1, 2, 0);
  depth(0, 0) = 1;
  depth(0, 1) = 65535;
  const Result<Image<float>> disparity = DepthToDisparity(depth, k, band);
  ASSERT_TRUE(disparity.ok()) << disparity.status().message;
  for (std::size_t i = 0; i < disparity.value().size(); ++i) {
    EXPECT_TRUE(std::isfinite(disparity.value()[i]));
    EXPECT_FLOAT_EQ(disparity.value()[i], 0.0F);
  }
}

TEST(DepthToDisparityTest, RejectsUnusableIntrinsics) {
  Image<std::uint16_t> depth(1, 1, 3000);
  CameraIntrinsics no_fx = TestIntrinsics();
  no_fx.fx = 0.0;
  EXPECT_EQ(DepthToDisparity(depth, no_fx, DepthBand()).status().code,
            ErrorCode::kInvalidArgument);

  CameraIntrinsics no_baseline = TestIntrinsics();
  no_baseline.baseline_m = 0.0;
  EXPECT_EQ(DepthToDisparity(depth, no_baseline, DepthBand()).status().code,
            ErrorCode::kInvalidArgument);

  CameraIntrinsics negative_fx = TestIntrinsics();
  negative_fx.fx = -610.0;
  EXPECT_EQ(DepthToDisparity(depth, negative_fx, DepthBand()).status().code,
            ErrorCode::kInvalidArgument);
}

TEST(DepthToDisparityTest, RejectsEmptyImageAndInvertedBand) {
  const CameraIntrinsics k = TestIntrinsics();
  EXPECT_EQ(
      DepthToDisparity(Image<std::uint16_t>(), k, DepthBand()).status().code,
      ErrorCode::kInvalidArgument);

  Image<std::uint16_t> depth(1, 1, 3000);
  DepthBand inverted;
  inverted.min_m = 5.0;
  inverted.max_m = 0.25;
  EXPECT_EQ(DepthToDisparity(depth, k, inverted).status().code,
            ErrorCode::kInvalidArgument);
}

TEST(DepthToDisparityTest, HoleCountIsOptional) {
  const CameraIntrinsics k = TestIntrinsics();
  Image<std::uint16_t> depth(1, 1, 0);
  EXPECT_TRUE(DepthToDisparity(depth, k, DepthBand()).ok());
}

}  // namespace
}  // namespace axis_grasp
