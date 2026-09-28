#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
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

// The metres overload is what a float metric payload uses -- the live explore3d
// depth publishes CV_32FC1 metres rather than the bx_msgs uint16 millimetres. It
// must reach the same conversion, not a parallel one that can drift.

TEST(MetresDepthToDisparityTest, MatchesTheMillimetrePathSampleForSample) {
  const CameraIntrinsics k = TestIntrinsics();
  Image<std::uint16_t> mm(2, 3, 0);
  mm(0, 0) = 3000;
  mm(0, 1) = 0;  // hole
  mm(0, 2) = 6000;
  mm(1, 0) = 1000;
  mm(1, 1) = 0;  // hole
  mm(1, 2) = 3000;

  Image<float> metres(2, 3, 0.0F);
  metres(0, 0) = 3.0F;
  metres(0, 1) = 0.0F;  // hole
  metres(0, 2) = 6.0F;
  metres(1, 0) = 1.0F;
  metres(1, 1) = 0.0F;  // hole
  metres(1, 2) = 3.0F;

  std::size_t mm_holes = 0;
  std::size_t m_holes = 0;
  const Result<Image<float>> from_mm =
      DepthToDisparity(mm, k, DepthBand(), &mm_holes);
  const Result<Image<float>> from_m = DepthToDisparity(metres, k, DepthBand(),
                                                       &m_holes);
  ASSERT_TRUE(from_mm.ok()) << from_mm.status().message;
  ASSERT_TRUE(from_m.ok()) << from_m.status().message;
  ASSERT_EQ(from_m.value().size(), from_mm.value().size());
  EXPECT_EQ(m_holes, mm_holes);
  // Exact, not near: both paths hand the same double depth to the same
  // conversion, so nothing about the unit change may perturb the result.
  for (std::size_t i = 0; i < from_m.value().size(); ++i) {
    EXPECT_FLOAT_EQ(from_m.value()[i], from_mm.value()[i]) << "sample " << i;
  }
}

TEST(MetresDepthToDisparityTest, RoundTripsThroughThePipelineReprojection) {
  const CameraIntrinsics k = TestIntrinsics();
  for (const double depth_m : {0.05, 0.5, 1.0, 3.0, 12.75, 100.0}) {
    Image<float> depth(1, 1, static_cast<float>(depth_m));
    const Result<Image<float>> disparity = DepthToDisparity(depth, k, DepthBand());
    ASSERT_TRUE(disparity.ok()) << "depth " << depth_m;
    // No quantization stage on this path, so the round trip is exact to float
    // precision rather than to a millimetre.
    const double recovered = DepthFromDisparity(disparity.value()[0], k);
    EXPECT_NEAR(recovered, depth_m, 1e-4 * depth_m) << "depth " << depth_m;
  }
}

TEST(MetresDepthToDisparityTest, TreatsNonPositiveAndNonFiniteAsHoles) {
  const CameraIntrinsics k = TestIntrinsics();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  Image<float> depth(1, 5, 0.0F);
  depth(0, 0) = 0.0F;   // no reading
  depth(0, 1) = -1.0F;  // behind the camera
  depth(0, 2) = nan;
  depth(0, 3) = inf;
  depth(0, 4) = 3.0F;  // the only live sample

  std::size_t holes = 0;
  const Result<Image<float>> disparity =
      DepthToDisparity(depth, k, DepthBand(), &holes);
  ASSERT_TRUE(disparity.ok()) << disparity.status().message;
  for (int x = 0; x < 4; ++x) {
    // A negative depth would otherwise convert to a negative disparity and a
    // horizon behind the camera; the pipeline's filter and Sobel see this image
    // before anything scrubs non-finite values.
    EXPECT_TRUE(std::isfinite(disparity.value()(0, x)));
    EXPECT_FLOAT_EQ(disparity.value()(0, x), 0.0F) << "sample " << x;
  }
  EXPECT_FLOAT_EQ(disparity.value()(0, 4), 12.2F);
  EXPECT_EQ(holes, 4U);
}

TEST(MetresDepthToDisparityTest, RejectsSamplesOutsideTheBand) {
  const CameraIntrinsics k = TestIntrinsics();
  DepthBand band;
  band.min_m = 0.25;
  band.max_m = 5.0;
  Image<float> depth(1, 4, 0.0F);
  depth(0, 0) = 0.25F;  // endpoints are inclusive
  depth(0, 1) = 5.0F;
  depth(0, 2) = 0.249F;
  depth(0, 3) = 5.001F;

  std::size_t holes = 0;
  const Result<Image<float>> disparity =
      DepthToDisparity(depth, k, band, &holes);
  ASSERT_TRUE(disparity.ok()) << disparity.status().message;
  EXPECT_GT(disparity.value()(0, 0), 0.0F);
  EXPECT_GT(disparity.value()(0, 1), 0.0F);
  EXPECT_FLOAT_EQ(disparity.value()(0, 2), 0.0F);
  EXPECT_FLOAT_EQ(disparity.value()(0, 3), 0.0F);
  EXPECT_EQ(holes, 2U);
}

TEST(MetresDepthToDisparityTest, RejectsUnusableIntrinsicsAndEmptyImage) {
  Image<float> depth(1, 1, 3.0F);
  CameraIntrinsics no_fx = TestIntrinsics();
  no_fx.fx = 0.0;
  EXPECT_EQ(DepthToDisparity(depth, no_fx, DepthBand()).status().code,
            ErrorCode::kInvalidArgument);

  CameraIntrinsics no_baseline = TestIntrinsics();
  no_baseline.baseline_m = 0.0;
  EXPECT_EQ(DepthToDisparity(depth, no_baseline, DepthBand()).status().code,
            ErrorCode::kInvalidArgument);

  // An unusable band must be caught on this path too, not fall through to a
  // loop that quietly rejects every sample.
  DepthBand inverted;
  inverted.min_m = 5.0;
  inverted.max_m = 0.25;
  EXPECT_EQ(DepthToDisparity(depth, TestIntrinsics(), inverted).status().code,
            ErrorCode::kInvalidArgument);

  EXPECT_EQ(DepthToDisparity(Image<float>(), TestIntrinsics(), DepthBand())
                .status()
                .code,
            ErrorCode::kInvalidArgument);
}

}  // namespace
}  // namespace axis_grasp
