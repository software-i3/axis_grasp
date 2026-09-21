#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "axis_grasp/core/config.h"
#include "axis_grasp/core/grasp_proposal.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/math_types.h"

namespace axis_grasp {
namespace {

struct StraightComponent {
  Image<Vec3d> points{35, 45};
  Image<std::uint8_t> mask{35, 45, 0};
  Image<int> labels{35, 45, 0};

  StraightComponent() {
    for (int y = 0; y < points.height(); ++y) {
      for (int x = 0; x < points.width(); ++x) {
        points(y, x) = {x * 0.002, y * 0.002, -1.0};
      }
    }
    for (int y = 15; y <= 19; ++y) {
      for (int x = 6; x <= 38; ++x) {
        mask(y, x) = 1;
        labels(y, x) = 1;
      }
    }
  }
};

TEST(GraspProposalTest, ReprojectsUsingConfiguredCalibration) {
  Image<float> disparity(1, 1, 142.5F);
  CameraIntrinsics intrinsics;
  intrinsics.fx = 479.2662;
  intrinsics.fy = 478.80985000920623;
  intrinsics.cx = 397.98265 - 400.0;
  intrinsics.cy = 322.8405 - 300.0;
  // Norm of camera 1's translation in calibration_underwater.json. Production
  // obtains this value from calibration; 0.1 m is only a capture-side
  // prototype override used by the supplied PLY validation sample.
  intrinsics.baseline_m = 0.0960151597535401;

  // Shifted principal coordinates make local pixel (0, 0) equivalent to the
  // captured frame's pixel (400, 300). These expected values are the analytic
  // protocol reprojection, expressed in metres.
  const Image<Vec3d> points = ReprojectDisparity(disparity, intrinsics);
  EXPECT_NEAR(points(0, 0).x, 0.0013592714563424996, 1e-12);
  EXPECT_NEAR(points(0, 0).y, 0.015389714079654277, 1e-12);
  EXPECT_NEAR(points(0, 0).z, -0.32292505794717263, 1e-12);
}

TEST(GraspProposalTest, OrdersEqualDistanceBranchesInRowMajorOrder) {
  Image<std::uint8_t> skeleton(3, 3, 0);
  skeleton(0, 1) = 1;
  skeleton(1, 1) = 1;
  skeleton(2, 0) = 1;
  skeleton(2, 1) = 1;
  skeleton(2, 2) = 1;

  const std::vector<Vec2i> ordered = OrderSkeletonPoints(skeleton);
  const std::vector<Vec2i> expected{
      {1, 0}, {1, 1}, {1, 2}, {0, 2}, {2, 2}};
  ASSERT_EQ(ordered.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(ordered[i].x, expected[i].x) << "at index " << i;
    EXPECT_EQ(ordered[i].y, expected[i].y) << "at index " << i;
  }
}

TEST(GraspProposalTest, CameraNormalProducesCameraForwardApproach) {
  StraightComponent input;
  GraspConfig config;
  config.strategy = GraspStrategy::kCameraNormal;
  config.edge_margin_pixels = 0;
  config.dense_spacing_pixels = 2.0;
  config.output_spacing_m = 0.01;
  Result<std::vector<GraspArc>> result = ProposeGrasps(
      input.points, input.mask, input.labels, {1}, config);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_FALSE(result.value()[0].poses.empty());
  const Vec3d approach = result.value()[0].poses[0].rotation.column(0);
  EXPECT_NEAR(approach.x, 0.0, 1e-10);
  EXPECT_NEAR(approach.y, 0.0, 1e-10);
  EXPECT_NEAR(approach.z, -1.0, 1e-10);
}

TEST(GraspProposalTest, PostSkeletonSvdProducesPlaneNormal) {
  StraightComponent input;
  GraspConfig config;
  config.strategy = GraspStrategy::kPostSkeletonComponentSvd;
  config.edge_margin_pixels = 0;
  config.dense_spacing_pixels = 2.0;
  config.output_spacing_m = 0.01;
  config.ransac_distance_m = 1e-5;
  Result<std::vector<GraspArc>> result = ProposeGrasps(
      input.points, input.mask, input.labels, {1}, config);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_FALSE(result.value()[0].poses.empty());
  const Vec3d approach = result.value()[0].poses[0].rotation.column(0);
  EXPECT_NEAR(approach.x, 0.0, 1e-8);
  EXPECT_NEAR(approach.y, 0.0, 1e-8);
  EXPECT_NEAR(approach.z, -1.0, 1e-8);
}

TEST(GraspProposalTest, PlaneRansacRejectsOneOutlier) {
  std::vector<Vec3d> points;
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 5; ++x) points.push_back({x * 0.01, y * 0.01, -1.0});
  }
  points.push_back({0.0, 0.0, 2.0});
  std::optional<PlaneFit> plane =
      FitPlaneRansac(points, 1e-4, 100, 5, 0.3, 0);
  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(std::abs(plane->normal.z), 1.0, 1e-8);
}

}  // namespace
}  // namespace axis_grasp
