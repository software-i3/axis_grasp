#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
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

  // last_x shortens the bar, which changes the chain's length and with it
  // whether the final resampling target rounds above that length.
  explicit StraightComponent(int last_x = 38) {
    for (int y = 0; y < points.height(); ++y) {
      for (int x = 0; x < points.width(); ++x) {
        points(y, x) = {x * 0.002, y * 0.002, 1.0};
      }
    }
    for (int y = 15; y <= 19; ++y) {
      for (int x = 6; x <= last_x; ++x) {
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
  // protocol reprojection, expressed in metres, in the standard optical frame:
  // x to the right, y downward, z forward, so z is the depth and is positive.
  const Image<Vec3d> points = ReprojectDisparity(disparity, intrinsics);
  EXPECT_NEAR(points(0, 0).x, 0.0013592714563424996, 1e-12);
  EXPECT_NEAR(points(0, 0).y, -0.015389714079654277, 1e-12);
  EXPECT_NEAR(points(0, 0).z, 0.32292505794717263, 1e-12);
}

// The published convention in one place: x right, y down, z forward. A uniform
// disparity is a fronto-parallel plane at Z = fx * baseline / d, so the sign and
// the direction of every axis is checkable without building a scene. This is the
// test that fails if the reprojection is ever rotated back.
TEST(GraspProposalTest, ReprojectsIntoTheStandardOpticalFrame) {
  Image<float> disparity(3, 3, 100.0F);
  CameraIntrinsics intrinsics;
  intrinsics.fx = 480.0;
  intrinsics.fy = 480.0;
  intrinsics.cx = 1.0;  // the centre column of a 3x3 image
  intrinsics.cy = 1.0;  // the centre row
  intrinsics.baseline_m = 0.1;
  const Image<Vec3d> points = ReprojectDisparity(disparity, intrinsics);

  const double depth = intrinsics.fx * intrinsics.baseline_m / 100.0;
  ASSERT_GT(depth, 0.0);
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      EXPECT_NEAR(points(y, x).z, depth, 1e-12);
    }
  }
  // The principal pixel sits on the optical axis.
  EXPECT_NEAR(points(1, 1).x, 0.0, 1e-12);
  EXPECT_NEAR(points(1, 1).y, 0.0, 1e-12);
  // x grows to the right, y grows downward.
  EXPECT_LT(points(1, 0).x, points(1, 2).x);
  EXPECT_LT(points(0, 1).y, points(2, 1).y);
  EXPECT_NEAR(points(1, 2).x, depth * (2 - intrinsics.cx) / intrinsics.fx,
              1e-12);
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
  EXPECT_NEAR(approach.z, 1.0, 1e-10);
}

TEST(GraspProposalTest, KeepsTheChainEndpointsOnTheDensePoints) {
  // The chain is resampled at output_spacing_m, i.e. at targets of
  // total * index / (count - 1), so the final target can land one ulp above the
  // chain's cumulative length and send lower_bound past the end of the array.
  // Both endpoints must still be the chain's own first and last dense points:
  // an index one past the end reads memory the algorithm never wrote, so the
  // end pose would otherwise follow the allocator rather than the geometry.
  // The two spacings below are the ones that make the final target round above
  // the chain's length for this bar; the same bar at other spacings does not,
  // which is exactly why the defect survived unnoticed.
  for (const auto& [dense, output] :
       {std::pair<double, double>{2.0, 0.007}, {1.5, 0.015},
        {2.0, 0.015}}) {
    SCOPED_TRACE("dense_spacing_pixels " + std::to_string(dense) +
                 " output_spacing_m " + std::to_string(output));
    StraightComponent input(30);
    GraspConfig config;
    config.strategy = GraspStrategy::kCameraNormal;
    config.edge_margin_pixels = 0;
    config.dense_spacing_pixels = dense;
    config.output_spacing_m = output;
    Result<std::vector<GraspArc>> result = ProposeGrasps(
        input.points, input.mask, input.labels, {1}, config);
    ASSERT_TRUE(result.ok()) << result.status().message;
    const GraspArc& arc = result.value()[0];
    ASSERT_GE(arc.poses.size(), 2U);
    ASSERT_FALSE(arc.dense_points.empty());
    const Vec3d& first = arc.dense_points.front().position_m;
    const Vec3d& last = arc.dense_points.back().position_m;
    // EXPECT_EQ rather than EXPECT_DOUBLE_EQ: the endpoints are copied out of
    // the chain rather than interpolated into it, so they must agree bit for
    // bit. Four-ulp tolerance is wide enough to swallow the difference between
    // the chain's last point and a value read one past its end.
    EXPECT_EQ(arc.poses.front().position_m.x, first.x) << "first pose x";
    EXPECT_EQ(arc.poses.front().position_m.y, first.y) << "first pose y";
    EXPECT_EQ(arc.poses.front().position_m.z, first.z) << "first pose z";
    EXPECT_EQ(arc.poses.back().position_m.x, last.x) << "last pose x";
    EXPECT_EQ(arc.poses.back().position_m.y, last.y) << "last pose y";
    EXPECT_EQ(arc.poses.back().position_m.z, last.z) << "last pose z";
  }
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
  EXPECT_NEAR(approach.z, 1.0, 1e-8);
}

TEST(GraspProposalTest, PlaneRansacRejectsOneOutlier) {
  std::vector<Vec3d> points;
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 5; ++x) points.push_back({x * 0.01, y * 0.01, 1.0});
  }
  points.push_back({0.0, 0.0, -2.0});
  std::optional<PlaneFit> plane =
      FitPlaneRansac(points, 1e-4, 100, 5, 0.3, 0);
  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(std::abs(plane->normal.z), 1.0, 1e-8);
}

}  // namespace
}  // namespace axis_grasp
