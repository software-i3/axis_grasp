#ifndef AXIS_GRASP_CORE_GRASP_PROPOSAL_H_
#define AXIS_GRASP_CORE_GRASP_PROPOSAL_H_

#include <optional>
#include <vector>

#include "axis_grasp/core/config.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/math_types.h"
#include "axis_grasp/core/status.h"
#include "axis_grasp/core/types.h"

namespace axis_grasp {

Image<Vec3d> ReprojectDisparity(const Image<float>& disparity,
                               const CameraIntrinsics& intrinsics);

std::vector<Vec2i> OrderSkeletonPoints(
    const Image<std::uint8_t>& skeleton);
std::vector<Vec2d> ResamplePolyline(const std::vector<Vec2i>& points,
                                    double spacing_pixels);

std::optional<Mat3d> BuildGraspFrame(const Vec3d& tangent,
                                     const Vec3d& surface_normal);

struct PlaneFit {
  Vec3d normal;
  Vec3d inlier_centroid;
};

std::optional<PlaneFit> FitPlaneRansac(
    const std::vector<Vec3d>& points, double distance_threshold,
    int iterations = 50, int min_inliers = 5,
    double min_inlier_fraction = 0.3, std::uint32_t seed = 0);

Vec3d OrientNormalTowardCamera(const Vec3d& normal, const Vec3d& anchor);
Quaterniond RotationToQuaternion(const Mat3d& rotation);

Result<std::vector<GraspArc>> ProposeGrasps(
    const Image<Vec3d>& points, const Image<std::uint8_t>& closed_mask,
    const Image<int>& component_labels, const std::vector<int>& kept_labels,
    const GraspConfig& config);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_GRASP_PROPOSAL_H_
