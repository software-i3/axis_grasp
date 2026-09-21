#ifndef AXIS_GRASP_CORE_TYPES_H_
#define AXIS_GRASP_CORE_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "axis_grasp/core/image.h"
#include "axis_grasp/core/math_types.h"

namespace axis_grasp {

struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double baseline_m = 0.0;
};

struct FrameInput {
  Image<float> disparity;
  // Current Python labels are polygon ROIs. Adapters rasterize them to 0 for
  // background and 1 for pixels allowed to vote. See docs/data_contracts.md.
  Image<std::uint8_t> labels;
  bool has_labels = false;
  std::int64_t timestamp_ns = 0;
  std::string frame_id = "camera";
  std::string name;
};

struct GraspPose {
  Vec3d position_m;
  Mat3d rotation;
  Quaterniond quaternion_xyzw;
};

struct GraspPoint {
  Vec2i pixel;
  Vec3d position_m;
};

struct GraspArc {
  int source_component = 0;
  std::vector<Vec2d> skeleton_pixels;
  std::vector<GraspPoint> dense_points;
  std::vector<GraspPose> poses;
  double arc_length_m = 0.0;
};

struct StageTimings {
  double voting_ms = 0.0;
  double filtering_ms = 0.0;
  double reprojection_ms = 0.0;
  double grasp_ms = 0.0;
  double total_ms = 0.0;
};

struct PipelineOutput {
  std::string frame_name;
  std::string frame_id;
  std::int64_t timestamp_ns = 0;
  std::vector<GraspArc> arcs;
  StageTimings timings;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_TYPES_H_
