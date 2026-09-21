#include "axis_grasp/adapters/adapter_utils.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

namespace axis_grasp {
namespace {

std::optional<double> NumberAfterKey(const std::string& text,
                                     const std::string& key,
                                     std::size_t start = 0) {
  const std::string quoted = "\"" + key + "\"";
  const std::size_t key_position = text.find(quoted, start);
  if (key_position == std::string::npos) return std::nullopt;
  const std::size_t colon = text.find(':', key_position + quoted.size());
  if (colon == std::string::npos) return std::nullopt;
  const char* number_start = text.c_str() + colon + 1;
  char* number_end = nullptr;
  const double value = std::strtod(number_start, &number_end);
  if (number_end == number_start) return std::nullopt;
  return value;
}

std::optional<double> NestedVal(const std::string& text,
                                const std::string& key) {
  const std::string quoted = "\"" + key + "\"";
  const std::size_t key_position = text.find(quoted);
  if (key_position == std::string::npos) return std::nullopt;
  return NumberAfterKey(text, "val", key_position + quoted.size());
}

}  // namespace

void StderrLogger::Log(LogLevel level, const std::string& message) {
  const char* prefix = "INFO";
  if (level == LogLevel::kDebug) prefix = "DEBUG";
  if (level == LogLevel::kWarning) prefix = "WARN";
  if (level == LogLevel::kError) prefix = "ERROR";
  std::cerr << '[' << prefix << "] " << message << '\n';
}

Result<CameraIntrinsics> LoadCalibrationJson(
    const std::filesystem::path& path, int capture_width, int capture_height,
    int native_width, int native_height) {
  std::ifstream stream(path);
  if (!stream) {
    return Status::Error(ErrorCode::kIo,
                         "Cannot open calibration file: " + path.string());
  }
  const std::string json((std::istreambuf_iterator<char>(stream)),
                         std::istreambuf_iterator<char>());
  const std::optional<double> focal = NestedVal(json, "f");
  const std::optional<double> aspect_ratio = NestedVal(json, "ar");
  const std::optional<double> cx = NestedVal(json, "cx");
  const std::optional<double> cy = NestedVal(json, "cy");
  std::size_t first_translation = json.find("\"translation\"");
  std::size_t second_translation = first_translation == std::string::npos
                                       ? std::string::npos
                                       : json.find("\"translation\"",
                                                   first_translation + 1);
  if (!focal || !aspect_ratio || !cx || !cy ||
      second_translation == std::string::npos) {
    return Status::Error(
        ErrorCode::kUnsupportedFormat,
        "Calibration JSON does not match the two-camera libCalib layout");
  }
  const std::optional<double> tx = NumberAfterKey(json, "x", second_translation);
  const std::optional<double> ty = NumberAfterKey(json, "y", second_translation);
  const std::optional<double> tz = NumberAfterKey(json, "z", second_translation);
  if (!tx || !ty || !tz || capture_width <= 0 || capture_height <= 0 ||
      native_width <= 0 || native_height <= 0) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Malformed calibration translation or image size");
  }
  const double scale_x =
      static_cast<double>(capture_width) / static_cast<double>(native_width);
  const double scale_y =
      static_cast<double>(capture_height) / static_cast<double>(native_height);
  CameraIntrinsics intrinsics;
  intrinsics.fx = *focal * scale_x;
  intrinsics.fy = *focal * *aspect_ratio * scale_y;
  intrinsics.cx = *cx * scale_x;
  intrinsics.cy = *cy * scale_y;
  intrinsics.baseline_m = std::sqrt(*tx * *tx + *ty * *ty + *tz * *tz);
  return intrinsics;
}

Result<GraspStrategy> ParseGraspStrategy(const std::string& value) {
  if (value == "camera") return GraspStrategy::kCameraNormal;
  if (value == "postskel") return GraspStrategy::kPostSkeletonComponentSvd;
  return Status::Error(ErrorCode::kInvalidArgument,
                       "Unknown strategy '" + value +
                           "'; expected camera or postskel");
}

const char* GraspStrategyName(GraspStrategy strategy) {
  switch (strategy) {
    case GraspStrategy::kCameraNormal:
      return "camera";
    case GraspStrategy::kPostSkeletonComponentSvd:
      return "postskel";
  }
  return "unknown";
}

const char* ErrorCodeName(ErrorCode code) {
  switch (code) {
    case ErrorCode::kOk: return "ok";
    case ErrorCode::kInvalidArgument: return "invalid_argument";
    case ErrorCode::kShapeMismatch: return "shape_mismatch";
    case ErrorCode::kEmptyRoi: return "empty_roi";
    case ErrorCode::kNoVotes: return "no_votes";
    case ErrorCode::kNoComponents: return "no_components";
    case ErrorCode::kNoCenterline: return "no_centerline";
    case ErrorCode::kNoValidGrasps: return "no_valid_grasps";
    case ErrorCode::kIo: return "io";
    case ErrorCode::kUnsupportedFormat: return "unsupported_format";
    case ErrorCode::kProtocolUnavailable: return "protocol_unavailable";
    case ErrorCode::kEndOfStream: return "end_of_stream";
    case ErrorCode::kTimeout: return "timeout";
    case ErrorCode::kInternal: return "internal";
  }
  return "unknown";
}

Status WriteOutputCsv(const PipelineOutput& output,
                      const std::filesystem::path& output_directory,
                      GraspStrategy strategy) {
  std::error_code filesystem_error;
  std::filesystem::create_directories(output_directory, filesystem_error);
  if (filesystem_error) {
    return Status::Error(ErrorCode::kIo,
                         "Cannot create output directory: " +
                             filesystem_error.message());
  }
  const std::filesystem::path poses_path =
      output_directory / (output.frame_name + "_poses.csv");
  const std::filesystem::path points_path =
      output_directory / (output.frame_name + "_points.csv");
  const std::filesystem::path timing_path =
      output_directory / (output.frame_name + "_timings.csv");
  std::ofstream poses(poses_path);
  std::ofstream points(points_path);
  std::ofstream timing(timing_path);
  if (!poses || !points || !timing) {
    return Status::Error(ErrorCode::kIo,
                         "Cannot create one or more output CSV files");
  }
  poses << "frame,frame_id,timestamp_ns,strategy,arc_index,component,pose_index,"
           "x_m,y_m,z_m,qx,qy,qz,qw,r00,r01,r02,r10,r11,r12,r20,r21,r22\n";
  points << "frame,arc_index,component,point_index,pixel_x,pixel_y,x_m,y_m,z_m\n";
  poses << std::setprecision(12);
  points << std::setprecision(12);
  std::size_t pose_index = 0;
  for (std::size_t arc_index = 0; arc_index < output.arcs.size(); ++arc_index) {
    const GraspArc& arc = output.arcs[arc_index];
    for (const GraspPoint& point : arc.dense_points) {
      points << output.frame_name << ',' << arc_index << ','
             << arc.source_component << ',' << (&point - arc.dense_points.data())
             << ',' << point.pixel.x << ',' << point.pixel.y << ','
             << point.position_m.x << ',' << point.position_m.y << ','
             << point.position_m.z << '\n';
    }
    for (const GraspPose& pose : arc.poses) {
      poses << output.frame_name << ',' << output.frame_id << ','
            << output.timestamp_ns << ',' << GraspStrategyName(strategy) << ','
            << arc_index << ',' << arc.source_component << ',' << pose_index++
            << ',' << pose.position_m.x << ',' << pose.position_m.y << ','
            << pose.position_m.z << ',' << pose.quaternion_xyzw.x << ','
            << pose.quaternion_xyzw.y << ',' << pose.quaternion_xyzw.z << ','
            << pose.quaternion_xyzw.w;
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          poses << ',' << pose.rotation(row, column);
        }
      }
      poses << '\n';
    }
  }
  timing << "frame,voting_ms,filtering_ms,reprojection_ms,grasp_ms,total_ms\n"
         << output.frame_name << ',' << output.timings.voting_ms << ','
         << output.timings.filtering_ms << ','
         << output.timings.reprojection_ms << ',' << output.timings.grasp_ms
         << ',' << output.timings.total_ms << '\n';
  if (!poses || !points || !timing) {
    return Status::Error(ErrorCode::kIo, "Failed while writing output CSV");
  }
  return Status::Ok();
}

}  // namespace axis_grasp
