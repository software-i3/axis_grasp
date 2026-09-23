#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "axis_grasp/adapters/adapter_utils.h"
#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/core/config.h"
#include "axis_grasp/core/pipeline.h"
#include "axis_grasp/ros_data_source.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "axis_grasp_node");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  std::string disparity_topic = "/disparity";
  std::string label_topic = "/label";
  std::string output_topic = "/grasp_poses";
  std::string calibration_path;
  std::string strategy_name = "camera";
  double sync_slop_seconds = 0.03;
  double mask_wait_timeout_seconds = 0.10;
  int native_width = 1600;
  int native_height = 1200;
  std::string label_source_name = "mask";
  std::string detections_topic = "/ikan/vision/ml/detections";
  std::string detector_name;
  std::vector<std::string> detection_classes;
  int min_confidence = 0;
  private_node.param("disparity_topic", disparity_topic, disparity_topic);
  private_node.param("label_topic", label_topic, label_topic);
  private_node.param("output_topic", output_topic, output_topic);
  private_node.param("calibration", calibration_path, calibration_path);
  private_node.param("strategy", strategy_name, strategy_name);
  private_node.param("sync_slop_seconds", sync_slop_seconds,
                     sync_slop_seconds);
  private_node.param("mask_wait_timeout_seconds", mask_wait_timeout_seconds,
                     mask_wait_timeout_seconds);
  private_node.param("native_width", native_width, native_width);
  private_node.param("native_height", native_height, native_height);
  private_node.param("label_source", label_source_name, label_source_name);
  private_node.param("detections_topic", detections_topic, detections_topic);
  private_node.param("detector_name", detector_name, detector_name);
  private_node.param("detection_classes", detection_classes,
                     detection_classes);
  private_node.param("min_confidence", min_confidence, min_confidence);
  if (calibration_path.empty()) {
    ROS_FATAL("~calibration is required");
    return 1;
  }
  if (!std::isfinite(sync_slop_seconds) || sync_slop_seconds < 0.0) {
    ROS_FATAL("~sync_slop_seconds must be finite and non-negative");
    return 1;
  }
  if (!std::isfinite(mask_wait_timeout_seconds) ||
      mask_wait_timeout_seconds < 0.0) {
    ROS_FATAL("~mask_wait_timeout_seconds must be finite and non-negative");
    return 1;
  }
  axis_grasp::Result<axis_grasp_ros1::LabelSource> label_source =
      axis_grasp_ros1::ParseLabelSource(label_source_name);
  if (!label_source.ok()) {
    ROS_FATAL_STREAM(label_source.status().message);
    return 1;
  }
#ifndef AXIS_GRASP_WITH_DETECTIONS
  if (label_source.value() == axis_grasp_ros1::LabelSource::kDetections) {
    ROS_FATAL("label_source is 'detections' but axis_grasp was built without "
              "bx_msgs; rebuild with -DAXIS_GRASP_WITH_DETECTIONS=ON");
    return 1;
  }
#endif

  axis_grasp::PipelineConfig config;
  private_node.param("r_min", config.voting.radius_min,
                     config.voting.radius_min);
  private_node.param("r_max", config.voting.radius_max,
                     config.voting.radius_max);
  private_node.param("direction_bins", config.voting.direction_bins,
                     config.voting.direction_bins);
  private_node.param("magnitude_percentile",
                     config.voting.magnitude_percentile,
                     config.voting.magnitude_percentile);
  private_node.param("accumulator_percentile",
                     config.components.accumulator_percentile,
                     config.components.accumulator_percentile);
  private_node.param("close_kernel", config.components.close_kernel,
                     config.components.close_kernel);
  config.voting.close_kernel = config.components.close_kernel;
  private_node.param("top_components", config.components.top_components,
                     config.components.top_components);
  private_node.param("remove_straight_rope",
                     config.voting.remove_straight_rope,
                     config.voting.remove_straight_rope);
  private_node.param("bilateral_diameter", config.voting.bilateral_diameter,
                     config.voting.bilateral_diameter);
  private_node.param("bilateral_sigma_color",
                     config.voting.bilateral_sigma_color,
                     config.voting.bilateral_sigma_color);
  private_node.param("bilateral_sigma_space",
                     config.voting.bilateral_sigma_space,
                     config.voting.bilateral_sigma_space);
  private_node.param("enable_roi_crop", config.enable_roi_crop,
                     config.enable_roi_crop);
  private_node.param("roi_crop_padding", config.roi_crop_padding,
                     config.roi_crop_padding);
  private_node.param("spacing_m", config.grasp.output_spacing_m,
                     config.grasp.output_spacing_m);
  private_node.param("dense_spacing_pixels", config.grasp.dense_spacing_pixels,
                     config.grasp.dense_spacing_pixels);
  private_node.param("edge_margin_pixels", config.grasp.edge_margin_pixels,
                     config.grasp.edge_margin_pixels);
  private_node.param("post_skeleton_dilation_pixels",
                     config.grasp.post_skeleton_dilation_pixels,
                     config.grasp.post_skeleton_dilation_pixels);
  private_node.param("ransac_distance_m", config.grasp.ransac_distance_m,
                     config.grasp.ransac_distance_m);
  private_node.param("ransac_iterations", config.grasp.ransac_iterations,
                     config.grasp.ransac_iterations);
  private_node.param("ransac_min_inliers", config.grasp.ransac_min_inliers,
                     config.grasp.ransac_min_inliers);
  private_node.param("ransac_min_inlier_fraction",
                     config.grasp.ransac_min_inlier_fraction,
                     config.grasp.ransac_min_inlier_fraction);
  int ransac_seed = static_cast<int>(config.grasp.ransac_seed);
  private_node.param("ransac_seed", ransac_seed, ransac_seed);
  config.grasp.ransac_seed = static_cast<std::uint32_t>(ransac_seed);
  axis_grasp::Result<axis_grasp::GraspStrategy> strategy =
      axis_grasp::ParseGraspStrategy(strategy_name);
  if (!strategy.ok()) {
    ROS_FATAL_STREAM(strategy.status().message);
    return 1;
  }
  config.grasp.strategy = strategy.value();

  axis_grasp_ros1::DetectionFilterConfig detection_filter;
  detection_filter.detector_name = detector_name;
  detection_filter.classes = detection_classes;
  detection_filter.min_confidence = min_confidence;

  auto source = std::make_unique<axis_grasp_ros1::RosDataSource>(
      node, disparity_topic, label_topic, sync_slop_seconds,
      mask_wait_timeout_seconds, label_source.value(), detections_topic,
      detection_filter);
  ros::Publisher publisher =
      node.advertise<geometry_msgs::PoseArray>(output_topic, 1);
  axis_grasp_ros1::RosLogger logger;
  std::optional<axis_grasp::Pipeline> pipeline;

  std::string label_description =
      label_topic + " (fallback_timeout=" +
      std::to_string(mask_wait_timeout_seconds) + "s)";
  if (label_source.value() == axis_grasp_ros1::LabelSource::kDetections) {
    label_description =
        detections_topic + " (detector='" +
        (detector_name.empty() ? std::string("*") : detector_name) +
        "', classes=" +
        (detection_classes.empty()
             ? std::string("*")
             : std::to_string(detection_classes.size()) + " listed") +
        ", min_confidence=" + std::to_string(min_confidence) +
        ", fallback_timeout=" + std::to_string(mask_wait_timeout_seconds) +
        "s)";
  }
  ROS_INFO_STREAM("axis_grasp ready: disparity=" << disparity_topic
                  << " label_source=" << label_source_name << " from "
                  << label_description << " output=" << output_topic
                  << " strategy=" << strategy_name
                  << " roi_crop=" << (config.enable_roi_crop ? "on" : "off"));

  while (ros::ok()) {
    axis_grasp::FrameInput frame;
    const axis_grasp::Status next = source->Next(&frame);
    if (next.code == axis_grasp::ErrorCode::kEndOfStream) break;
    if (!next.ok()) {
      ROS_ERROR_STREAM(next.message);
      continue;
    }
    if (!pipeline.has_value()) {
      axis_grasp::Result<axis_grasp::CameraIntrinsics> intrinsics =
          axis_grasp::LoadCalibrationJson(
              calibration_path, frame.disparity.width(),
              frame.disparity.height(), native_width, native_height);
      if (!intrinsics.ok()) {
        ROS_FATAL_STREAM(intrinsics.status().message);
        return 1;
      }
      pipeline.emplace(config, intrinsics.value(), &logger);
    }
    axis_grasp::Result<axis_grasp::PipelineOutput> result =
        pipeline->Process(frame);
    if (!result.ok()) {
      ROS_WARN_STREAM(frame.name << ": " << result.status().message);
      continue;
    }
    geometry_msgs::PoseArray message;
    message.header.frame_id = result.value().frame_id;
    message.header.stamp.fromNSec(result.value().timestamp_ns);
    for (const axis_grasp::GraspArc& arc : result.value().arcs) {
      for (const axis_grasp::GraspPose& grasp : arc.poses) {
        geometry_msgs::Pose pose;
        pose.position.x = grasp.position_m.x;
        pose.position.y = grasp.position_m.y;
        pose.position.z = grasp.position_m.z;
        pose.orientation.x = grasp.quaternion_xyzw.x;
        pose.orientation.y = grasp.quaternion_xyzw.y;
        pose.orientation.z = grasp.quaternion_xyzw.z;
        pose.orientation.w = grasp.quaternion_xyzw.w;
        message.poses.push_back(pose);
      }
    }
    publisher.publish(message);
  }
  return 0;
}
