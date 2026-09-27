#ifndef AXIS_GRASP_ROS_DATA_SOURCE_H_
#define AXIS_GRASP_ROS_DATA_SOURCE_H_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/adapters/depth_to_disparity.h"
#include "axis_grasp/core/logger.h"

#ifdef AXIS_GRASP_WITH_DETECTIONS
#include <bx_msgs/DepthImage.h>
#include <bx_msgs/DetectedInstances.h>
#endif

namespace axis_grasp_ros1 {

class RosLogger final : public axis_grasp::Logger {
 public:
  void Log(axis_grasp::LogLevel level, const std::string& message) override;
};

// Where the pipeline's binary ROI mask comes from.
//   kMask        a mono8 label image co-published with the disparity frame.
//   kDetections  polygon contours carried by bx_msgs/DetectedInstances.
enum class LabelSource { kMask, kDetections };

// Parses "mask" or "detections"; an empty string means kMask.
axis_grasp::Result<LabelSource> ParseLabelSource(const std::string& value);

// Which sensor produces FrameInput::disparity.
//   kDisparity  a 32FC1/64FC1 pixel-disparity image, used as published.
//   kDepth      a bx_msgs/DepthImage uint16-millimetre depth map, converted to
//               pixel disparity on arrival. Requires bx_msgs at build time.
enum class InputKind { kDisparity, kDepth };

// Parses "disparity" or "depth"; an empty string means kDisparity.
axis_grasp::Result<InputKind> ParseInputKind(const std::string& value);

const char* InputKindName(InputKind kind);

// What the depth input mode needs beyond the shared ROI-pairing policy.
struct DepthInputConfig {
  // DepthImage topic. Subscribed only when InputKind::kDepth is selected; the
  // disparity topic is subscribed only when it is not.
  std::string topic;
  // libCalib JSON. Read by this adapter to convert depth to disparity, and read
  // again by node.cpp for reprojection; both sites pass the same frame size, so
  // both derive identical intrinsics.
  std::string calibration_path;
  // The resolution the calibration file is valid at, as in node.cpp. Must be
  // the value node.cpp passes to its own LoadCalibrationJson: that call builds
  // the pipeline's reprojection, so if these two disagree the disparity this
  // adapter produces and the geometry the pipeline reprojects it with are
  // scaled differently -- finite, plausible, and wrong. The defaults match
  // config/default.yaml, which is also node.cpp's fallback.
  int native_width = 1600;
  int native_height = 1200;
  // Frame id for published poses. DepthImage carries no header, so there is no
  // frame id to inherit; empty keeps FrameInput's "camera" default.
  std::string frame_id;
  // Depths outside this band are treated as holes.
  axis_grasp::DepthBand band;
};

// Which detections are allowed to contribute to the ROI.
struct DetectionFilterConfig {
  // Detector that must have produced the message. Empty accepts any detector,
  // which is rarely what you want: several detectors share one topic.
  std::string detector_name;
  // Accepted `name` values. Empty accepts every class.
  std::vector<std::string> classes;
  // Minimum `confidence`; the message defines 0 as "not available", so 0
  // accepts everything.
  int min_confidence = 0;
};

// One accepted detection contour, still in the detector's own image frame.
struct DetectionContour {
  std::vector<std::uint16_t> contour;
  int image_width = 0;
  int image_height = 0;
};

class RosDataSource final : public axis_grasp::DataSource {
 public:
  RosDataSource(ros::NodeHandle node, const std::string& disparity_topic,
                const std::string& label_topic, double sync_slop_seconds,
                double mask_wait_timeout_seconds,
                LabelSource label_source = LabelSource::kMask,
                const std::string& detections_topic = std::string(),
                const DetectionFilterConfig& detection_filter =
                    DetectionFilterConfig(),
                InputKind input_kind = InputKind::kDisparity,
                const DepthInputConfig& depth = DepthInputConfig());

  axis_grasp::Status Next(axis_grasp::FrameInput* frame) override;
  std::string name() const override { return "ros1"; }

 private:
  void OnDisparity(const sensor_msgs::ImageConstPtr& message);
  void OnLabels(const sensor_msgs::ImageConstPtr& message);
  void PopulateFrame(axis_grasp::FrameInput* frame,
                     axis_grasp::Image<std::uint8_t>* labels);
#ifdef AXIS_GRASP_WITH_DETECTIONS
  void OnDepth(const bx_msgs::DepthImageConstPtr& message);
  void OnDetections(const bx_msgs::DetectedInstancesConstPtr& message);
#endif
  // Intrinsics for the depth conversion, loaded on the first decoded frame:
  // the frame size they must be scaled to is not known until then.
  axis_grasp::Status LoadDepthIntrinsics(int width, int height,
                                         double focal_length);

  ros::NodeHandle node_;
  ros::Subscriber disparity_subscriber_;
  ros::Subscriber depth_subscriber_;
  ros::Subscriber label_subscriber_;
  ros::Subscriber detections_subscriber_;
  std::int64_t sync_slop_ns_;
  std::int64_t mask_wait_timeout_ns_;
  std::optional<axis_grasp::Image<float>> disparity_;
  std::optional<std::chrono::steady_clock::time_point> disparity_wait_started_;
  std::optional<axis_grasp::Image<std::uint8_t>> labels_;
  ros::Time disparity_stamp_;
  ros::Time label_stamp_;
  // When the label image was received locally. Header stamps and receipt times
  // are different clocks, so mask pairing in depth mode -- where the range input
  // has no header at all -- takes both sides from this one.
  ros::Time label_arrival_;
  // Set when the newest label could not pair with the disparity it arrived
  // beside. Such a label is newer than that disparity, so for a monotonic
  // publisher it is the natural partner of the *next* disparity and must not be
  // rejected as stale against it.
  bool label_pending_ = false;
  // When the disparity frame was received locally. The detections source has no
  // publisher stamp to compare against, so it synchronizes on arrival times;
  // comparing those against a header stamp would mix two different clocks.
  ros::Time disparity_arrival_;
  std::string disparity_frame_id_;
  axis_grasp::Status disparity_status_;
  axis_grasp::Status label_status_;
  std::uint64_t sequence_ = 0;

  LabelSource label_source_ = LabelSource::kMask;
  DetectionFilterConfig detection_filter_;
  // Accepted contours from the newest DetectedInstances message, and when that
  // message arrived. DetectedInstances carries no Header, so there is no
  // publisher stamp to synchronize on and receipt time is the only option.
  std::optional<std::vector<DetectionContour>> detection_contours_;
  ros::Time detection_stamp_;
  // Set when the newest detections rasterise to an empty ROI, so the wait
  // message can say why instead of repeating itself.
  bool detection_roi_empty_ = false;

  InputKind input_kind_ = InputKind::kDisparity;
  DepthInputConfig depth_config_;
  // Empty until the first decodable depth frame supplies the frame size.
  std::optional<axis_grasp::CameraIntrinsics> depth_intrinsics_;
  // Latched, because an un-loadable calibration makes every depth frame
  // unconvertible: reporting it once per callback would say nothing new.
  axis_grasp::Status depth_intrinsics_status_ = axis_grasp::Status::Ok();
};

}  // namespace axis_grasp_ros1

#endif  // AXIS_GRASP_ROS_DATA_SOURCE_H_
