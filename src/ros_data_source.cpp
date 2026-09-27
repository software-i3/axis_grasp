#include "axis_grasp/ros_data_source.h"

#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "axis_grasp/adapters/adapter_utils.h"
#include "axis_grasp/adapters/polygon_mask.h"
#include "axis_grasp/mask_pairing.h"

namespace axis_grasp_ros1 {
namespace {

bool HostIsBigEndian() {
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 0x01;
}

template <typename T>
T ReadScalar(const std::uint8_t* data, bool source_is_big_endian) {
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), data, sizeof(T));
  if (source_is_big_endian != HostIsBigEndian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value;
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

axis_grasp::Result<axis_grasp::Image<float>> DecodeDisparity(
    const sensor_msgs::Image& message) {
  std::size_t element_size = 0;
  bool float64 = false;
  if (message.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    element_size = sizeof(float);
  } else if (message.encoding == sensor_msgs::image_encodings::TYPE_64FC1) {
    element_size = sizeof(double);
    float64 = true;
  } else {
    return axis_grasp::Status::Error(
        axis_grasp::ErrorCode::kUnsupportedFormat,
        "disparity image encoding must be 32FC1 or 64FC1");
  }
  if (message.step < message.width * element_size ||
      message.data.size() < static_cast<std::size_t>(message.step) *
                                message.height) {
    return axis_grasp::Status::Error(axis_grasp::ErrorCode::kShapeMismatch,
                                     "disparity Image step/data is truncated");
  }
  axis_grasp::Image<float> output(message.height, message.width, 0.0F);
  for (std::uint32_t y = 0; y < message.height; ++y) {
    const std::uint8_t* row = message.data.data() + y * message.step;
    for (std::uint32_t x = 0; x < message.width; ++x) {
      output(y, x) = float64
                         ? static_cast<float>(ReadScalar<double>(
                               row + x * element_size, message.is_bigendian))
                         : ReadScalar<float>(row + x * element_size,
                                             message.is_bigendian);
    }
  }
  return output;
}

axis_grasp::Result<axis_grasp::Image<std::uint8_t>> DecodeLabels(
    const sensor_msgs::Image& message) {
  if (message.encoding != sensor_msgs::image_encodings::MONO8 &&
      message.encoding != sensor_msgs::image_encodings::TYPE_8UC1) {
    return axis_grasp::Status::Error(
        axis_grasp::ErrorCode::kUnsupportedFormat,
        "label image encoding must be mono8 or 8UC1");
  }
  if (message.step < message.width ||
      message.data.size() < static_cast<std::size_t>(message.step) *
                                message.height) {
    return axis_grasp::Status::Error(axis_grasp::ErrorCode::kShapeMismatch,
                                     "label Image step/data is truncated");
  }
  axis_grasp::Image<std::uint8_t> output(message.height, message.width, 0);
  for (std::uint32_t y = 0; y < message.height; ++y) {
    const std::uint8_t* row = message.data.data() + y * message.step;
    for (std::uint32_t x = 0; x < message.width; ++x) {
      output(y, x) = row[x] != 0;
    }
  }
  return output;
}

#ifdef AXIS_GRASP_WITH_DETECTIONS
// Each detection reports its own image size, so contours are rasterised one at
// a time in their own frame and unioned, rather than assuming the whole message
// shares one frame.
axis_grasp::Result<axis_grasp::Image<std::uint8_t>> RasterizeDetectionContours(
    const std::vector<DetectionContour>& contours, int width, int height) {
  axis_grasp::Image<std::uint8_t> mask(height, width, 0);
  for (const DetectionContour& contour : contours) {
    axis_grasp::Result<axis_grasp::Image<std::uint8_t>> single =
        axis_grasp::RasterizeFlatContours({contour.contour}, contour.image_width,
                                          contour.image_height, width, height);
    if (!single.ok()) continue;  // No polygon with 3+ vertices in this contour.
    const axis_grasp::Image<std::uint8_t>& raster = single.value();
    for (std::size_t i = 0; i < mask.size(); ++i) {
      if (raster[i] != 0) mask[i] = 1;
    }
  }
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] != 0) return std::move(mask);
  }
  return axis_grasp::Status::Error(
      axis_grasp::ErrorCode::kEmptyRoi,
      "detection contours did not cover any pixel of the disparity frame");
}
#endif

}  // namespace

axis_grasp::Result<LabelSource> ParseLabelSource(const std::string& value) {
  if (value.empty() || value == "mask") return LabelSource::kMask;
  if (value == "detections") return LabelSource::kDetections;
  return axis_grasp::Status::Error(
      axis_grasp::ErrorCode::kInvalidArgument,
      "label_source must be 'mask' or 'detections', got '" + value + "'");
}

axis_grasp::Result<InputKind> ParseInputKind(const std::string& value) {
  if (value.empty() || value == "disparity") return InputKind::kDisparity;
  if (value == "depth") return InputKind::kDepth;
  return axis_grasp::Status::Error(
      axis_grasp::ErrorCode::kInvalidArgument,
      "input_kind must be 'disparity' or 'depth', got '" + value + "'");
}

const char* InputKindName(InputKind kind) {
  switch (kind) {
    case InputKind::kDisparity:
      return "disparity";
    case InputKind::kDepth:
      return "depth";
  }
  return "unknown";
}

void RosLogger::Log(axis_grasp::LogLevel level, const std::string& message) {
  switch (level) {
    case axis_grasp::LogLevel::kDebug:
      ROS_DEBUG_STREAM(message);
      break;
    case axis_grasp::LogLevel::kInfo:
      ROS_INFO_STREAM(message);
      break;
    case axis_grasp::LogLevel::kWarning:
      ROS_WARN_STREAM(message);
      break;
    case axis_grasp::LogLevel::kError:
      ROS_ERROR_STREAM(message);
      break;
  }
}

RosDataSource::RosDataSource(ros::NodeHandle node,
                             const std::string& disparity_topic,
                             const std::string& label_topic,
                             double sync_slop_seconds,
                             double mask_wait_timeout_seconds,
                             LabelSource label_source,
                             const std::string& detections_topic,
                             const DetectionFilterConfig& detection_filter,
                             InputKind input_kind,
                             const DepthInputConfig& depth)
    : node_(std::move(node)),
      sync_slop_ns_(static_cast<std::int64_t>(sync_slop_seconds * 1e9)),
      mask_wait_timeout_ns_(
          static_cast<std::int64_t>(mask_wait_timeout_seconds * 1e9)),
      disparity_status_(axis_grasp::Status::Ok()),
      label_status_(axis_grasp::Status::Ok()),
      label_source_(label_source),
      detection_filter_(detection_filter),
      input_kind_(input_kind),
      depth_config_(depth) {
  // Exactly one range topic is subscribed, as with label_topic/detections_topic.
  if (input_kind_ == InputKind::kDepth) {
#ifdef AXIS_GRASP_WITH_DETECTIONS
    depth_subscriber_ =
        node_.subscribe(depth_config_.topic, 1, &RosDataSource::OnDepth, this);
#else
    // node.cpp rejects this configuration at startup; a library caller that
    // reaches here simply never receives a range image.
    ROS_WARN_STREAM("input_kind is 'depth' but axis_grasp was built without "
                    "bx_msgs; no frame will ever be produced");
#endif
  } else {
    disparity_subscriber_ =
        node_.subscribe(disparity_topic, 1, &RosDataSource::OnDisparity, this);
  }
  if (label_source_ == LabelSource::kDetections) {
#ifdef AXIS_GRASP_WITH_DETECTIONS
    detections_subscriber_ = node_.subscribe(
        detections_topic, 1, &RosDataSource::OnDetections, this);
#else
    // node.cpp rejects this configuration at startup; a library caller that
    // reaches here simply never receives labels.
    ROS_WARN_STREAM("label_source is 'detections' but axis_grasp was built "
                    "without bx_msgs; no ROI mask will be produced");
#endif
  } else {
    label_subscriber_ =
        node_.subscribe(label_topic, 1, &RosDataSource::OnLabels, this);
  }
}

void RosDataSource::OnDisparity(const sensor_msgs::ImageConstPtr& message) {
  // Keep one mask-mode disparity stable until it pairs or reaches its deadline.
  // Replacing it at camera rate would continually restart the timeout.
  if (label_source_ == LabelSource::kMask && disparity_.has_value()) return;

  axis_grasp::Result<axis_grasp::Image<float>> decoded =
      DecodeDisparity(*message);
  if (!decoded.ok()) {
    disparity_status_ = decoded.status();
    return;
  }
  disparity_ = std::move(decoded).value();
  disparity_stamp_ = message->header.stamp;
  disparity_arrival_ = ros::Time::now();
  disparity_frame_id_ = message->header.frame_id;
  // Restart the wait for the disparity that is actually being waited on. A
  // frame that was pending when an unpaired ROI source arrived is waiting for
  // that source, not for incidental clock drift, so the deadline has to be
  // measured from when this disparity became the pending one.
  disparity_wait_started_ = std::chrono::steady_clock::now();
  disparity_status_ = axis_grasp::Status::Ok();
}

#ifdef AXIS_GRASP_WITH_DETECTIONS
axis_grasp::Status RosDataSource::LoadDepthIntrinsics(int width, int height,
                                                      double focal_length) {
  axis_grasp::Result<axis_grasp::CameraIntrinsics> loaded =
      axis_grasp::LoadCalibrationJson(depth_config_.calibration_path, width,
                                      height, depth_config_.native_width,
                                      depth_config_.native_height);
  if (!loaded.ok()) {
    return axis_grasp::Status::Error(
        loaded.status().code,
        "depth input cannot use calibration '" +
            depth_config_.calibration_path + "' at " + std::to_string(width) +
            "x" + std::to_string(height) + ": " + loaded.status().message);
  }
  depth_intrinsics_ = loaded.value();
  const axis_grasp::CameraIntrinsics& intrinsics = *depth_intrinsics_;
  ROS_INFO_STREAM("depth input: " << width << "x" << height << " fx="
                  << intrinsics.fx << " fy=" << intrinsics.fy << " cx="
                  << intrinsics.cx << " cy=" << intrinsics.cy << " baseline="
                  << intrinsics.baseline_m << "m");
  // The conversion needs both, and the calibration file is free to supply
  // neither: a stereo pair with no translation has a zero baseline, and the
  // pipeline would reject the frame later anyway.
  if (!(intrinsics.fx > 0.0) || !(intrinsics.baseline_m > 0.0)) {
    return axis_grasp::Status::Error(
        axis_grasp::ErrorCode::kInvalidArgument,
        "depth input needs fx > 0 and a baseline > 0 to convert depth to "
        "disparity, but calibration '" +
            depth_config_.calibration_path + "' gives fx=" +
            std::to_string(intrinsics.fx) +
            " baseline_m=" + std::to_string(intrinsics.baseline_m));
  }
  // DepthImage.focal_length is fx in pixels despite documenting metres, as
  // mine_centering measured: (W/2)/tan(hfov/2) reproduces it exactly. It is an
  // independent read on the quantity the calibration file supplies, so a
  // disagreement means one of the two is wrong. This is also what catches a
  // wrong native_width/native_height, whose only other symptom is geometry that
  // is finite, plausible, and off by a scale factor.
  if (focal_length > 0.0) {
    const double ratio = intrinsics.fx / focal_length;
    if (std::fabs(ratio - 1.0) > 0.05) {
      ROS_WARN_STREAM(
          "depth input: calibration fx=" << intrinsics.fx
          << " disagrees with the depth message's focal_length="
          << focal_length << " by " << (ratio - 1.0) * 100.0
          << "%; if focal_length describes the published image size, check "
             "~native_width/~native_height, otherwise one of the two is wrong "
             "and every range is scaled by it");
    }
  }
  return axis_grasp::Status::Ok();
}

void RosDataSource::OnDepth(const bx_msgs::DepthImageConstPtr& message) {
  // Same keep-one-pending guard as OnDisparity: replacing the pending frame at
  // camera rate would continually restart the pairing deadline.
  if (label_source_ == LabelSource::kMask && disparity_.has_value()) return;

  if (message->depth_data.empty()) {
    ROS_WARN_THROTTLE(5.0,
                      "depth_data is empty; expected a uint16 PNG in "
                      "millimetres");
    return;
  }
  // PNG-encoded 16UC1, as mine_centering reads the same field.
  const cv::Mat encoded(1, static_cast<int>(message->depth_data.size()), CV_8U,
                        const_cast<std::uint8_t*>(message->depth_data.data()));
  // A malformed payload is a per-message fault, not a permanent one: a
  // truncated PNG in flight must not latch an error that outlives it. Skipping
  // the frame leaves the next one free to become the pending frame, because the
  // keep-one-pending guard only holds a frame that already converted.
  const cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  if (decoded.empty()) {
    ROS_WARN_THROTTLE(5.0,
                      "depth_data did not decode as an image (%zu bytes); "
                      "expected a uint16 PNG in millimetres",
                      message->depth_data.size());
    return;
  }
  if (decoded.type() != CV_16UC1) {
    ROS_WARN_THROTTLE(5.0,
                      "depth PNG decoded to OpenCV type %d, expected 16UC1 "
                      "(uint16 millimetres)",
                      decoded.type());
    return;
  }
  if (!depth_intrinsics_.has_value()) {
    const axis_grasp::Status loaded =
        LoadDepthIntrinsics(decoded.cols, decoded.rows, message->focal_length);
    if (!loaded.ok()) {
      // Latched: every later frame would fail the same way, so this becomes the
      // sticky error Next() reports until the node is restarted.
      depth_intrinsics_status_ = loaded;
      return;
    }
  }

  axis_grasp::Image<std::uint16_t> depth_mm(decoded.rows, decoded.cols, 0);
  for (int y = 0; y < decoded.rows; ++y) {
    const std::uint16_t* row = decoded.ptr<std::uint16_t>(y);
    for (int x = 0; x < decoded.cols; ++x) {
      depth_mm(y, x) = row[x];
    }
  }

  std::size_t holes = 0;
  axis_grasp::Result<axis_grasp::Image<float>> converted =
      axis_grasp::DepthToDisparity(depth_mm, *depth_intrinsics_,
                                   depth_config_.band, &holes);
  if (!converted.ok()) {
    // Unreachable while LoadDepthIntrinsics validates the same inputs, but a
    // conversion error must not latch: the frame is simply skipped.
    ROS_WARN_THROTTLE(5.0, "depth conversion failed: %s",
                      converted.status().message.c_str());
    return;
  }
  if (holes == depth_mm.size()) {
    ROS_WARN_THROTTLE(5.0,
                      "every sample of the depth frame is outside the writable "
                      "band [%f, %f] m or missing; check ~depth_min_m and "
                      "~depth_max_m",
                      depth_config_.band.min_m, depth_config_.band.max_m);
  } else if (holes * 4 > depth_mm.size()) {
    ROS_WARN_THROTTLE(5.0, "%zu of %zu depth samples are holes or out of band",
                      holes, depth_mm.size());
  }

  disparity_ = std::move(converted).value();
  // DepthImage has no Header. unix_time_ms keeps a sensor-time stamp on the
  // published pose array; pairing never uses it, because the label side would
  // have to come from a different clock.
  if (message->unix_time_ms == 0) {
    disparity_stamp_ = ros::Time::now();
  } else {
    disparity_stamp_.fromNSec(message->unix_time_ms * 1000000ULL);
  }
  disparity_arrival_ = ros::Time::now();
  disparity_frame_id_ = depth_config_.frame_id;
  disparity_wait_started_ = std::chrono::steady_clock::now();
  disparity_status_ = axis_grasp::Status::Ok();

  // 100/101 are the WaterLinked 3D sonar types. Their beams are equiangular, so
  // d = fx * baseline / Z is not their imaging model and the converted frame is
  // not a stereo disparity at all.
  if (message->sensor_type == 100 || message->sensor_type == 101) {
    ROS_WARN_THROTTLE(
        30.0,
        "depth frame carries sensor_type=%u, a WaterLinked 3D sonar, whose "
        "equiangular beams this stereo conversion does not model",
        static_cast<unsigned>(message->sensor_type));
  }
}
#endif

void RosDataSource::OnLabels(const sensor_msgs::ImageConstPtr& message) {
  axis_grasp::Result<axis_grasp::Image<std::uint8_t>> decoded =
      DecodeLabels(*message);
  if (!decoded.ok()) {
    label_status_ = decoded.status();
    return;
  }
  labels_ = std::move(decoded).value();
  label_stamp_ = message->header.stamp;
  label_arrival_ = ros::Time::now();
  label_pending_ = false;
  label_status_ = axis_grasp::Status::Ok();
}

void RosDataSource::PopulateFrame(
    axis_grasp::FrameInput* frame,
    axis_grasp::Image<std::uint8_t>* labels) {
  frame->disparity = std::move(*disparity_);
  if (labels == nullptr) {
    frame->labels = {};
    frame->has_labels = false;
  } else {
    frame->labels = std::move(*labels);
    frame->has_labels = true;
  }
  frame->timestamp_ns = disparity_stamp_.toNSec();
  frame->frame_id =
      disparity_frame_id_.empty() ? "camera" : disparity_frame_id_;
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << sequence_++;
  frame->name = name.str();
  disparity_.reset();
  disparity_wait_started_.reset();
  label_pending_ = false;
}

#ifdef AXIS_GRASP_WITH_DETECTIONS
void RosDataSource::OnDetections(
    const bx_msgs::DetectedInstancesConstPtr& message) {
  if (!detection_filter_.detector_name.empty() &&
      message->detector_name != detection_filter_.detector_name) {
    return;
  }
  std::vector<DetectionContour> accepted;
  int wrong_class = 0;
  int low_confidence = 0;
  int malformed = 0;
  for (const bx_msgs::DetectedInstance& instance : message->detections) {
    if (!detection_filter_.classes.empty() &&
        std::find(detection_filter_.classes.begin(),
                  detection_filter_.classes.end(),
                  instance.name) == detection_filter_.classes.end()) {
      ++wrong_class;
      continue;
    }
    if (instance.confidence < detection_filter_.min_confidence) {
      ++low_confidence;
      continue;
    }
    if (instance.image_width == 0 || instance.image_height == 0 ||
        instance.contour.size() < 6) {
      ++malformed;
      continue;
    }
    accepted.push_back(DetectionContour{
        instance.contour, static_cast<int>(instance.image_width),
        static_cast<int>(instance.image_height)});
  }
  ROS_DEBUG_STREAM("detections from '"
                   << message->detector_name << "': " << accepted.size()
                   << " accepted, " << wrong_class << " wrong class, "
                   << low_confidence << " low confidence, " << malformed
                   << " malformed");
  detection_contours_ = std::move(accepted);
  // DetectedInstances has no Header, so there is no publisher stamp to
  // synchronize on. Receipt time is the only signal available.
  detection_stamp_ = ros::Time::now();
  detection_roi_empty_ = false;
  label_status_ = axis_grasp::Status::Ok();
}
#endif

axis_grasp::Status RosDataSource::Next(axis_grasp::FrameInput* frame) {
  if (frame == nullptr) {
    return axis_grasp::Status::Error(
        axis_grasp::ErrorCode::kInvalidArgument,
        "RosDataSource::Next received a null frame");
  }
  ros::WallRate wait_rate(200.0);
  while (ros::ok()) {
    ros::spinOnce();
    if (!disparity_status_.ok()) return disparity_status_;
    if (!label_status_.ok()) return label_status_;
    if (!depth_intrinsics_status_.ok()) return depth_intrinsics_status_;
    if (label_source_ == LabelSource::kDetections) {
#ifdef AXIS_GRASP_WITH_DETECTIONS
      if (disparity_.has_value()) {
        // Both sides are local receipt times, so they share a clock. The
        // disparity's header stamp must not be used here: a bag replay or a
        // sim-time publisher can stamp on a completely different time base
        // than ros::Time::now(), which would reject every pair.
        const auto now = std::chrono::steady_clock::now();
        const std::int64_t elapsed_wait_ns =
            disparity_wait_started_.has_value()
                ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                      now - *disparity_wait_started_)
                      .count()
                : 0;
        const MaskPairingAction action = DecideMaskPairing(
            RoiSource::kDetections, detection_contours_.has_value(),
            disparity_arrival_.toNSec(),
            detection_contours_.has_value() ? detection_stamp_.toNSec() : 0,
            elapsed_wait_ns, sync_slop_ns_, mask_wait_timeout_ns_);
        if (action == MaskPairingAction::kDiscardStaleMask) {
          detection_contours_.reset();
          detection_roi_empty_ = false;
          continue;
        }
        if (action == MaskPairingAction::kPair) {
          axis_grasp::Result<axis_grasp::Image<std::uint8_t>> mask =
              RasterizeDetectionContours(*detection_contours_,
                                         disparity_->width(),
                                         disparity_->height());
          if (!mask.ok()) {
            // The contours belong to this disparity but covered none of it, so
            // they cannot become a ROI. Fall through to whole-frame voting
            // rather than dropping the frame.
            detection_contours_.reset();
            detection_roi_empty_ = true;
          } else {
            PopulateFrame(frame, &mask.value());
            detection_contours_.reset();
            detection_roi_empty_ = false;
            return axis_grasp::Status::Ok();
          }
        } else if (action == MaskPairingAction::kFallbackTimeout) {
          ROS_WARN_THROTTLE(
              5.0,
              detection_roi_empty_
                  ? "axis_grasp: detections produced an empty ROI at the "
                    "disparity resolution; processing the full disparity frame "
                    "(check ~detector_name and ~detection_classes)"
                  : "axis_grasp: no detections arrived within the configured "
                    "grace period; processing the full disparity frame");
          PopulateFrame(frame, nullptr);
          detection_roi_empty_ = false;
          return axis_grasp::Status::Ok();
        }
      }
      ROS_WARN_THROTTLE(
          5.0,
          "axis_grasp is waiting briefly for synchronized disparity and "
          "detections before whole-frame fallback");
      wait_rate.sleep();
      continue;
#endif
    }
    if (disparity_.has_value()) {
      const auto now = std::chrono::steady_clock::now();
      const std::int64_t elapsed_wait_ns =
          disparity_wait_started_.has_value()
              ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now - *disparity_wait_started_)
                    .count()
              : 0;
      // One clock on both sides. A depth frame has no header stamp, so in depth
      // mode the range side has to be its receipt time -- and then the label
      // side must be its own receipt time too, not its header stamp, which may
      // sit on a completely different time base (bag replay, sim time).
      const bool arrival_pairing = input_kind_ == InputKind::kDepth;
      const MaskPairingAction action = DecideMaskPairing(
          RoiSource::kLabelMask, labels_.has_value(),
          arrival_pairing ? disparity_arrival_.toNSec()
                          : disparity_stamp_.toNSec(),
          labels_.has_value() ? (arrival_pairing ? label_arrival_.toNSec()
                                                 : label_stamp_.toNSec())
                              : 0,
          elapsed_wait_ns, sync_slop_ns_, mask_wait_timeout_ns_,
          label_pending_);
      if (action == MaskPairingAction::kDiscardStaleMask) {
        labels_.reset();
        label_pending_ = false;
        continue;
      }
      if (action == MaskPairingAction::kPair) {
        if (disparity_->height() != labels_->height() ||
            disparity_->width() != labels_->width()) {
          disparity_.reset();
          disparity_wait_started_.reset();
          labels_.reset();
          label_pending_ = false;
          return axis_grasp::Status::Error(
              axis_grasp::ErrorCode::kShapeMismatch,
              "synchronized disparity and label image shapes differ");
        }
        PopulateFrame(frame, &*labels_);
        labels_.reset();
        return axis_grasp::Status::Ok();
      }
      if (action == MaskPairingAction::kFallbackMaskNewer) {
        // This label is ahead of the disparity, so it belongs to the next one.
        // Emit the current disparity unlabeled and keep the label for it.
        ROS_WARN_THROTTLE(
            5.0,
            "axis_grasp: label image is newer than the pending disparity; "
            "processing the full disparity frame and keeping the label");
        PopulateFrame(frame, nullptr);
        // Set after the call, not before it. PopulateFrame clears this flag so
        // that every caller states its own outcome, and setting it first let
        // that clear win: the retained label was then judged on the next
        // iteration with mask_is_expected false and discarded as stale, which
        // is the opposite of what keeping it is for.
        label_pending_ = true;
        return axis_grasp::Status::Ok();
      }
      if (action == MaskPairingAction::kFallbackTimeout) {
        ROS_WARN_THROTTLE(
            5.0,
            "axis_grasp: no label mask paired with the disparity within the "
            "configured grace period; processing the full disparity frame");
        PopulateFrame(frame, nullptr);
        return axis_grasp::Status::Ok();
      }
    }
    ROS_WARN_THROTTLE(
        5.0,
        "axis_grasp is waiting briefly for a synchronized disparity and label "
        "image before whole-frame fallback");
    wait_rate.sleep();
  }
  return axis_grasp::Status::Error(axis_grasp::ErrorCode::kEndOfStream,
                                   "ROS shutdown");
}

}  // namespace axis_grasp_ros1
