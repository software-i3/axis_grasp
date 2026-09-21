#include "axis_grasp/ros_data_source.h"

#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

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

}  // namespace

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
                             double sync_slop_seconds)
    : node_(std::move(node)),
      sync_slop_ns_(static_cast<std::int64_t>(sync_slop_seconds * 1e9)),
      disparity_status_(axis_grasp::Status::Ok()),
      label_status_(axis_grasp::Status::Ok()) {
  disparity_subscriber_ =
      node_.subscribe(disparity_topic, 1, &RosDataSource::OnDisparity, this);
  label_subscriber_ =
      node_.subscribe(label_topic, 1, &RosDataSource::OnLabels, this);
}

void RosDataSource::OnDisparity(const sensor_msgs::ImageConstPtr& message) {
  axis_grasp::Result<axis_grasp::Image<float>> decoded =
      DecodeDisparity(*message);
  if (!decoded.ok()) {
    disparity_status_ = decoded.status();
    return;
  }
  disparity_ = std::move(decoded).value();
  disparity_stamp_ = message->header.stamp;
  disparity_frame_id_ = message->header.frame_id;
  disparity_status_ = axis_grasp::Status::Ok();
}

void RosDataSource::OnLabels(const sensor_msgs::ImageConstPtr& message) {
  axis_grasp::Result<axis_grasp::Image<std::uint8_t>> decoded =
      DecodeLabels(*message);
  if (!decoded.ok()) {
    label_status_ = decoded.status();
    return;
  }
  labels_ = std::move(decoded).value();
  label_stamp_ = message->header.stamp;
  label_status_ = axis_grasp::Status::Ok();
}

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
    if (disparity_.has_value() && labels_.has_value()) {
      const std::int64_t disparity_ns = disparity_stamp_.toNSec();
      const std::int64_t label_ns = label_stamp_.toNSec();
      const std::int64_t difference = disparity_ns - label_ns;
      if (std::llabs(difference) > sync_slop_ns_) {
        if (difference < 0) {
          disparity_.reset();
        } else {
          labels_.reset();
        }
        continue;
      }
      if (disparity_->height() != labels_->height() ||
          disparity_->width() != labels_->width()) {
        disparity_.reset();
        labels_.reset();
        return axis_grasp::Status::Error(
            axis_grasp::ErrorCode::kShapeMismatch,
            "synchronized disparity and label image shapes differ");
      }
      frame->disparity = std::move(*disparity_);
      frame->labels = std::move(*labels_);
      frame->has_labels = true;
      frame->timestamp_ns = disparity_ns;
      frame->frame_id = disparity_frame_id_.empty() ? "camera" :
                                                      disparity_frame_id_;
      std::ostringstream name;
      name << std::setw(6) << std::setfill('0') << sequence_++;
      frame->name = name.str();
      disparity_.reset();
      labels_.reset();
      return axis_grasp::Status::Ok();
    }
    ROS_WARN_THROTTLE(
        5.0,
        "axis_grasp is waiting for synchronized disparity and label images");
    wait_rate.sleep();
  }
  return axis_grasp::Status::Error(axis_grasp::ErrorCode::kEndOfStream,
                                   "ROS shutdown");
}

}  // namespace axis_grasp_ros1
