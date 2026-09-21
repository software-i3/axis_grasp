#ifndef AXIS_GRASP_ROS_DATA_SOURCE_H_
#define AXIS_GRASP_ROS_DATA_SOURCE_H_

#include <cstdint>
#include <optional>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/core/logger.h"

namespace axis_grasp_ros1 {

class RosLogger final : public axis_grasp::Logger {
 public:
  void Log(axis_grasp::LogLevel level, const std::string& message) override;
};

class RosDataSource final : public axis_grasp::DataSource {
 public:
  RosDataSource(ros::NodeHandle node, const std::string& disparity_topic,
                const std::string& label_topic, double sync_slop_seconds);

  axis_grasp::Status Next(axis_grasp::FrameInput* frame) override;
  std::string name() const override { return "ros1"; }

 private:
  void OnDisparity(const sensor_msgs::ImageConstPtr& message);
  void OnLabels(const sensor_msgs::ImageConstPtr& message);

  ros::NodeHandle node_;
  ros::Subscriber disparity_subscriber_;
  ros::Subscriber label_subscriber_;
  std::int64_t sync_slop_ns_;
  std::optional<axis_grasp::Image<float>> disparity_;
  std::optional<axis_grasp::Image<std::uint8_t>> labels_;
  ros::Time disparity_stamp_;
  ros::Time label_stamp_;
  std::string disparity_frame_id_;
  axis_grasp::Status disparity_status_;
  axis_grasp::Status label_status_;
  std::uint64_t sequence_ = 0;
};

}  // namespace axis_grasp_ros1

#endif  // AXIS_GRASP_ROS_DATA_SOURCE_H_
