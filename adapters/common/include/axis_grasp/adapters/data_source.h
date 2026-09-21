#ifndef AXIS_GRASP_ADAPTERS_DATA_SOURCE_H_
#define AXIS_GRASP_ADAPTERS_DATA_SOURCE_H_

#include <string>

#include "axis_grasp/core/status.h"
#include "axis_grasp/core/types.h"

namespace axis_grasp {

// The only ingestion contract used by dataset, socket, and ROS1 builds.
// Implementations own all transport/file concerns and return the same typed
// frame consumed by Pipeline. kEndOfStream is a normal terminal status.
class DataSource {
 public:
  virtual ~DataSource() = default;
  virtual Status Next(FrameInput* frame) = 0;
  virtual std::string name() const = 0;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_DATA_SOURCE_H_
