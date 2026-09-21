#ifndef AXIS_GRASP_ADAPTERS_ADAPTER_UTILS_H_
#define AXIS_GRASP_ADAPTERS_ADAPTER_UTILS_H_

#include <filesystem>
#include <string>

#include "axis_grasp/core/config.h"
#include "axis_grasp/core/logger.h"
#include "axis_grasp/core/status.h"
#include "axis_grasp/core/types.h"

namespace axis_grasp {

class StderrLogger final : public Logger {
 public:
  void Log(LogLevel level, const std::string& message) override;
};

Result<CameraIntrinsics> LoadCalibrationJson(
    const std::filesystem::path& path, int capture_width, int capture_height,
    int native_width, int native_height);

Result<GraspStrategy> ParseGraspStrategy(const std::string& value);
const char* GraspStrategyName(GraspStrategy strategy);
const char* ErrorCodeName(ErrorCode code);

Status WriteOutputCsv(const PipelineOutput& output,
                      const std::filesystem::path& output_directory,
                      GraspStrategy strategy);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_ADAPTER_UTILS_H_
