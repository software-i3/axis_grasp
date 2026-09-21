#ifndef AXIS_GRASP_CORE_PIPELINE_H_
#define AXIS_GRASP_CORE_PIPELINE_H_

#include "axis_grasp/core/config.h"
#include "axis_grasp/core/logger.h"
#include "axis_grasp/core/status.h"
#include "axis_grasp/core/types.h"

namespace axis_grasp {

class Pipeline {
 public:
  Pipeline(PipelineConfig config, CameraIntrinsics intrinsics, Logger* logger);

  Result<PipelineOutput> Process(const FrameInput& frame) const;

 private:
  PipelineConfig config_;
  CameraIntrinsics intrinsics_;
  Logger* logger_;
  mutable NullLogger null_logger_;
};

Status ValidateConfig(const PipelineConfig& config,
                      const CameraIntrinsics& intrinsics);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_PIPELINE_H_
