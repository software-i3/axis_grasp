#ifndef AXIS_GRASP_ADAPTERS_DATASET_SOURCE_H_
#define AXIS_GRASP_ADAPTERS_DATASET_SOURCE_H_

#include <string>
#include <vector>

#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/adapters/filesystem_compat.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {

struct DatasetFramePaths {
  axis_grasp::fs::path disparity;
  axis_grasp::fs::path label;
  bool has_label = true;
};

class DatasetDataSource final : public DataSource {
 public:
  explicit DatasetDataSource(std::vector<DatasetFramePaths> frames);

  static Result<DatasetDataSource> FromPair(
      const axis_grasp::fs::path& disparity,
      const axis_grasp::fs::path& label, bool use_label);
  static Result<DatasetDataSource> Discover(
      const axis_grasp::fs::path& dataset_root, bool use_labels);

  Status Next(FrameInput* frame) override;
  std::string name() const override { return "dataset"; }

 private:
  std::vector<DatasetFramePaths> frames_;
  std::size_t next_index_ = 0;
};

Result<Image<float>> LoadNpyDisparity(const axis_grasp::fs::path& path);
Result<Image<std::uint8_t>> LoadLabelMask(const axis_grasp::fs::path& path,
                                         int width, int height);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_DATASET_SOURCE_H_
