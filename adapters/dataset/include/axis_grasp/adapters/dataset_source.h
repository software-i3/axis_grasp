#ifndef AXIS_GRASP_ADAPTERS_DATASET_SOURCE_H_
#define AXIS_GRASP_ADAPTERS_DATASET_SOURCE_H_

#include <filesystem>
#include <string>
#include <vector>

#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {

struct DatasetFramePaths {
  std::filesystem::path disparity;
  std::filesystem::path label;
  bool has_label = true;
};

class DatasetDataSource final : public DataSource {
 public:
  explicit DatasetDataSource(std::vector<DatasetFramePaths> frames);

  static Result<DatasetDataSource> FromPair(
      const std::filesystem::path& disparity,
      const std::filesystem::path& label, bool use_label);
  static Result<DatasetDataSource> Discover(
      const std::filesystem::path& dataset_root, bool use_labels);

  Status Next(FrameInput* frame) override;
  std::string name() const override { return "dataset"; }

 private:
  std::vector<DatasetFramePaths> frames_;
  std::size_t next_index_ = 0;
};

Result<Image<float>> LoadNpyDisparity(const std::filesystem::path& path);
Result<Image<std::uint8_t>> LoadLabelMask(const std::filesystem::path& path,
                                         int width, int height);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_DATASET_SOURCE_H_
