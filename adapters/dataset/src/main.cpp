#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>

#include "axis_grasp/adapters/adapter_utils.h"
#include "axis_grasp/adapters/dataset_source.h"
#include "axis_grasp/core/config.h"
#include "axis_grasp/core/pipeline.h"

namespace axis_grasp {
namespace {

struct Options {
  std::filesystem::path disparity;
  std::filesystem::path label;
  std::filesystem::path dataset_root;
  std::filesystem::path calibration;
  std::filesystem::path output = "cpp_results";
  std::string strategy = "camera";
  bool use_label = true;
  int capture_width = 800;
  int capture_height = 600;
  int native_width = 1600;
  int native_height = 1200;
  PipelineConfig pipeline;
};

void PrintHelp() {
  std::cout
      << "Usage:\n"
      << "  axis_grasp_dataset --disparity FRAME.npy --label ROI.txt \\\n+     --calibration calibration.json [options]\n"
      << "  axis_grasp_dataset --dataset-root testing_dataset \\\n+     --calibration calibration.json [options]\n\n"
      << "Options:\n"
      << "  --output DIR                 Output CSV directory (cpp_results)\n"
      << "  --strategy camera|postskel  Approach-normal strategy (camera)\n"
      << "  --no-label                   Vote on the full frame\n"
      << "  --no-roi-crop                Disable label bounding-box crop\n"
      << "  --roi-crop-padding N         Crop halo (-1 selects safe automatic halo)\n"
      << "  --capture-width N            Calibrated output width (800)\n"
      << "  --capture-height N           Calibrated output height (600)\n"
      << "  --native-width N             Native calibration width (1600)\n"
      << "  --native-height N            Native calibration height (1200)\n"
      << "  --r-min N --r-max N          Vote radii (2, 14)\n"
      << "  --mag-percentile X           Gradient threshold percentile (70)\n"
      << "  --n-bins N                   Even direction-bin count (8)\n"
      << "  --acc-percentile X           Accumulator percentile (30)\n"
      << "  --close-k N                  Closing kernel size (9)\n"
      << "  --top-components N           Components kept by vote mass (3)\n"
      << "  --keep-rope                  Disable straight-component removal\n"
      << "  --dense-spacing-px X         Dense centerline spacing (3.5)\n"
      << "  --edge-margin-px N           Invalid image-edge margin (6)\n"
      << "  --spacing-m X                Output pose spacing (0.005)\n"
      << "  --approach-axis X Y Z        Camera method approach (0 0 -1)\n"
      << "  --postskel-dil N             SVD band half-width (2)\n"
      << "  --ransac-dist-m X            Plane inlier distance (0.005)\n"
      << "  --ransac-iters N             Plane RANSAC iterations (50)\n"
      << "  --ransac-min-inliers N       Plane minimum inliers (5)\n";
}

Status MissingValue(const std::string& option) {
  return Status::Error(ErrorCode::kInvalidArgument,
                       "Missing value after " + option);
}

Result<Options> ParseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    auto value = [&]() -> const char* {
      if (i + 1 >= argc) return nullptr;
      return argv[++i];
    };
    if (argument == "--help" || argument == "-h") {
      PrintHelp();
      std::exit(0);
    } else if (argument == "--no-label") {
      options.use_label = false;
    } else if (argument == "--no-roi-crop") {
      options.pipeline.enable_roi_crop = false;
    } else if (argument == "--keep-rope") {
      options.pipeline.voting.remove_straight_rope = false;
    } else if (argument == "--approach-axis") {
      if (i + 3 >= argc) return MissingValue(argument);
      options.pipeline.grasp.camera_approach_axis =
          {std::stod(argv[++i]), std::stod(argv[++i]), std::stod(argv[++i])};
    } else {
      const char* raw = value();
      if (raw == nullptr) return MissingValue(argument);
      const std::string text = raw;
      if (argument == "--disparity") options.disparity = text;
      else if (argument == "--label") options.label = text;
      else if (argument == "--dataset-root") options.dataset_root = text;
      else if (argument == "--calibration") options.calibration = text;
      else if (argument == "--output") options.output = text;
      else if (argument == "--strategy") options.strategy = text;
      else if (argument == "--capture-width") options.capture_width = std::stoi(text);
      else if (argument == "--capture-height") options.capture_height = std::stoi(text);
      else if (argument == "--native-width") options.native_width = std::stoi(text);
      else if (argument == "--native-height") options.native_height = std::stoi(text);
      else if (argument == "--r-min") options.pipeline.voting.radius_min = std::stoi(text);
      else if (argument == "--r-max") options.pipeline.voting.radius_max = std::stoi(text);
      else if (argument == "--roi-crop-padding") options.pipeline.roi_crop_padding = std::stoi(text);
      else if (argument == "--mag-percentile") options.pipeline.voting.magnitude_percentile = std::stod(text);
      else if (argument == "--n-bins") options.pipeline.voting.direction_bins = std::stoi(text);
      else if (argument == "--acc-percentile") options.pipeline.components.accumulator_percentile = std::stod(text);
      else if (argument == "--close-k") {
        options.pipeline.voting.close_kernel = std::stoi(text);
        options.pipeline.components.close_kernel = std::stoi(text);
      } else if (argument == "--top-components") options.pipeline.components.top_components = std::stoi(text);
      else if (argument == "--dense-spacing-px") options.pipeline.grasp.dense_spacing_pixels = std::stod(text);
      else if (argument == "--edge-margin-px") options.pipeline.grasp.edge_margin_pixels = std::stoi(text);
      else if (argument == "--spacing-m") options.pipeline.grasp.output_spacing_m = std::stod(text);
      else if (argument == "--postskel-dil") options.pipeline.grasp.post_skeleton_dilation_pixels = std::stoi(text);
      else if (argument == "--ransac-dist-m") options.pipeline.grasp.ransac_distance_m = std::stod(text);
      else if (argument == "--ransac-iters") options.pipeline.grasp.ransac_iterations = std::stoi(text);
      else if (argument == "--ransac-min-inliers") options.pipeline.grasp.ransac_min_inliers = std::stoi(text);
      else {
        return Status::Error(ErrorCode::kInvalidArgument,
                             "Unknown option: " + argument);
      }
    }
  }
  if (options.calibration.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "--calibration is required");
  }
  if (options.dataset_root.empty() == options.disparity.empty()) {
    return Status::Error(
        ErrorCode::kInvalidArgument,
        "Pass exactly one of --dataset-root or --disparity");
  }
  if (!options.dataset_root.empty() && !options.label.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "--label is only valid with --disparity");
  }
  if (!options.disparity.empty() && options.use_label && options.label.empty()) {
    const std::filesystem::path candidate =
        options.disparity.parent_path().parent_path() / "labels" /
        (options.disparity.stem().string() + ".txt");
    options.label = candidate;
  }
  Result<GraspStrategy> strategy = ParseGraspStrategy(options.strategy);
  if (!strategy.ok()) return strategy.status();
  options.pipeline.grasp.strategy = strategy.value();
  return options;
}

int Fail(const Status& status) {
  std::cerr << "error[" << ErrorCodeName(status.code) << "]: "
            << status.message << '\n';
  return 1;
}

}  // namespace
}  // namespace axis_grasp

int main(int argc, char** argv) {
  using namespace axis_grasp;
  try {
    Result<Options> parsed = ParseOptions(argc, argv);
    if (!parsed.ok()) return Fail(parsed.status());
    Options options = std::move(parsed).value();
    Result<DatasetDataSource> source_result =
        options.dataset_root.empty()
            ? DatasetDataSource::FromPair(options.disparity, options.label,
                                          options.use_label)
            : DatasetDataSource::Discover(options.dataset_root,
                                          options.use_label);
    if (!source_result.ok()) return Fail(source_result.status());
    DatasetDataSource source = std::move(source_result).value();
    Result<CameraIntrinsics> intrinsics = LoadCalibrationJson(
        options.calibration, options.capture_width, options.capture_height,
        options.native_width, options.native_height);
    if (!intrinsics.ok()) return Fail(intrinsics.status());
    StderrLogger logger;
    Pipeline pipeline(options.pipeline, intrinsics.value(), &logger);
    int failures = 0;
    while (true) {
      FrameInput frame;
      const Status next = source.Next(&frame);
      if (next.code == ErrorCode::kEndOfStream) break;
      if (!next.ok()) return Fail(next);
      Result<PipelineOutput> result = pipeline.Process(frame);
      if (!result.ok()) {
        ++failures;
        logger.Log(LogLevel::kError,
                   frame.name + ": " + result.status().message);
        continue;
      }
      const Status write = WriteOutputCsv(
          result.value(), options.output, options.pipeline.grasp.strategy);
      if (!write.ok()) return Fail(write);
    }
    return failures == 0 ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "setup error: " << error.what() << '\n';
    return 1;
  }
}
