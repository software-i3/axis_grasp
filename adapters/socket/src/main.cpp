#include <cstdlib>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "axis_grasp/adapters/adapter_utils.h"
#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/adapters/socket_source.h"
#include "axis_grasp/core/config.h"
#include "axis_grasp/core/pipeline.h"

namespace axis_grasp {
namespace {

struct Options {
  std::filesystem::path calibration;
  std::filesystem::path output = "socket_results";
  std::string strategy = "camera";
  SocketOptions socket;
  PipelineConfig pipeline;
  int native_width = 1600;
  int native_height = 1200;
  int max_frames = 0;
};

void PrintHelp() {
  std::cout
      << "Usage: axis_grasp_socket --calibration FILE [options]\n\n"
      << "The RTPSender must already be streaming the sample_dumping.cc generic "
         "RTP payload to this host.\n\n"
      << "Options:\n"
      << "  --rtp-port N                 Receive port (5600)\n"
      << "  --local-address ADDRESS      uvgRTP local address (0.0.0.0)\n"
      << "  --timeout-ms N               Frame pull timeout (5000)\n"
      << "  --output DIR                 Output CSV directory (socket_results)\n"
      << "  --strategy camera|postskel  Normal strategy (camera)\n"
      << "  --native-width N             Calibration width (1600)\n"
      << "  --native-height N            Calibration height (1200)\n"
      << "  --max-frames N               Stop after N frames; 0 is unlimited\n"
      << "  --r-min N --r-max N          Vote radii (2, 14)\n"
      << "  --mag-percentile X           Gradient percentile (70)\n"
      << "  --acc-percentile X           Accumulator percentile (30)\n"
      << "  --close-k N                  Closing kernel (9)\n"
      << "  --top-components N           Kept components (3)\n"
      << "  --keep-rope                  Disable straight-component removal\n"
      << "  --spacing-m X                Output pose spacing (0.005)\n"
      << "  --postskel-dil N             SVD band half-width (2)\n";
}

Result<Options> ParseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      PrintHelp();
      std::exit(0);
    }
    if (argument == "--keep-rope") {
      options.pipeline.voting.remove_straight_rope = false;
      continue;
    }
    if (i + 1 >= argc) {
      return Status::Error(ErrorCode::kInvalidArgument,
                           "Missing value after " + argument);
    }
    const std::string value = argv[++i];
    if (argument == "--calibration") options.calibration = value;
    else if (argument == "--output") options.output = value;
    else if (argument == "--strategy") options.strategy = value;
    else if (argument == "--rtp-port") options.socket.rtp_port = static_cast<std::uint16_t>(std::stoul(value));
    else if (argument == "--local-address") options.socket.local_address = value;
    else if (argument == "--timeout-ms") options.socket.pull_timeout_ms = std::stoi(value);
    else if (argument == "--native-width") options.native_width = std::stoi(value);
    else if (argument == "--native-height") options.native_height = std::stoi(value);
    else if (argument == "--max-frames") options.max_frames = std::stoi(value);
    else if (argument == "--r-min") options.pipeline.voting.radius_min = std::stoi(value);
    else if (argument == "--r-max") options.pipeline.voting.radius_max = std::stoi(value);
    else if (argument == "--mag-percentile") options.pipeline.voting.magnitude_percentile = std::stod(value);
    else if (argument == "--acc-percentile") options.pipeline.components.accumulator_percentile = std::stod(value);
    else if (argument == "--close-k") {
      options.pipeline.voting.close_kernel = std::stoi(value);
      options.pipeline.components.close_kernel = std::stoi(value);
    } else if (argument == "--top-components") options.pipeline.components.top_components = std::stoi(value);
    else if (argument == "--spacing-m") options.pipeline.grasp.output_spacing_m = std::stod(value);
    else if (argument == "--postskel-dil") options.pipeline.grasp.post_skeleton_dilation_pixels = std::stoi(value);
    else {
      return Status::Error(ErrorCode::kInvalidArgument,
                           "Unknown option: " + argument);
    }
  }
  if (options.calibration.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "--calibration is required");
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
    // Swapping DatasetDataSource for SocketDataSource is the only ingestion
    // change; Pipeline and its configuration stay identical.
    std::unique_ptr<DataSource> source =
        std::make_unique<SocketDataSource>(options.socket);
    Status open = static_cast<SocketDataSource*>(source.get())->Open();
    if (!open.ok()) return Fail(open);

    StderrLogger logger;
    std::optional<Pipeline> pipeline;
    int processed = 0;
    while (options.max_frames == 0 || processed < options.max_frames) {
      FrameInput frame;
      const Status next = source->Next(&frame);
      if (next.code == ErrorCode::kTimeout) {
        logger.Log(LogLevel::kWarning, next.message);
        continue;
      }
      if (!next.ok()) return Fail(next);
      if (!pipeline.has_value()) {
        Result<CameraIntrinsics> intrinsics = LoadCalibrationJson(
            options.calibration, frame.disparity.width(),
            frame.disparity.height(), options.native_width,
            options.native_height);
        if (!intrinsics.ok()) return Fail(intrinsics.status());
        pipeline.emplace(options.pipeline, intrinsics.value(), &logger);
      }
      Result<PipelineOutput> result = pipeline->Process(frame);
      if (!result.ok()) {
        logger.Log(LogLevel::kError,
                   frame.name + ": " + result.status().message);
        continue;
      }
      Status write = WriteOutputCsv(
          result.value(), options.output, options.pipeline.grasp.strategy);
      if (!write.ok()) return Fail(write);
      ++processed;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "setup error: " << error.what() << '\n';
    return 1;
  }
}
