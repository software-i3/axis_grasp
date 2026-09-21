#ifndef AXIS_GRASP_ADAPTERS_SOCKET_SOURCE_H_
#define AXIS_GRASP_ADAPTERS_SOCKET_SOURCE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "axis_grasp/adapters/data_source.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {

struct SocketOptions {
  std::uint16_t rtp_port = 5600;
  int pull_timeout_ms = 5000;
  std::string local_address = "0.0.0.0";
};

struct DecodedSocketPayload {
  Image<float> disparity;
  bool has_right_image = false;
};

// Exposed for deterministic protocol tests and replay tools. Width/height are
// in/out because sample_dumping.cc auto-detects them from the first payload.
Result<DecodedSocketPayload> DecodeSampleDumpingPayload(
    const std::uint8_t* payload, std::size_t payload_size, bool first_frame,
    int* width, int* height);

class SocketDataSource final : public DataSource {
 public:
  explicit SocketDataSource(SocketOptions options);
  ~SocketDataSource() override;
  SocketDataSource(SocketDataSource&&) noexcept;
  SocketDataSource& operator=(SocketDataSource&&) noexcept;
  SocketDataSource(const SocketDataSource&) = delete;
  SocketDataSource& operator=(const SocketDataSource&) = delete;

  Status Open();
  Status Next(FrameInput* frame) override;
  std::string name() const override { return "socket"; }

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_ADAPTERS_SOCKET_SOURCE_H_
