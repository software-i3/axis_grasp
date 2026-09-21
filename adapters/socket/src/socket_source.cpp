#include "axis_grasp/adapters/socket_source.h"

#include <uvgrtp/lib.hh>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace axis_grasp {
namespace {

// The constants, FP16 conversion, layout detection, offsets, and size checks
// in this section are copied from SVCReceiverSample/src/sample_dumping.cc.
constexpr int kNtpHeaderSize = sizeof(int) * 2;

float Fp16ToFp32(std::uint16_t half) {
  const std::uint32_t sign = static_cast<std::uint32_t>(half & 0x8000U) << 16;
  const std::uint32_t exponent = (half & 0x7C00U) >> 10;
  const std::uint32_t mantissa = half & 0x03FFU;
  std::uint32_t bits;
  if (exponent == 0) {
    if (mantissa == 0) {
      bits = sign;
    } else {
      int adjustment = 0;
      std::uint32_t normalized = mantissa;
      while ((normalized & 0x0400U) == 0) {
        normalized <<= 1;
        ++adjustment;
      }
      normalized &= 0x03FFU;
      bits = sign |
             (static_cast<std::uint32_t>(127 - 15 - adjustment) << 23) |
             (normalized << 13);
    }
  } else if (exponent == 0x1F) {
    bits = sign | 0x7F800000U | (mantissa << 13);
  } else {
    bits = sign | ((exponent + (127 - 15)) << 23) | (mantissa << 13);
  }
  float output;
  std::memcpy(&output, &bits, sizeof(output));
  return output;
}

std::vector<float> DecodeDepth(const std::uint8_t* data, std::size_t bytes,
                               int width, int height) {
  const std::size_t count = static_cast<std::size_t>(width) * height;
  if (bytes == count * 2) {
    const auto* values = reinterpret_cast<const std::uint16_t*>(data);
    std::vector<float> output(count);
    for (std::size_t i = 0; i < count; ++i) output[i] = Fp16ToFp32(values[i]);
    return output;
  }
  if (bytes == count * 4) {
    const auto* values = reinterpret_cast<const float*>(data);
    return std::vector<float>(values, values + count);
  }
  return {};
}

}  // namespace

Result<DecodedSocketPayload> DecodeSampleDumpingPayload(
    const std::uint8_t* payload, std::size_t payload_size, bool first_frame,
    int* width, int* height) {
  if (payload == nullptr || width == nullptr || height == nullptr ||
      payload_size < static_cast<std::size_t>(kNtpHeaderSize)) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "Invalid RTP payload or output dimensions");
  }

  // Verbatim decision order and constants from sample_dumping.cc. In the
  // source deployment, 800x600 is the SGM/refined_pro path in active use.
  if (first_frame) {
    const std::size_t data_len = payload_size - kNtpHeaderSize;
    if (data_len == 2400000) {
      *width = 800;
      *height = 600;
    } else if (data_len == 600008) {
      *width = 400;
      *height = 300;
    } else if (data_len == 3840008) {
      *width = 800;
      *height = 600;
    } else if (data_len == 960008) {
      *width = 400;
      *height = 300;
    } else {
      *width = 800;
      *height = 600;
    }
  }

  const int image_size = *width * *height * 3;
  const int disparity_size = *width * *height * 2;
  const std::size_t minimum_payload =
      kNtpHeaderSize + image_size + disparity_size;
  const std::size_t full_payload =
      kNtpHeaderSize + 2 * image_size + disparity_size;
  if (payload_size < minimum_payload) {
    std::ostringstream message;
    message << "Received invalid frame of size " << payload_size
            << " (minimum expected " << minimum_payload << ')';
    return Status::Error(ErrorCode::kUnsupportedFormat, message.str());
  }
  const bool has_right = payload_size >= full_payload;
  const std::uint8_t* left_image = payload + kNtpHeaderSize;
  const std::uint8_t* disparity_raw =
      left_image + (has_right ? 2 * image_size : image_size);
  std::vector<float> disparity = DecodeDepth(
      disparity_raw, disparity_size, *width, *height);
  if (disparity.empty()) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Failed to decode RTP FP16 disparity buffer");
  }
  return DecodedSocketPayload{
      Image<float>(*height, *width, std::move(disparity)), has_right};
}

class SocketDataSource::Impl {
 public:
  explicit Impl(SocketOptions source_options)
      : options(std::move(source_options)) {}

  ~Impl() {
    if (session != nullptr) context.destroy_session(session);
  }

  SocketOptions options;
  uvgrtp::context context;
  uvgrtp::session* session = nullptr;
  uvgrtp::media_stream* receiver = nullptr;
  std::int64_t frame_index = 0;
  int width = 800;
  int height = 600;
};

SocketDataSource::SocketDataSource(SocketOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

SocketDataSource::~SocketDataSource() = default;
SocketDataSource::SocketDataSource(SocketDataSource&&) noexcept = default;
SocketDataSource& SocketDataSource::operator=(SocketDataSource&&) noexcept =
    default;

Status SocketDataSource::Open() {
  if (impl_->session != nullptr) return Status::Ok();
  impl_->session = impl_->context.create_session(impl_->options.local_address);
  if (impl_->session == nullptr) {
    return Status::Error(ErrorCode::kIo, "uvgRTP could not create a session");
  }
  impl_->receiver = impl_->session->create_stream(
      impl_->options.rtp_port, RTP_FORMAT_GENERIC,
      RCE_RECEIVE_ONLY | RCE_FRAGMENT_GENERIC);
  if (impl_->receiver == nullptr) {
    impl_->context.destroy_session(impl_->session);
    impl_->session = nullptr;
    return Status::Error(ErrorCode::kIo,
                         "uvgRTP could not create the receive stream");
  }
  return Status::Ok();
}

Status SocketDataSource::Next(FrameInput* frame) {
  if (frame == nullptr) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "SocketDataSource::Next received a null frame");
  }
  if (impl_->receiver == nullptr) {
    return Status::Error(ErrorCode::kIo,
                         "SocketDataSource must be opened before Next");
  }
  uvgrtp::frame::rtp_frame* rtp_frame =
      impl_->receiver->pull_frame(impl_->options.pull_timeout_ms);
  if (rtp_frame == nullptr) {
    return Status::Error(ErrorCode::kTimeout, "RTP frame pull timed out");
  }
  ++impl_->frame_index;
  Result<DecodedSocketPayload> decoded = DecodeSampleDumpingPayload(
      rtp_frame->payload, rtp_frame->payload_len, impl_->frame_index == 1,
      &impl_->width, &impl_->height);
  uvgrtp::frame::dealloc_frame(rtp_frame);
  if (!decoded.ok()) return decoded.status();

  frame->disparity = std::move(decoded).value().disparity;
  frame->labels = {};
  frame->has_labels = false;
  frame->timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  frame->frame_id = "camera";
  std::ostringstream frame_name;
  frame_name << std::setw(6) << std::setfill('0') << (impl_->frame_index - 1);
  frame->name = frame_name.str();

  // TODO(camera-driver): publish the decoded raw image/disparity packet as
  // ROS1 sensor_msgs here through a separate bridge owned by the adapter.
  return Status::Ok();
}

}  // namespace axis_grasp
