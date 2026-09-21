#ifndef AXIS_GRASP_CORE_STATUS_H_
#define AXIS_GRASP_CORE_STATUS_H_

#include <optional>
#include <string>
#include <utility>

namespace axis_grasp {

enum class ErrorCode {
  kOk = 0,
  kInvalidArgument,
  kShapeMismatch,
  kEmptyRoi,
  kNoVotes,
  kNoComponents,
  kNoCenterline,
  kNoValidGrasps,
  kIo,
  kUnsupportedFormat,
  kProtocolUnavailable,
  kEndOfStream,
  kTimeout,
  kInternal,
};

struct Status {
  ErrorCode code = ErrorCode::kOk;
  std::string message;

  static Status Ok() { return {}; }
  static Status Error(ErrorCode error_code, std::string error_message) {
    return {error_code, std::move(error_message)};
  }
  bool ok() const { return code == ErrorCode::kOk; }
};

template <typename T>
class Result {
 public:
  Result(T value) : value_(std::move(value)), status_(Status::Ok()) {}
  Result(Status status) : status_(std::move(status)) {}

  bool ok() const { return status_.ok(); }
  const Status& status() const { return status_; }
  const T& value() const& { return *value_; }
  T& value() & { return *value_; }
  T&& value() && { return std::move(*value_); }

 private:
  std::optional<T> value_;
  Status status_;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_STATUS_H_
