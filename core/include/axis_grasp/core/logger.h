#ifndef AXIS_GRASP_CORE_LOGGER_H_
#define AXIS_GRASP_CORE_LOGGER_H_

#include <string>

namespace axis_grasp {

enum class LogLevel { kDebug, kInfo, kWarning, kError };

class Logger {
 public:
  virtual ~Logger() = default;
  virtual void Log(LogLevel level, const std::string& message) = 0;
};

class NullLogger final : public Logger {
 public:
  void Log(LogLevel, const std::string&) override {}
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_LOGGER_H_
