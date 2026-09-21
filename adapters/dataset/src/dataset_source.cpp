#include "axis_grasp/adapters/dataset_source.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace axis_grasp {
namespace {

struct NpyData {
  std::string descriptor;
  int height = 0;
  int width = 0;
  std::vector<std::uint8_t> bytes;
};

bool HostIsLittleEndian() {
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 1;
}

template <typename T>
T ByteSwap(T value) {
  std::array<std::uint8_t, sizeof(T)> source{};
  std::array<std::uint8_t, sizeof(T)> target{};
  std::memcpy(source.data(), &value, sizeof(T));
  std::reverse_copy(source.begin(), source.end(), target.begin());
  std::memcpy(&value, target.data(), sizeof(T));
  return value;
}

Result<NpyData> ReadNpy(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return Status::Error(ErrorCode::kIo,
                         "Cannot open NPY file: " + path.string());
  }
  char magic[6] = {};
  stream.read(magic, sizeof(magic));
  const char expected[6] = {static_cast<char>(0x93), 'N', 'U', 'M', 'P', 'Y'};
  if (!stream || std::memcmp(magic, expected, sizeof(magic)) != 0) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "File is not a NumPy NPY array: " + path.string());
  }
  std::uint8_t major = 0;
  std::uint8_t minor = 0;
  stream.read(reinterpret_cast<char*>(&major), 1);
  stream.read(reinterpret_cast<char*>(&minor), 1);
  (void)minor;
  std::uint32_t header_length = 0;
  if (major == 1) {
    std::uint16_t length16 = 0;
    stream.read(reinterpret_cast<char*>(&length16), sizeof(length16));
    if (!HostIsLittleEndian()) length16 = ByteSwap(length16);
    header_length = length16;
  } else if (major == 2 || major == 3) {
    stream.read(reinterpret_cast<char*>(&header_length), sizeof(header_length));
    if (!HostIsLittleEndian()) header_length = ByteSwap(header_length);
  } else {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Unsupported NPY version");
  }
  if (!stream || header_length == 0 || header_length > 1024 * 1024) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Invalid NPY header length");
  }
  std::string header(header_length, '\0');
  stream.read(header.data(), header.size());
  std::smatch match;
  const std::regex descriptor_regex(
      R"(['\"]descr['\"]\s*:\s*['\"]([^'\"]+)['\"])");
  const std::regex shape_regex(
      R"(['\"]shape['\"]\s*:\s*\(\s*([0-9]+)\s*,\s*([0-9]+)\s*,?\s*\))");
  if (!std::regex_search(header, match, descriptor_regex)) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "NPY descriptor is missing");
  }
  NpyData data;
  data.descriptor = match[1].str();
  if (!std::regex_search(header, match, shape_regex)) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Only two-dimensional NPY arrays are supported");
  }
  data.height = std::stoi(match[1].str());
  data.width = std::stoi(match[2].str());
  if (header.find("'fortran_order': True") != std::string::npos ||
      header.find("\"fortran_order\": True") != std::string::npos) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Fortran-order NPY arrays are unsupported");
  }
  const std::size_t element_count =
      static_cast<std::size_t>(data.height) * data.width;
  std::size_t element_size = 0;
  if (data.descriptor == "<f4" || data.descriptor == ">f4" ||
      data.descriptor == "=f4" || data.descriptor == "<i4" ||
      data.descriptor == ">i4" || data.descriptor == "=i4") {
    element_size = 4;
  } else if (data.descriptor == "<f8" || data.descriptor == ">f8" ||
             data.descriptor == "=f8") {
    element_size = 8;
  } else if (data.descriptor == "|u1" || data.descriptor == "|i1" ||
             data.descriptor == "|b1" || data.descriptor == "?") {
    element_size = 1;
  } else if (data.descriptor == "<u2" || data.descriptor == ">u2" ||
             data.descriptor == "=u2") {
    element_size = 2;
  } else {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Unsupported NPY dtype: " + data.descriptor);
  }
  if (element_count > std::numeric_limits<std::size_t>::max() / element_size) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "NPY array size overflows address space");
  }
  data.bytes.resize(element_count * element_size);
  stream.read(reinterpret_cast<char*>(data.bytes.data()), data.bytes.size());
  if (!stream || stream.peek() != std::char_traits<char>::eof()) {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "NPY payload size does not match shape and dtype");
  }
  return data;
}

template <typename T>
T ReadElement(const NpyData& data, std::size_t index) {
  T value;
  std::memcpy(&value, data.bytes.data() + index * sizeof(T), sizeof(T));
  const bool big_endian = !data.descriptor.empty() && data.descriptor[0] == '>';
  const bool little_endian =
      !data.descriptor.empty() && data.descriptor[0] == '<';
  const bool swap = (big_endian && HostIsLittleEndian()) ||
                    (little_endian && !HostIsLittleEndian());
  if (swap && sizeof(T) > 1) value = ByteSwap(value);
  return value;
}

bool PointOnSegment(double px, double py, const Vec2d& a, const Vec2d& b) {
  const double cross = (px - a.x) * (b.y - a.y) -
                       (py - a.y) * (b.x - a.x);
  if (std::abs(cross) > 1e-9) return false;
  return px >= std::min(a.x, b.x) && px <= std::max(a.x, b.x) &&
         py >= std::min(a.y, b.y) && py <= std::max(a.y, b.y);
}

bool PointInPolygon(double x, double y, const std::vector<Vec2d>& polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (PointOnSegment(x, y, polygon[j], polygon[i])) return true;
    const bool crosses = ((polygon[i].y > y) != (polygon[j].y > y)) &&
                         (x < (polygon[j].x - polygon[i].x) *
                                      (y - polygon[i].y) /
                                      (polygon[j].y - polygon[i].y) +
                                  polygon[i].x);
    if (crosses) inside = !inside;
  }
  return inside;
}

Result<Image<std::uint8_t>> LoadPolygonMask(
    const std::filesystem::path& path, int width, int height) {
  std::ifstream stream(path);
  if (!stream) {
    return Status::Error(ErrorCode::kIo,
                         "Cannot open polygon label: " + path.string());
  }
  std::vector<double> tokens((std::istream_iterator<double>(stream)),
                             std::istream_iterator<double>());
  if (tokens.size() < 7 || (tokens.size() - 1) % 2 != 0) {
    return Status::Error(
        ErrorCode::kUnsupportedFormat,
        "Polygon label must be: class x1 y1 x2 y2 x3 y3 ...");
  }
  std::vector<Vec2d> polygon;
  for (std::size_t i = 1; i < tokens.size(); i += 2) {
    // NumPy's cast to int32 truncates these scaled non-negative coordinates.
    polygon.push_back({static_cast<double>(static_cast<int>(tokens[i] * width)),
                       static_cast<double>(
                           static_cast<int>(tokens[i + 1] * height))});
  }
  Image<std::uint8_t> mask(height, width, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      mask(y, x) = PointInPolygon(x, y, polygon) ? 1 : 0;
    }
  }
  return mask;
}

}  // namespace

DatasetDataSource::DatasetDataSource(std::vector<DatasetFramePaths> frames)
    : frames_(std::move(frames)) {}

Result<DatasetDataSource> DatasetDataSource::FromPair(
    const std::filesystem::path& disparity,
    const std::filesystem::path& label, bool use_label) {
  if (!std::filesystem::is_regular_file(disparity)) {
    return Status::Error(ErrorCode::kIo,
                         "Disparity file does not exist: " + disparity.string());
  }
  if (use_label && !std::filesystem::is_regular_file(label)) {
    return Status::Error(ErrorCode::kIo,
                         "Label file does not exist: " + label.string());
  }
  return DatasetDataSource({{disparity, label, use_label}});
}

Result<DatasetDataSource> DatasetDataSource::Discover(
    const std::filesystem::path& dataset_root, bool use_labels) {
  if (!std::filesystem::is_directory(dataset_root)) {
    return Status::Error(ErrorCode::kIo,
                         "Dataset root does not exist: " +
                             dataset_root.string());
  }
  std::vector<std::filesystem::path> categories;
  if (std::filesystem::is_directory(dataset_root / "disparity")) {
    categories.push_back(dataset_root);
  }
  for (const auto& entry : std::filesystem::directory_iterator(dataset_root)) {
    if (entry.is_directory() &&
        std::filesystem::is_directory(entry.path() / "disparity")) {
      categories.push_back(entry.path());
    }
  }
  std::sort(categories.begin(), categories.end());
  std::vector<DatasetFramePaths> frames;
  for (const std::filesystem::path& category : categories) {
    std::vector<std::filesystem::path> disparities;
    for (const auto& entry :
         std::filesystem::directory_iterator(category / "disparity")) {
      if (entry.is_regular_file() && entry.path().extension() == ".npy") {
        disparities.push_back(entry.path());
      }
    }
    std::sort(disparities.begin(), disparities.end());
    for (const std::filesystem::path& disparity : disparities) {
      std::filesystem::path label = category / "labels" /
                                    (disparity.stem().string() + ".txt");
      if (use_labels && !std::filesystem::is_regular_file(label)) continue;
      frames.push_back({disparity, label, use_labels});
    }
  }
  if (frames.empty()) {
    return Status::Error(
        ErrorCode::kIo,
        "No complete disparity/label pairs found below " +
            dataset_root.string());
  }
  return DatasetDataSource(std::move(frames));
}

Status DatasetDataSource::Next(FrameInput* frame) {
  if (frame == nullptr) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "DatasetDataSource::Next received a null frame");
  }
  if (next_index_ >= frames_.size()) {
    return Status::Error(ErrorCode::kEndOfStream, "Dataset exhausted");
  }
  const DatasetFramePaths& paths = frames_[next_index_++];
  Result<Image<float>> disparity = LoadNpyDisparity(paths.disparity);
  if (!disparity.ok()) return disparity.status();
  frame->disparity = std::move(disparity).value();
  frame->has_labels = paths.has_label;
  if (paths.has_label) {
    Result<Image<std::uint8_t>> labels = LoadLabelMask(
        paths.label, frame->disparity.width(), frame->disparity.height());
    if (!labels.ok()) return labels.status();
    frame->labels = std::move(labels).value();
  } else {
    frame->labels = {};
  }
  frame->name = paths.disparity.stem().string();
  frame->frame_id = "camera";
  std::error_code timestamp_error;
  const auto timestamp =
      std::filesystem::last_write_time(paths.disparity, timestamp_error);
  if (timestamp_error) {
    frame->timestamp_ns = 0;
  } else {
    const auto system_time = std::chrono::time_point_cast<std::chrono::nanoseconds>(
        timestamp - std::filesystem::file_time_type::clock::now() +
        std::chrono::system_clock::now());
    frame->timestamp_ns = system_time.time_since_epoch().count();
  }
  return Status::Ok();
}

Result<Image<float>> LoadNpyDisparity(const std::filesystem::path& path) {
  Result<NpyData> source_result = ReadNpy(path);
  if (!source_result.ok()) return source_result.status();
  NpyData source = std::move(source_result).value();
  Image<float> output(source.height, source.width, 0.0F);
  for (std::size_t i = 0; i < output.size(); ++i) {
    if (source.descriptor.find("f4") != std::string::npos) {
      output[i] = ReadElement<float>(source, i);
    } else if (source.descriptor.find("f8") != std::string::npos) {
      output[i] = static_cast<float>(ReadElement<double>(source, i));
    } else {
      return Status::Error(ErrorCode::kUnsupportedFormat,
                           "Disparity NPY must have float32 or float64 dtype");
    }
  }
  return output;
}

Result<Image<std::uint8_t>> LoadLabelMask(const std::filesystem::path& path,
                                         int width, int height) {
  if (path.extension() == ".txt") return LoadPolygonMask(path, width, height);
  if (path.extension() != ".npy") {
    return Status::Error(ErrorCode::kUnsupportedFormat,
                         "Label must be a polygon TXT or 2D NPY mask");
  }
  Result<NpyData> source_result = ReadNpy(path);
  if (!source_result.ok()) return source_result.status();
  NpyData source = std::move(source_result).value();
  if (source.height != height || source.width != width) {
    return Status::Error(ErrorCode::kShapeMismatch,
                         "Label NPY and disparity shapes differ");
  }
  Image<std::uint8_t> output(height, width, 0);
  for (std::size_t i = 0; i < output.size(); ++i) {
    if (source.descriptor == "|u1" || source.descriptor == "|i1" ||
        source.descriptor == "|b1" || source.descriptor == "?") {
      output[i] = source.bytes[i] != 0;
    } else if (source.descriptor.find("i4") != std::string::npos) {
      output[i] = ReadElement<std::int32_t>(source, i) != 0;
    } else if (source.descriptor.find("u2") != std::string::npos) {
      output[i] = ReadElement<std::uint16_t>(source, i) != 0;
    } else {
      return Status::Error(ErrorCode::kUnsupportedFormat,
                           "Unsupported label NPY dtype: " +
                               source.descriptor);
    }
  }
  return output;
}

}  // namespace axis_grasp
