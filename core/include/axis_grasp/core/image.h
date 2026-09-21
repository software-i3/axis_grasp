#ifndef AXIS_GRASP_CORE_IMAGE_H_
#define AXIS_GRASP_CORE_IMAGE_H_

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace axis_grasp {

template <typename T>
class Image {
 public:
  Image() = default;
  Image(int height, int width) : height_(height), width_(width), data_(Size(height, width)) {}
  Image(int height, int width, const T& value)
      : height_(height), width_(width), data_(Size(height, width), value) {}
  Image(int height, int width, std::vector<T> data)
      : height_(height), width_(width), data_(std::move(data)) {
    if (data_.size() != Size(height, width)) {
      throw std::invalid_argument("Image data size does not match its shape");
    }
  }

  int height() const { return height_; }
  int width() const { return width_; }
  bool empty() const { return data_.empty(); }
  std::size_t size() const { return data_.size(); }

  T& operator()(int y, int x) {
    return data_[static_cast<std::size_t>(y) * width_ + x];
  }
  const T& operator()(int y, int x) const {
    return data_[static_cast<std::size_t>(y) * width_ + x];
  }
  T& operator[](std::size_t index) { return data_[index]; }
  const T& operator[](std::size_t index) const { return data_[index]; }

  std::vector<T>& data() { return data_; }
  const std::vector<T>& data() const { return data_; }

 private:
  static std::size_t Size(int height, int width) {
    if (height < 0 || width < 0) {
      throw std::invalid_argument("Image dimensions must be non-negative");
    }
    return static_cast<std::size_t>(height) * static_cast<std::size_t>(width);
  }

  int height_ = 0;
  int width_ = 0;
  std::vector<T> data_;
};

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_IMAGE_H_
