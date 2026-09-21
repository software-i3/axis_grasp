#include "internal/image_ops.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace axis_grasp::internal {
namespace {

template <typename T>
double PercentileImpl(std::vector<T> values, double percentile) {
  if (values.empty()) return 0.0;
  const double bounded = std::clamp(percentile, 0.0, 100.0);
  const double rank = bounded * 0.01 * static_cast<double>(values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(rank));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(rank));
  const double fraction = rank - static_cast<double>(lower);
  std::nth_element(values.begin(), values.begin() + lower, values.end());
  const double lower_value = values[lower];
  if (upper == lower) return lower_value;
  std::nth_element(values.begin(), values.begin() + upper, values.end());
  return (1.0 - fraction) * lower_value + fraction * values[upper];
}

cv::Mat FloatView(const Image<float>& image) {
  return cv::Mat(image.height(), image.width(), CV_32FC1,
                 const_cast<float*>(image.data().data()));
}

cv::Mat ByteView(const Image<std::uint8_t>& image) {
  return cv::Mat(image.height(), image.width(), CV_8UC1,
                 const_cast<std::uint8_t*>(image.data().data()));
}

Image<float> CopyFloat(const cv::Mat& matrix) {
  Image<float> output(matrix.rows, matrix.cols, 0.0F);
  cv::Mat destination(matrix.rows, matrix.cols, CV_32FC1,
                      output.data().data());
  matrix.copyTo(destination);
  return output;
}

Image<std::uint8_t> CopyByte(const cv::Mat& matrix) {
  Image<std::uint8_t> output(matrix.rows, matrix.cols, 0);
  cv::Mat destination(matrix.rows, matrix.cols, CV_8UC1,
                      output.data().data());
  matrix.copyTo(destination);
  return output;
}

cv::Mat RectKernel(int kernel_size) {
  return cv::getStructuringElement(cv::MORPH_RECT,
                                   cv::Size(kernel_size, kernel_size));
}

}  // namespace

double Percentile(std::vector<double> values, double percentile) {
  return PercentileImpl(std::move(values), percentile);
}

double Percentile(std::vector<float> values, double percentile) {
  return PercentileImpl(std::move(values), percentile);
}

Image<float> BilateralFilter(const Image<float>& source, int diameter,
                             double sigma_color, double sigma_space) {
  if (diameter <= 1 || sigma_color <= 0.0 || sigma_space <= 0.0) return source;
  cv::Mat filtered;
  cv::bilateralFilter(FloatView(source), filtered, diameter, sigma_color,
                      sigma_space, cv::BORDER_REFLECT_101);
  return CopyFloat(filtered);
}

void Sobel(const Image<float>& source, Image<float>* dx, Image<float>* dy) {
  cv::Mat gradient_x;
  cv::Mat gradient_y;
  cv::Sobel(FloatView(source), gradient_x, CV_32F, 1, 0, 3, 1.0, 0.0,
            cv::BORDER_REFLECT_101);
  cv::Sobel(FloatView(source), gradient_y, CV_32F, 0, 1, 3, 1.0, 0.0,
            cv::BORDER_REFLECT_101);
  *dx = CopyFloat(gradient_x);
  *dy = CopyFloat(gradient_y);
}

Image<std::uint8_t> Dilate(const Image<std::uint8_t>& source,
                           int kernel_size) {
  if (kernel_size <= 1) return source;
  cv::Mat result;
  cv::dilate(ByteView(source), result, RectKernel(kernel_size),
             cv::Point(-1, -1), 1, cv::BORDER_CONSTANT,
             cv::morphologyDefaultBorderValue());
  return CopyByte(result);
}

Image<std::uint8_t> Erode(const Image<std::uint8_t>& source,
                          int kernel_size) {
  if (kernel_size <= 1) return source;
  cv::Mat result;
  cv::erode(ByteView(source), result, RectKernel(kernel_size),
            cv::Point(-1, -1), 1, cv::BORDER_CONSTANT,
            cv::morphologyDefaultBorderValue());
  return CopyByte(result);
}

Image<std::uint8_t> Close(const Image<std::uint8_t>& source,
                          int kernel_size) {
  if (kernel_size <= 1) return source;
  cv::Mat result;
  cv::morphologyEx(ByteView(source), result, cv::MORPH_CLOSE,
                   RectKernel(kernel_size), cv::Point(-1, -1), 1,
                   cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
  return CopyByte(result);
}

ConnectedComponents LabelConnected(const Image<std::uint8_t>& mask,
                                   int connectivity) {
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int count = cv::connectedComponentsWithStats(
      ByteView(mask), labels, stats, centroids, connectivity, CV_32S);

  ConnectedComponents output;
  output.labels = Image<int>(mask.height(), mask.width(), 0);
  cv::Mat label_destination(mask.height(), mask.width(), CV_32SC1,
                            output.labels.data().data());
  labels.copyTo(label_destination);
  output.area.resize(count);
  output.left.resize(count);
  output.top.resize(count);
  output.width.resize(count);
  output.height.resize(count);
  for (int label = 0; label < count; ++label) {
    output.area[label] = stats.at<int>(label, cv::CC_STAT_AREA);
    output.left[label] = stats.at<int>(label, cv::CC_STAT_LEFT);
    output.top[label] = stats.at<int>(label, cv::CC_STAT_TOP);
    output.width[label] = stats.at<int>(label, cv::CC_STAT_WIDTH);
    output.height[label] = stats.at<int>(label, cv::CC_STAT_HEIGHT);
  }
  return output;
}

}  // namespace axis_grasp::internal
