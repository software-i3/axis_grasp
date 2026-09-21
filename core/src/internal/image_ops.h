#ifndef AXIS_GRASP_CORE_SRC_INTERNAL_IMAGE_OPS_H_
#define AXIS_GRASP_CORE_SRC_INTERNAL_IMAGE_OPS_H_

#include <cstdint>
#include <vector>

#include "axis_grasp/core/image.h"
#include "axis_grasp/core/math_types.h"

namespace axis_grasp::internal {

struct ConnectedComponents {
  Image<int> labels;
  std::vector<int> area;
  std::vector<int> left;
  std::vector<int> top;
  std::vector<int> width;
  std::vector<int> height;
};

double Percentile(std::vector<double> values, double percentile);
double Percentile(std::vector<float> values, double percentile);

Image<float> BilateralFilter(const Image<float>& source, int diameter,
                             double sigma_color, double sigma_space);
void Sobel(const Image<float>& source, Image<float>* dx, Image<float>* dy);

Image<std::uint8_t> Dilate(const Image<std::uint8_t>& source, int kernel_size);
Image<std::uint8_t> Erode(const Image<std::uint8_t>& source, int kernel_size);
Image<std::uint8_t> Close(const Image<std::uint8_t>& source, int kernel_size);
ConnectedComponents LabelConnected(const Image<std::uint8_t>& mask,
                                   int connectivity);

}  // namespace axis_grasp::internal

#endif  // AXIS_GRASP_CORE_SRC_INTERNAL_IMAGE_OPS_H_
