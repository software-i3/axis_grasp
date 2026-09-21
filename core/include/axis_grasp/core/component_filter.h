#ifndef AXIS_GRASP_CORE_COMPONENT_FILTER_H_
#define AXIS_GRASP_CORE_COMPONENT_FILTER_H_

#include <cstdint>
#include <vector>

#include "axis_grasp/core/config.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {

struct ComponentOutput {
  Image<std::uint8_t> threshold_mask;
  Image<std::uint8_t> closed_mask;
  Image<int> labels;
  std::vector<double> weights;
  std::vector<int> kept_labels;
};

// Ports utils/voting.py::remove_straight_components.
Result<Image<float>> RemoveStraightComponents(
    const Image<float>& coherent, int close_kernel,
    const RopeFilterConfig& config = {});

// Ports accumulator_mask_at_percentile, accumulator_components,
// component_weights, and keep_top_components without changing their gates.
Result<ComponentOutput> FilterAccumulator(const Image<float>& accumulator,
                                          const ComponentConfig& config);

// LUT-driven Zhang-Suen thinning matching
// skimage.morphology.skeletonize(method="zhang"), including zero padding at
// the image boundary. Any nonzero input is foreground; output values are 0/1.
Image<std::uint8_t> Skeletonize(const Image<std::uint8_t>& mask);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_COMPONENT_FILTER_H_
