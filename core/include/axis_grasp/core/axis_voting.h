#ifndef AXIS_GRASP_CORE_AXIS_VOTING_H_
#define AXIS_GRASP_CORE_AXIS_VOTING_H_

#include <cstdint>

#include "axis_grasp/core/config.h"
#include "axis_grasp/core/image.h"
#include "axis_grasp/core/status.h"

namespace axis_grasp {

struct VotingOutput {
  Image<float> accumulator;
  Image<float> filtered_disparity;
};

// Ports utils/voting.py::accumulate_votes_directional. The optional ROI is a
// full-resolution binary image: nonzero pixels may cast votes.
Result<VotingOutput> AccumulateDirectionalVotes(
    const Image<float>& disparity, const Image<std::uint8_t>* roi,
    const VotingConfig& config);

}  // namespace axis_grasp

#endif  // AXIS_GRASP_CORE_AXIS_VOTING_H_
