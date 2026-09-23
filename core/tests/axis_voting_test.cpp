#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "axis_grasp/core/axis_voting.h"
#include "axis_grasp/core/config.h"
#include "axis_grasp/core/image.h"

namespace axis_grasp {
namespace {

TEST(AxisVotingTest, OpposedGradientsConvergeAtRidge) {
  Image<float> disparity(41, 41, 0.0F);
  Image<std::uint8_t> roi(41, 41, 1);
  for (int y = 0; y < disparity.height(); ++y) {
    for (int x = 0; x < disparity.width(); ++x) {
      disparity(y, x) = static_cast<float>(20 - std::abs(x - 20));
    }
  }
  VotingConfig config;
  config.radius_min = 1;
  config.radius_max = 12;
  config.magnitude_percentile = 0.0;
  config.bilateral_diameter = 1;
  config.remove_straight_rope = false;
  Result<VotingOutput> result =
      AccumulateDirectionalVotes(disparity, &roi, config);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_GT(result.value().accumulator(20, 20), 0.0F);
  EXPECT_GT(result.value().accumulator(20, 20),
            result.value().accumulator(20, 2));
}

TEST(AxisVotingTest, FullFrameMatchesAllOnesRoi) {
  Image<float> disparity(41, 41, 0.0F);
  Image<std::uint8_t> roi(41, 41, 1);
  for (int y = 0; y < disparity.height(); ++y) {
    for (int x = 0; x < disparity.width(); ++x) {
      disparity(y, x) = static_cast<float>(20 - std::abs(x - 20));
    }
  }
  VotingConfig config;
  config.radius_min = 1;
  config.radius_max = 12;
  config.magnitude_percentile = 0.0;
  config.bilateral_diameter = 1;
  config.remove_straight_rope = false;

  Result<VotingOutput> full_frame =
      AccumulateDirectionalVotes(disparity, nullptr, config);
  Result<VotingOutput> all_ones =
      AccumulateDirectionalVotes(disparity, &roi, config);
  ASSERT_TRUE(full_frame.ok()) << full_frame.status().message;
  ASSERT_TRUE(all_ones.ok()) << all_ones.status().message;
  ASSERT_EQ(full_frame.value().accumulator.size(),
            all_ones.value().accumulator.size());
  for (std::size_t i = 0; i < full_frame.value().accumulator.size(); ++i) {
    EXPECT_FLOAT_EQ(full_frame.value().accumulator[i],
                    all_ones.value().accumulator[i]);
  }
}

TEST(AxisVotingTest, RejectsOddDirectionBinCount) {
  Image<float> disparity(8, 8, 1.0F);
  VotingConfig config;
  config.direction_bins = 7;
  Result<VotingOutput> result =
      AccumulateDirectionalVotes(disparity, nullptr, config);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kInvalidArgument);
}

}  // namespace
}  // namespace axis_grasp
