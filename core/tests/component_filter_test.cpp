#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "axis_grasp/core/component_filter.h"
#include "axis_grasp/core/config.h"
#include "axis_grasp/core/image.h"

namespace axis_grasp {
namespace {

TEST(ComponentFilterTest, KeepsHighestVoteMass) {
  Image<float> accumulator(20, 30, 0.0F);
  for (int y = 3; y <= 6; ++y) {
    for (int x = 3; x <= 6; ++x) accumulator(y, x) = 10.0F;
  }
  for (int y = 12; y <= 15; ++y) {
    for (int x = 20; x <= 23; ++x) accumulator(y, x) = 4.0F;
  }
  accumulator(0, 0) = 3.0F;
  ComponentConfig config;
  config.accumulator_percentile = 0.0;
  config.close_kernel = 1;
  config.top_components = 1;
  Result<ComponentOutput> result = FilterAccumulator(accumulator, config);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_EQ(result.value().kept_labels.size(), 1U);
  EXPECT_EQ(result.value().labels(4, 4), result.value().kept_labels[0]);
}

TEST(ComponentFilterTest, SkeletonizesThickBar) {
  Image<std::uint8_t> mask(25, 35, 0);
  for (int y = 10; y <= 14; ++y) {
    for (int x = 5; x <= 29; ++x) mask(y, x) = 1;
  }
  Image<std::uint8_t> skeleton = Skeletonize(mask);
  int count = 0;
  for (std::uint8_t value : skeleton.data()) count += value != 0;
  EXPECT_GT(count, 15);
  EXPECT_LT(count, 30);
}

TEST(ComponentFilterTest, MatchesSkimageZhangEllipseExample) {
  Image<std::uint8_t> mask(9, 9, 0);
  for (int y = 0; y < 9; ++y) {
    for (int x = 0; x < 9; ++x) {
      const double dy = static_cast<double>(y - 4);
      const double dx = static_cast<double>(x - 4);
      mask(y, x) = ((dy * dy) / 3.0 + dx * dx) < 9.0 ? 1 : 0;
    }
  }

  const Image<std::uint8_t> skeleton = Skeletonize(mask);
  for (int y = 0; y < 9; ++y) {
    for (int x = 0; x < 9; ++x) {
      const bool expected = x == 4 && y >= 3 && y <= 6;
      EXPECT_EQ(skeleton(y, x) != 0, expected) << "at (" << x << ", "
                                                << y << ')';
    }
  }
}

TEST(ComponentFilterTest, MatchesSkimageAtImageBoundary) {
  const Image<std::uint8_t> mask(5, 5, 1);
  const Image<std::uint8_t> skeleton = Skeletonize(mask);
  const std::vector<Vec2i> expected{{2, 1}, {3, 1}, {1, 2}, {1, 3}};
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 5; ++x) {
      bool should_be_set = false;
      for (const Vec2i& point : expected) {
        should_be_set |= point.x == x && point.y == y;
      }
      EXPECT_EQ(skeleton(y, x) != 0, should_be_set) << "at (" << x << ", "
                                                     << y << ')';
    }
  }
}

TEST(ComponentFilterTest, MatchesSkimageDeterministicCorpus) {
  constexpr int kHeight = 17;
  constexpr int kWidth = 19;
  constexpr int kCases = 64;
  std::uint32_t state = 0x12345678U;
  std::uint64_t hash = 14695981039346656037ULL;
  std::size_t input_pixels = 0;
  std::size_t output_pixels = 0;

  for (int sample = 0; sample < kCases; ++sample) {
    Image<std::uint8_t> raw(kHeight, kWidth, 0);
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        state = state * 1664525U + 1013904223U;
        raw(y, x) = (state >> 24U) < 144U ? 1 : 0;
      }
    }

    Image<std::uint8_t> mask(kHeight, kWidth, 0);
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        int count = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int yy = y + dy;
            const int xx = x + dx;
            if (yy >= 0 && yy < kHeight && xx >= 0 && xx < kWidth) {
              count += raw(yy, xx) != 0;
            }
          }
        }
        mask(y, x) = count >= 5 ? 1 : 0;
        input_pixels += mask(y, x) != 0;
      }
    }

    const Image<std::uint8_t> skeleton = Skeletonize(mask);
    for (std::uint8_t value : skeleton.data()) {
      const std::uint8_t bit = value != 0 ? 1 : 0;
      output_pixels += bit;
      hash ^= bit;
      hash *= 1099511628211ULL;
    }
  }

  EXPECT_EQ(input_pixels, 11298U);
  EXPECT_EQ(output_pixels, 4061U);
  EXPECT_EQ(hash, 0xd1cdd8bb4aacad3eULL);
}

TEST(ComponentFilterTest, RemovesLongStraightBand) {
  Image<float> coherent(35, 110, 0.0F);
  for (int y = 15; y <= 19; ++y) {
    for (int x = 10; x <= 99; ++x) coherent(y, x) = 1.0F;
  }
  Result<Image<float>> result = RemoveStraightComponents(coherent, 1);
  ASSERT_TRUE(result.ok()) << result.status().message;
  double mass = 0.0;
  for (float value : result.value().data()) mass += value;
  EXPECT_DOUBLE_EQ(mass, 0.0);
}

TEST(ComponentFilterTest, ZeroesEveryFragmentOfASplitStraightRope) {
  // A low-value gap near an attachment point splits the rope into two raw
  // coherent>0 fragments that closing bridges into a single detection. Clearing
  // only the fragment holding the band's first raster-scanned pixel left the
  // rest of the rope in the output.
  Image<float> coherent(35, 120, 0.0F);
  for (int y = 15; y <= 19; ++y) {
    for (int x = 5; x <= 104; ++x) coherent(y, x) = 1.0F;
  }
  for (int y = 15; y <= 19; ++y) {
    for (int x = 54; x <= 57; ++x) coherent(y, x) = 0.0F;
  }
  Result<Image<float>> result = RemoveStraightComponents(coherent, 9);
  ASSERT_TRUE(result.ok()) << result.status().message;
  double mass = 0.0;
  for (float value : result.value().data()) mass += value;
  EXPECT_DOUBLE_EQ(mass, 0.0);
}

TEST(ComponentFilterTest, RemovesStraightRopeMergedWithThinClutter) {
  // Closing can bridge a straight rope into unrelated nearby clutter. The
  // fragment is a minority of the band's pixels but a large share of its
  // skeleton, so a whole-population PCA reads the merged shape as bent and
  // spares the rope; RANSAC still recovers the rope line as the consensus.
  Image<float> coherent(60, 120, 0.0F);
  for (int y = 15; y <= 19; ++y) {
    for (int x = 5; x <= 104; ++x) coherent(y, x) = 1.0F;
  }
  for (int y = 20; y <= 39; ++y) coherent(y, 97) = 1.0F;
  Result<Image<float>> result = RemoveStraightComponents(coherent, 1);
  ASSERT_TRUE(result.ok()) << result.status().message;
  double mass = 0.0;
  for (float value : result.value().data()) mass += value;
  EXPECT_DOUBLE_EQ(mass, 0.0);
}

TEST(ComponentFilterTest, KeepsShortCurvedFragment) {
  // A short chord of a loop is straight over its own span. The skeleton length
  // gate keeps it, where the baseline deleted it as if it were rope.
  Image<float> coherent(40, 60, 0.0F);
  for (int x = 0; x <= 40; ++x) {
    const int dy = static_cast<int>(std::lround(0.0075 * (x - 20) * (x - 20)));
    for (int y = 20 + dy - 1; y <= 20 + dy + 1; ++y) coherent(y, x) = 1.0F;
  }
  Result<Image<float>> result = RemoveStraightComponents(coherent, 1);
  ASSERT_TRUE(result.ok()) << result.status().message;
  double mass = 0.0;
  for (float value : result.value().data()) mass += value;
  EXPECT_GT(mass, 0.0);
}

}  // namespace
}  // namespace axis_grasp
