#include <gtest/gtest.h>

#include "axis_grasp/mask_pairing.h"

namespace axis_grasp_ros1 {
namespace {

constexpr std::int64_t kDisparityStamp = 1'000'000'000;
constexpr std::int64_t kSlop = 30'000'000;
constexpr std::int64_t kTimeout = 100'000'000;

// Both sources share the same deadline behavior; only their handling of a
// present-but-misaligned source differs. Running the shared cases over both
// keeps that guarantee checked rather than assumed.
constexpr RoiSource kSources[] = {RoiSource::kLabelMask, RoiSource::kDetections};

TEST(MaskPairingTest, WaitsForMissingSourceBeforeDeadline) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, false, kDisparityStamp, 0, kTimeout - 1,
                                kSlop, kTimeout),
              MaskPairingAction::kWait);
  }
}

TEST(MaskPairingTest, FallsBackAtDeadline) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, false, kDisparityStamp, 0, kTimeout,
                                kSlop, kTimeout),
              MaskPairingAction::kFallbackTimeout);
  }
}

TEST(MaskPairingTest, ZeroTimeoutFallsBackWithoutSource) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, false, kDisparityStamp, 0, 0, kSlop, 0),
              MaskPairingAction::kFallbackTimeout);
  }
}

TEST(MaskPairingTest, CompatibleSourceWinsAtDeadline) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, true, kDisparityStamp,
                                kDisparityStamp + kSlop, kTimeout, kSlop,
                                kTimeout),
              MaskPairingAction::kPair);
  }
}

TEST(MaskPairingTest, SourceExactlyAtNegativeSlopBoundaryPairs) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, true, kDisparityStamp,
                                kDisparityStamp - kSlop, 0, kSlop, kTimeout),
              MaskPairingAction::kPair);
  }
}

TEST(MaskPairingTest, DiscardsSourceOlderThanSlop) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, true, kDisparityStamp,
                                kDisparityStamp - kSlop - 1, 0, kSlop,
                                kTimeout),
              MaskPairingAction::kDiscardStaleMask);
  }
}

TEST(MaskPairingTest, NewerLabelMakesPendingDisparityUnpairable) {
  EXPECT_EQ(DecideMaskPairing(RoiSource::kLabelMask, true, kDisparityStamp,
                              kDisparityStamp + kSlop + 1, 0, kSlop, kTimeout),
            MaskPairingAction::kFallbackMaskNewer);
}

TEST(MaskPairingTest, NewerDetectionsWaitForTheDisparityTheyTarget) {
  // A detection receipt time only ever trails its own disparity, so one stamped
  // ahead of the pending disparity targets a later frame. That frame can still
  // satisfy both sides, so no fallback is warranted yet.
  EXPECT_EQ(DecideMaskPairing(RoiSource::kDetections, true, kDisparityStamp,
                              kDisparityStamp + kSlop + 1, kTimeout, kSlop,
                              kTimeout),
            MaskPairingAction::kWait);
}

TEST(MaskPairingTest, RetainedFutureLabelPairsWithFollowingDisparity) {
  // The previous disparity was emitted unlabeled and this label was held for
  // the next one. Against that next disparity it is still ahead by more than
  // the slop, so without the expectation flag it would be discarded forever.
  const std::int64_t mask_stamp = kDisparityStamp + kSlop + 1;
  EXPECT_EQ(DecideMaskPairing(RoiSource::kLabelMask, true, mask_stamp,
                              mask_stamp, 0, kSlop, kTimeout),
            MaskPairingAction::kPair);
}

TEST(MaskPairingTest, RetainedSourceWaitsPastTheOriginalDeadline) {
  for (RoiSource source : kSources) {
    EXPECT_EQ(DecideMaskPairing(source, false, kDisparityStamp, 0,
                                kTimeout * 10, kSlop, kTimeout,
                                /*mask_is_expected=*/true),
              MaskPairingAction::kWait);
  }
}

}  // namespace
}  // namespace axis_grasp_ros1
