#ifndef AXIS_GRASP_MASK_PAIRING_H_
#define AXIS_GRASP_MASK_PAIRING_H_

#include <cstdint>

namespace axis_grasp_ros1 {

// Which clock the ROI source is synchronized on, and therefore what "no ROI yet"
// means. Both sources fall back to whole-frame voting once their grace period
// expires, but they disagree about a source that has already arrived.
enum class RoiSource {
  // A mono8 label image paired by header stamp. A label that arrived too early
  // is retained, because the disparity it belongs to may simply not have been
  // published yet.
  kLabelMask,
  // DetectedInstances contours paired by local receipt time. The source always
  // arrives after the disparity it applies to, so a past-stamped entry is
  // genuinely stale and a future one means a newer disparity is in flight.
  kDetections,
};

// Next action for a disparity waiting for a pairable ROI.
enum class MaskPairingAction {
  kWait,
  kPair,
  kDiscardStaleMask,
  kFallbackTimeout,
  kFallbackMaskNewer,
};

// Decides how to handle one pending disparity. Stamps and durations are in
// nanoseconds and the durations must be non-negative. A ROI exactly
// sync_slop_ns away is pairable.
//
// mask_is_expected: the ROI source is known to be ahead of the pending
//   disparity, so it has not had a chance to arrive yet. That is true only
//   while waiting for a source that policy already deferred to this disparity.
MaskPairingAction DecideMaskPairing(RoiSource source, bool has_mask,
                                    std::int64_t disparity_stamp_ns,
                                    std::int64_t mask_stamp_ns,
                                    std::int64_t elapsed_wait_ns,
                                    std::int64_t sync_slop_ns,
                                    std::int64_t wait_timeout_ns,
                                    bool mask_is_expected = false);

}  // namespace axis_grasp_ros1

#endif  // AXIS_GRASP_MASK_PAIRING_H_
