#include "axis_grasp/mask_pairing.h"

#include <cstdint>

namespace axis_grasp_ros1 {
namespace {

std::uint64_t NonNegativeDifference(std::int64_t newer,
                                    std::int64_t older) {
  return static_cast<std::uint64_t>(newer) -
         static_cast<std::uint64_t>(older);
}

}  // namespace

MaskPairingAction DecideMaskPairing(RoiSource source, bool has_mask,
                                    std::int64_t disparity_stamp_ns,
                                    std::int64_t mask_stamp_ns,
                                    std::int64_t elapsed_wait_ns,
                                    std::int64_t sync_slop_ns,
                                    std::int64_t wait_timeout_ns,
                                    bool mask_is_expected) {
  if (has_mask) {
    const std::uint64_t slop = static_cast<std::uint64_t>(sync_slop_ns);
    if (disparity_stamp_ns >= mask_stamp_ns) {
      // The ROI source is behind this disparity. It is only stale if policy is
      // not already holding it as this disparity's intended partner.
      if (!mask_is_expected &&
          NonNegativeDifference(disparity_stamp_ns, mask_stamp_ns) > slop) {
        return MaskPairingAction::kDiscardStaleMask;
      }
    } else if (NonNegativeDifference(mask_stamp_ns, disparity_stamp_ns) > slop) {
      if (mask_is_expected) {
        // An earlier fallback deferred this source to the disparity now
        // pending, so it is this frame's partner rather than a stale leftover.
        return MaskPairingAction::kPair;
      }
      if (source == RoiSource::kDetections) {
        // The source is a receipt time: this entry belongs to a disparity that
        // arrived after the pending one, and one disparity can only take one
        // ROI. Wait for one that actually targets this frame.
        return MaskPairingAction::kWait;
      }
      // A label stamped ahead of this disparity belongs to the next one, which
      // has not been published yet.
      return MaskPairingAction::kFallbackMaskNewer;
    }
    return MaskPairingAction::kPair;
  }
  if (mask_is_expected) {
    // A partner was deliberately held back; wait for it regardless of the
    // deadline that was set when the disparity itself first arrived.
    return MaskPairingAction::kWait;
  }
  return elapsed_wait_ns >= wait_timeout_ns
             ? MaskPairingAction::kFallbackTimeout
             : MaskPairingAction::kWait;
}

}  // namespace axis_grasp_ros1
