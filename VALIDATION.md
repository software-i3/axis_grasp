# Validation

## ROI crop equivalence

The crop was tested on the three supplied 800x600 pairs: `000095`, `000098`,
and `000101`. Each ran with cropping enabled and disabled for both strategies.

| Strategy | Frame | Pose rows equal | Dense-point rows equal | Maximum numeric difference |
| --- | --- | --- | --- | ---: |
| camera | 000095 | yes (1053) | yes (96) | 0 |
| camera | 000098 | yes (994) | yes (130) | 0 |
| camera | 000101 | yes (142) | yes (35) | 0 |
| postskel | 000095 | yes (1053) | yes (96) | 0 |
| postskel | 000098 | yes (994) | yes (130) | 0 |
| postskel | 000101 | yes (142) | yes (35) | 0 |

The comparison covered pixel coordinates, 3D positions, quaternions, and every
rotation-matrix element. Agreement was 100% on these samples, exceeding 98%.

**The optical-frame change does not invalidate this harness.** It compares a
cropped run against an uncropped run of the *same* build, and the convention
change is a rotation about the camera origin: it preserves distances, and it acts
on both sides of the comparison identically, so an equivalence that held before
still holds. What does need re-baselining is a comparison against output *stored*
before the change — an older CSV, a replayed bag, a recorded `/grasp_poses`. Map
those onto the current convention with `y, z -> -y, -z` and rotation rows 1 and 2
negated (a quaternion goes by the shuffle `(qx, qy, qz, qw) -> (qw, -qz, qy,
-qx)`); `x` is unchanged. `|z|` is still the depth either side reported.

The crop-equivalence harness used the established scalar reference image
operators to isolate the crop transformation. Total times were 153-198 ms
full-frame and 34-55 ms cropped. The delivered backend uses equivalent OpenCV
primitives; final timing depends on CPU, OpenCV, mask size, and scene gradients.

## On-target check

Run deployment data once with `enable_roi_crop: false` and once with it enabled,
then compare `/grasp_poses`. Keep automatic padding (`-1`) unless a smaller
value also passes that comparison.
