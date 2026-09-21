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

The crop-equivalence harness used the established scalar reference image
operators to isolate the crop transformation. Total times were 153-198 ms
full-frame and 34-55 ms cropped. The delivered backend uses equivalent OpenCV
primitives; final timing depends on CPU, OpenCV, mask size, and scene gradients.

## On-target check

Run deployment data once with `enable_roi_crop: false` and once with it enabled,
then compare `/grasp_poses`. Keep automatic padding (`-1`) unless a smaller
value also passes that comparison.
