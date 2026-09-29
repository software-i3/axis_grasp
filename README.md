# axis_grasp

Standalone ROS1/catkin package for disparity-based axis voting and grasp-pose
proposal. It uses OpenCV for optimized image processing while retaining the
custom scikit-image-compatible Zhang thinning implementation.

## Features

- OpenCV bilateral filtering, Sobel, morphology, and connected components.
- Custom lookup-table Zhang thinning; OpenCV thinning is intentionally unused.
- Label-bounding-box processing with a conservative automatic halo.
- Runtime ROS YAML configuration. The ROI mask comes either from a `/label`
  image or from `bx_msgs/DetectedInstances` contours.
- Release mode whenever catkin/CMake does not specify a build type.
- Camera-normal and post-skeleton plane-normal strategies.
- Offline NPY/polygon runner and optional uvgRTP receiver.

## Build in catkin

> Building and running on *this* machine has its own constraints — a container-only
> toolchain, and a catkin whitelist that makes a plain `catkin_make` skip this
> package silently. See **[docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md)** for a
> walkthrough of the commands that actually work here, how to try both label
> sources offline against the capture bag, and a troubleshooting table.

OpenCV normally comes with desktop ROS, but its development headers must exist:

```bash
sudo apt install ros-noetic-desktop libopencv-dev
cd ~/catkin_ws/src
cp -r /path/to/axis_grasp ./axis_grasp
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

`Release` is the default. It is also fine to specify it explicitly:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

The RTP executable is disabled by default. Enable it with:

```bash
catkin_make -DAXIS_GRASP_BUILD_SOCKET=ON
```

## Run

Calibration is deliberately not included. Pass your libCalib JSON path:

```bash
roslaunch axis_grasp axis_grasp.launch \
  calibration:=/absolute/path/to/calibration.json
```

The production stereo baseline comes from that file and is not hard-coded.

| Direction | Default topic | ROS type | Encoding/content |
| --- | --- | --- | --- |
| Input | `/disparity` | `sensor_msgs/Image` | `32FC1` or `64FC1`, pixel disparity; used when `input_kind: disparity` |
| Input | `/ikan/camera/depth_data` | `bx_msgs/DepthImage` | Encoded depth map: `16UC1` PNG millimetres, or `32FC1` metres; used when `input_kind: depth` |
| Input | `/label` | `sensor_msgs/Image` | `mono8` or `8UC1`; zero outside, nonzero inside |
| Input | `/ikan/vision/ml/detections` | `bx_msgs/DetectedInstances` | Polygon contours, used when `label_source: detections` |
| Output | `/grasp_poses_by_pipeline` | `geometry_msgs/PoseArray` | Metres and quaternion `xyzw` |

Exactly one range topic is subscribed, per `input_kind`; exactly one label source
is active, per `label_source`. See [Range input](#range-input).

### Pose frame

Every published pose is in the camera's **standard optical frame**: `x` to the
right, `y` downward, `z` forward, so the scene lies at `z > 0` and `z` *is* the
depth in metres, `z = fx * baseline / d`. It is right-handed, and the rotation
columns are the conventional gripper axes — column 0 the approach (pointing from
the gripper into the surface, i.e. away from the camera), column 1 the jaw axis,
column 2 the closing axis.

`frame_id` is the range frame's own header when there is one, and
`depth_frame_id` otherwise — `ikan/camera_link` by default. That is a robot
*link* name rather than an `..._optical_frame`, and it is informational: nothing
in the package resolves TF, so the numbers are the optical-frame coordinates of
that link, not a transform into it.

Earlier builds published the same cloud rotated 180° about `x` — identical `x`,
negated `y` and `z`, so `z < 0` with `|z|` the depth. Stored output from those
builds (CSVs, replayed bags, older validation baselines) maps onto the current
convention by negating `y` and `z` and negating rotation rows 1 and 2; a
quaternion goes by the shuffle `(qx, qy, qz, qw) -> (qw, -qz, qy, -qx)`, not by a
sign flip. `core/tests/grasp_proposal_test.cpp` pins the convention directly.

`config/default.yaml` sets `output_topic: /grasp_poses_by_pipeline`; the built-in
fallback when the YAML is not loaded is `/grasp_poses`.

Either label source pairs its ROI with a range frame when the two stamps differ
by at most `sync_slop_seconds` — except in `input_kind: depth`, which pairs by
receipt time because a `DepthImage` carries no header stamp (see
[Range input](#range-input)). A range frame waits up to
`mask_wait_timeout_seconds` (0.10 s by default) for a usable ROI; if none can
pair, it is processed over the complete disparity frame instead of being
dropped, so the node publishes whether or not a label or detections topic is
present. The node logs its interface at startup and per-stage timings for every
processed frame.

`scripts/` holds testing aids, not part of the pipeline. `synthetic_label.py`
needs no perception stack at all: it publishes a hard-coded box as the ROI, one
label per range frame, defaulting to the same `input_kind` and range topic as
`config/default.yaml` (`depth` on `/ikan/explore3d/depth_image`), so a bare
`rosrun axis_grasp synthetic_label.py _x0:=308 _y0:=308 _x1:=521 _y1:=493` is
enough on a standard build — see [Trying it without
hardware](docs/BUILD_AND_RUN.md). `detection_relay.py`
republishes a `/label` image as `DetectedInstances` so the `detections` source
can be exercised without a live perception stack; `disparity_to_depth.py` goes
the other way, republishing a `/disparity` image as a synthetic
`bx_msgs/DepthImage`, which is how the depth input mode can be exercised against
a bag that carries only disparity; `disparity_quantizer.py` republishes
`/disparity` rounded through the millimetre depth grid, isolating the
quantization the depth mode introduces from everything else about it;
`depth_to_disparity_relay.py` is the third direction — a metric depth image on
a plain `sensor_msgs/Image` converted to disparity in flight, so a depth topic
that has not migrated to `bx_msgs/DepthImage` can drive the node with
`input_kind: disparity` unchanged. That last one is a stopgap for a producer
that is expected to move to `DepthImage`, at which point `input_kind: depth`
supersedes it and the script can go. See `docs/BUILD_AND_RUN.md`.

## Range input

`input_kind` selects where the range image comes from. Both modes produce the
same `FrameInput::disparity`; nothing downstream of it changes.

### `disparity` (default)

A `32FC1` or `64FC1` pixel-disparity image on `disparity_topic`.

### `depth`

A `bx_msgs/DepthImage` on `depth_topic` — an encoded metric depth map, decoded
and reparametrized to pixel disparity using the calibration's `fx` and baseline.
The decoded image type fixes the unit, so it is never guessed:

| Decoded type | Unit | Where it comes from |
| --- | --- | --- |
| `16UC1` | millimetres | the `DepthImage` contract, a PNG, as `mine_centering` reads it |
| `32FC1` | metres | a float metric payload; the live explore3d depth publishes one (an OpenEXR map) |

```
d = fx * baseline_m / Z      with Z = raw_mm / 1000  (16UC1)
                                  Z = value          (32FC1)
```

That is the exact inverse of the reprojection the grasp stage already performs
(`Z = fx * baseline / d`, the standard optical `+Z`; see
[Pose frame](#pose-frame)), so the geometry is the one the disparity mode sees.
Both payloads are one implementation: a millimetre sample and a metre sample
differ only in the unit they arrive in, so they cannot drift apart. A zero, a
negative, a non-finite value, one outside `depth_min_m`/`depth_max_m`, and a
`raw_mm == 0` hole all become `d = 0`, which is how an invalid stereo pixel
already reads. A frame that is *entirely* holes is logged rather than silently
published, because that is also what a payload read in the wrong unit looks like.

Run it without editing the YAML:

```bash
roslaunch axis_grasp axis_grasp.launch input_kind:=depth \
  depth_topic:=/ikan/camera/depth_data label_source:=detections
```

`input_kind`, `disparity_topic` and `depth_topic` are launch arguments; each left
empty keeps its value from `config`.

**Pairing uses arrival time in depth mode.** `DepthImage` has no
`std_msgs/Header`, so there is no publisher stamp to pair on: the mask source
pairs by receipt time instead (`label_source: detections` already did). A nonzero
`unix_time_ms` still stamps the published `PoseArray`, so output timestamps stay
sensor-referenced even though pairing does not use them.

### Traps

- **Resolution.** The converted disparity is at the *depth* image's resolution.
  The mask path's shape gate rejects a label image of any other size as a hard
  mismatch and drops that frame — so with a mismatched label, every frame is
  dropped. Use `label_source: detections`, which rescales per-instance contours
  onto the range resolution by construction, or a same-resolution mask.
- **`native_width`/`native_height`** are the resolution the calibration file is
  valid *at*; the loader scales `fx`/`fy`/`cx`/`cy` by capture/native. If the
  file is valid at the depth image's own size, set these to it so the scale is
  1.0. The `focal_length` cross-check warns when the loaded `fx` disagrees with
  `DepthImage.focal_length` by more than 5%, naming `~native_width` — that
  warning is the symptom of getting this wrong.
- **`DepthImage.focal_length` documents metres, and a producer may mean it.**
  Where it holds fx in pixels the cross-check above is a useful independent read;
  the live explore3d depth publishes a physical length there instead (`0.003`, a
  sensor pitch), which is not the same quantity and would warn on every frame.
  The check therefore only runs when the value is plausibly pixels (`>= 1.0`, and
  any real focal length in pixels is at least tens); below that the node logs
  that it is skipping the comparison rather than passing it silently. A skipped
  check means a wrong `native_width`/`native_height` has no early symptom there,
  so confirm the `fx` the node logs at startup.
- **`fx` must be right even though the baseline cancels.** In `X = (x-cx)*B/d`
  and `Z = fx*B/d` the baseline cancels out of the geometry entirely, but `fx`
  and `cx` do not — and the voting stage also works in *disparity pixels*
  (`r_min`/`r_max`, `bilateral_sigma_color`), so a wrong `fx` changes the
  effective metric voting radius. `r_min`/`r_max` tuned at one resolution do not
  transfer to a depth image at another; re-validate them.
- **Expect the pose *set* to move, not the poses.** Recovering disparity from
  quantized millimetres perturbs it by up to ~0.05 px at 1 m, and component
  selection is threshold-sharp: a perturbation that small changes which
  components and arcs are selected, so pose *counts* differ from run to run even
  though positions agree to well under a millimetre. The same is true of any
  disagreement between the sensor's true `fx*baseline` and the calibration
  file's, which enters as a uniform scale on every disparity. Verified against
  this repository's own captures: feeding the depth path's exact pixels through
  the disparity path reproduces 121 of 123 comparable frames to within a micron,
  while pose counts on individual frames still differ by factors.

## Label sources

`label_source` selects where the binary ROI — the set of pixels allowed to vote
— comes from. Exactly one source is active; switching it does not change the
voting or grasp-proposal stages.

### `mask` (default)

A `mono8`/`8UC1` image on `label_topic`, paired with the disparity frame by
header stamp within `sync_slop_seconds`. Zero is outside the ROI, nonzero
inside. The class identity of a pixel is discarded; only foreground counts.

The node holds one pending disparity for `mask_wait_timeout_seconds`, measured
on a steady local clock. If no compatible mask arrives before the deadline, or
a newer mask proves that disparity cannot pair, the frame continues with
`has_labels: false`: every disparity pixel may vote and ROI cropping is skipped.
Set the timeout to `0` for fallback after the first callback-processing pass.
A synchronized all-zero mask remains an explicit empty ROI, and a synchronized
wrong-size mask remains an error; neither is treated as a missing mask.

### `detections`

Polygon contours from `bx_msgs/DetectedInstances` on `detections_topic`, which
is what the BeeX perception stack publishes. Each accepted
`DetectedInstance.contour` is split on its `(0, 0)` separators, scaled from that
instance's own `image_width`/`image_height` onto the disparity resolution, and
rasterized; the results are unioned into one ROI.

`detector_name`, `detection_classes`, and `min_confidence` filter which
detections are eligible. Note that several detectors share the topic, so leaving
`detector_name` empty will mix their contours together.

Two things worth knowing:

- **`DetectedInstances` has no `std_msgs/Header`.** There is no publisher stamp
  or frame id, so this source is synchronized on *arrival* time against the
  disparity frame rather than on header stamps.
- **Contours are in the detector's own image frame**, which is generally not the
  disparity resolution. The front camera reports `128x72`; the disparity it must
  be scaled onto varies by capture (`480x270` in one recording, `800x600` in
  another), so the factor is per-instance and can be anything. The per-instance
  scaling handles it, but a wrong factor shows up as a misplaced or empty ROI
  rather than an error.

If detections arrive but cover no pixel of the disparity frame, or none arrive
within `mask_wait_timeout_seconds`, the node logs a throttled warning and
processes that disparity over the complete frame.

```bash
roslaunch axis_grasp axis_grasp.launch \
  label_source:=detections \
  detector_name:=sim_front_cam_seg
```

`label_source`, `detections_topic`, `detector_name`, and `min_confidence` can
all be overridden on the command line. Set `detection_classes` in the YAML,
since it is a list. Leaving any of them empty keeps the value from `config`.

### Building without bx_msgs

`bx_msgs` is a build option, on by default:

```bash
catkin_make -DAXIS_GRASP_WITH_DETECTIONS=OFF
```

A build without it still supports `label_source: mask` and
`input_kind: disparity`, and rejects `label_source: detections` and
`input_kind: depth` at startup with a fatal message rather than failing to link.
All three of these uses of `bx_msgs` — detections contours, the `DepthImage`
payload, and the depth mode's message type — are behind that one option.

## Configuration

Edit `config/default.yaml`, or select another file:

```bash
roslaunch axis_grasp axis_grasp.launch \
  config:=/absolute/path/to/my_axis_grasp.yaml \
  calibration:=/absolute/path/to/calibration.json
```

Parameters are loaded at startup; restart the node after changing the YAML.

Input keys: `input_kind`, `disparity_topic`, `depth_topic`, `depth_frame_id`,
`depth_min_m`, `depth_max_m` (see [Range input](#range-input)); `label_source`,
`label_topic`, `detections_topic`, `detector_name`, `min_confidence`,
`detection_classes` (see [Label sources](#label-sources)).

### ROI crop

`enable_roi_crop: true` processes the label bounding box plus a safe halo. The
automatic halo covers bilateral/Sobel support, the maximum voting radius,
morphology, edge rejection, and the post-skeleton band. Reprojection remains
in the original camera geometry by shifting `cx` and `cy`, and reported pixel
coordinates are shifted back.

Keep `roi_crop_padding: -1` unless you validate a smaller manual value. Set
`enable_roi_crop: false` to process the full image. See `VALIDATION.md`.

### Rope filter

`remove_straight_rope: true` (the default) drops straight-line components — rope
and cable — from the coherent vote accumulator, so they never reach component
filtering or grasp proposal. It is also the one rope option exposed at launch,
which makes it the rope on/off switch:

```bash
roslaunch axis_grasp axis_grasp.launch remove_straight_rope:=false
```

Leaving the argument out keeps the YAML value, and a value that is not a bool
fails the launch rather than being read as `true`.

A component is treated as rope when it clears `min_pixels`, is elongated, has a
skeleton of at least 80 pixels, and at least 70% of that skeleton sits within
2 px of a single RANSAC line. Those thresholds are the `RopeFilterConfig`
defaults in `core/include/axis_grasp/core/config.h`; they are not YAML keys,
because they came from an offline validation rather than from launch-time tuning.

## OpenCV scope

OpenCV handles operations with direct optimized equivalents: bilateral filter,
Sobel, dilation, erosion, closing, and connected-component labeling. The
thinning LUT stays custom because its output must match the chosen scikit-image
Zhang behavior. Percentiles, directional voting, centerline ordering, and grasp
geometry have no direct OpenCV equivalent with the required semantics.

## Diagnostics

```bash
rosnode info /axis_grasp_node
rostopic hz /disparity
rostopic hz /label
rostopic type /disparity
rostopic type /label
rosparam get /axis_grasp_node
```

The node's startup line names the range topic it actually subscribed to, so it
is the quickest check that a launch argument took effect:

```
axis_grasp ready: range=/disparity (input_kind=disparity) label_source=mask ...
```

In depth mode, watch the depth topic instead, and note that the node subscribes
to it directly — there is no intermediate disparity topic to inspect:

```bash
rostopic hz /ikan/camera/depth_data
rostopic type /ikan/camera/depth_data
```

When using `label_source: detections`, watch the detections instead of `/label`:

```bash
rostopic hz /ikan/vision/ml/detections
rostopic echo -n1 /ikan/vision/ml/detections/detector_name
```

