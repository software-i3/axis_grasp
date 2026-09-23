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
| Input | `/disparity` | `sensor_msgs/Image` | `32FC1` or `64FC1`, pixel disparity |
| Input | `/label` | `sensor_msgs/Image` | `mono8` or `8UC1`; zero outside, nonzero inside |
| Input | `/ikan/vision/ml/detections` | `bx_msgs/DetectedInstances` | Polygon contours, used when `label_source: detections` |
| Output | `/grasp_poses_by_pipeline` | `geometry_msgs/PoseArray` | Metres and quaternion `xyzw` |

`config/default.yaml` sets `output_topic: /grasp_poses_by_pipeline`; the built-in
fallback when the YAML is not loaded is `/grasp_poses`.

Either label source pairs its ROI with a disparity when the two stamps differ by
at most `sync_slop_seconds`. A disparity waits up to
`mask_wait_timeout_seconds` (0.10 s by default) for a usable ROI; if none can
pair, it is processed over the complete disparity frame instead of being
dropped, so the node publishes whether or not a label or detections topic is
present. The node logs its interface at startup and per-stage timings for every
processed frame.

`scripts/detection_relay.py` is a testing aid, not part of the pipeline: it
republishes a `/label` image as `DetectedInstances` so the `detections` source can
be exercised without a live perception stack. See `docs/BUILD_AND_RUN.md`.

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

A build without it still supports `label_source: mask`, and rejects
`label_source: detections` at startup with a fatal message rather than failing
to link.

## Configuration

Edit `config/default.yaml`, or select another file:

```bash
roslaunch axis_grasp axis_grasp.launch \
  config:=/absolute/path/to/my_axis_grasp.yaml \
  calibration:=/absolute/path/to/calibration.json
```

Parameters are loaded at startup; restart the node after changing the YAML.

### ROI crop

`enable_roi_crop: true` processes the label bounding box plus a safe halo. The
automatic halo covers bilateral/Sobel support, the maximum voting radius,
morphology, edge rejection, and the post-skeleton band. Reprojection remains
in the original camera geometry by shifting `cx` and `cy`, and reported pixel
coordinates are shifted back.

Keep `roi_crop_padding: -1` unless you validate a smaller manual value. Set
`enable_roi_crop: false` to process the full image. See `VALIDATION.md`.

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

When using `label_source: detections`, watch the detections instead of `/label`:

```bash
rostopic hz /ikan/vision/ml/detections
rostopic echo -n1 /ikan/vision/ml/detections/detector_name
```

