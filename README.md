# axis_grasp

Standalone ROS1/catkin package for disparity-based axis voting and grasp-pose
proposal. It uses OpenCV for optimized image processing while retaining the
custom scikit-image-compatible Zhang thinning implementation.

## Features

- OpenCV bilateral filtering, Sobel, morphology, and connected components.
- Custom lookup-table Zhang thinning; OpenCV thinning is intentionally unused.
- Label-bounding-box processing with a conservative automatic halo.
- Runtime ROS YAML configuration and `/label` as the default label topic.
- Release mode whenever catkin/CMake does not specify a build type.
- Camera-normal and post-skeleton plane-normal strategies.
- Offline NPY/polygon runner and optional uvgRTP receiver.

## Build in catkin

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
| Output | `/grasp_poses` | `geometry_msgs/PoseArray` | Metres and quaternion `xyzw` |

Inputs must have equal dimensions and timestamps within `sync_slop_seconds`.
The node logs its interface at startup and per-stage timings for every pair.

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

