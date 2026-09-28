#!/usr/bin/env python3
"""Republish a metric depth image as a pixel-disparity image.

A stopgap, not part of the pipeline. `input_kind: depth` reads a
`bx_msgs/DepthImage`; a package that has not migrated to that message yet
publishes depth as a plain `sensor_msgs/Image` instead. This relay bridges the
two so the node can run against it unchanged, with `input_kind: disparity`.

  rosrun axis_grasp depth_to_disparity_relay.py \
    _input_topic:=/explore3d/depth _fx:=479.266 _baseline_m:=0.0960152

It is the inverse of `disparity_to_depth.py` and uses the same formula as the
node's depth mode, so the node's reprojection recovers the source depth exactly:

    d = fx * baseline_m / Z

`fx` must be the focal length the *node* will reproject with at this image's
resolution -- not the raw value in the calibration file. The node scales
fx/fy/cx/cy by capture/native, so with the default `native_width: 1600` a
800-pixel-wide depth image is reprojected at fx = file_fx * 800/1600. Get this
wrong and every range is scaled by the ratio; nothing downstream complains.
Read the number the node actually uses from its own startup log if unsure.

`baseline_m` cancels out of the geometry (`Z = fx*B/d` and `X = (x-cx)*Z/fx`),
so its exact value does not affect the recovered metric geometry -- but it does
set the disparity scale, and the voting stage works in disparity pixels.
Keep it equal to the node's calibration baseline so the disparity magnitude
matches the range your `r_min`/`r_max` were tuned at.

Parameters:
  ~input_topic   (str,   /explore3d/depth)  source depth image
  ~output_topic  (str,   /disparity)        disparity image to publish
  ~fx            (float, required)          focal length in pixels, at this
                                            image's resolution
  ~baseline_m    (float, required)          stereo baseline in metres
  ~depth_min_m   (float, 0.05)              band floor; outside becomes a hole
  ~depth_max_m   (float, 100.0)             band ceiling
"""

import numpy as np
import rospy
from sensor_msgs.msg import Image

# Depth conventions that appear on sensor_msgs/Image, and the metres-per-unit
# factor each one implies. A wrong guess is a 1000x scale error, so the encoding
# string decides rather than a heuristic on the values.
ENCODINGS = {
    "32FC1": (np.float32, 1.0),      # metres, the ROS depth-image convention
    "64FC1": (np.float64, 1.0),
    "16UC1": (np.uint16, 0.001),     # millimetres, as the *_raw topics publish
    "mono16": (np.uint16, 0.001),
}


class ToDisparity:
    def __init__(self):
        self.fx = rospy.get_param("~fx")
        self.baseline = rospy.get_param("~baseline_m")
        self.min_m = rospy.get_param("~depth_min_m", 0.05)
        self.max_m = rospy.get_param("~depth_max_m", 100.0)
        if self.fx <= 0.0 or self.baseline <= 0.0:
            raise rospy.ROSInitException(
                "~fx and ~baseline_m must both be positive")
        in_topic = rospy.get_param("~input_topic", "/explore3d/depth")
        out_topic = rospy.get_param("~output_topic", "/disparity")
        self.publisher = rospy.Publisher(out_topic, Image, queue_size=2)
        self.subscriber = rospy.Subscriber(
            in_topic, Image, self.on_image, queue_size=2)
        self.count = 0
        self.holes = 0
        rospy.loginfo("depth_to_disparity_relay: %s -> %s (fx=%.6g, B=%.6g m, "
                      "band [%.3g, %.3g] m)",
                      in_topic, out_topic, self.fx, self.baseline,
                      self.min_m, self.max_m)

    def on_image(self, message):
        spec = ENCODINGS.get(message.encoding)
        if spec is None:
            rospy.logwarn_throttle(
                5.0, "expected one of %s, got %s",
                "/".join(sorted(ENCODINGS)), message.encoding)
            return
        dtype, to_m = spec
        itemsize = np.dtype(dtype).itemsize
        if message.step < message.width * itemsize or \
                len(message.data) < message.step * message.height:
            rospy.logwarn_throttle(
                5.0, "depth image step/data is truncated (%d bytes, step %d, "
                     "%dx%d %s)", len(message.data), message.step,
                message.width, message.height, message.encoding)
            return

        # Whole rows, then slice off any padding: `step` may exceed the row.
        order = ">" if message.is_bigendian else "<"
        stride = message.step // itemsize
        rows = np.frombuffer(
            message.data,
            dtype=np.dtype(dtype).newbyteorder(order),
            count=stride * message.height).reshape(message.height, stride)
        depth_m = rows[:, :message.width].astype(np.float64) * to_m

        # Z <= 0, a NaN/inf sample, or one outside the band becomes d = 0, which
        # is how an invalid stereo pixel already reads. Never emit a negative or
        # non-finite disparity: the node's bilateral filter and Sobel run before
        # anything scrubs those. Clearing the non-finite samples first keeps the
        # band test below off NaN, which would otherwise warn on every frame.
        depth_m[~np.isfinite(depth_m)] = 0.0
        live = (depth_m >= self.min_m) & (depth_m <= self.max_m)
        disparity = np.zeros(depth_m.shape, dtype=np.float32)
        np.divide(self.fx * self.baseline, depth_m,
                  out=disparity, where=live)

        holes = int(depth_m.size - live.sum())
        if holes == depth_m.size:
            rospy.logwarn_throttle(
                5.0, "every sample is zero, non-finite, or outside [%g, %g] m; "
                     "check the units -- a millimetre image read as metres is "
                     "1000x out of band", self.min_m, self.max_m)

        out = Image()
        # Carried through unchanged, so the node keeps pairing by header stamp
        # the way it does for a real disparity image, and the published poses
        # stay sensor-referenced.
        out.header = message.header
        out.height = message.height
        out.width = message.width
        out.encoding = "32FC1"
        out.is_bigendian = 0
        out.step = message.width * 4
        out.data = disparity.tobytes()
        self.publisher.publish(out)

        self.count += 1
        self.holes += holes
        if self.count % 25 == 0:
            rospy.loginfo("republished %d frames, %d holes of %d samples",
                          self.count, self.holes, self.count * depth_m.size)


if __name__ == "__main__":
    rospy.init_node("depth_to_disparity_relay")
    ToDisparity()
    rospy.spin()
