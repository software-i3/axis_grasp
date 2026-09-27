#!/usr/bin/env python3
"""Republish /disparity rounded through the millimetre depth grid.

A testing aid, not part of the pipeline. It exists to isolate one variable: the
depth input mode feeds the voting stage a disparity that differs from a
published disparity image only by uint16-millimetre quantization. Running the
node in `input_kind: disparity` against this relay's output therefore reproduces
the depth mode's *input perturbation* while leaving the transport, stamps, and
pairing of the disparity path untouched.

If the poses from this run diverge from an unquantized run the same way the
depth run's do, the divergence is the pipeline's sensitivity to a sub-pixel
input change, not a defect of the depth conversion.

  rosrun axis_grasp disparity_quantizer.py

Parameters:
  ~input_topic   (str,  /disparity)  source disparity image
  ~output_topic  (str,  /disparity_quantized)
  ~fx            (float, required)   focal length in pixels
  ~baseline_m    (float, required)   stereo baseline in metres
  ~depth_min_m   (float, 0.05)       band floor; outside is a hole
  ~depth_max_m   (float, 100.0)      band ceiling
  ~grid_mm       (float, 1.0)        quantization step; 1.0 matches the depth
                                     input mode, larger values over-perturb the
                                     image on purpose to probe sensitivity
"""

import numpy as np
import rospy
from sensor_msgs.msg import Image


class Quantizer:
    def __init__(self):
        self.fx = rospy.get_param("~fx")
        self.baseline = rospy.get_param("~baseline_m")
        self.min_m = rospy.get_param("~depth_min_m", 0.05)
        self.max_m = rospy.get_param("~depth_max_m", 100.0)
        # The depth input mode's step is exactly 1 mm (uint16 millimetres).
        self.grid_mm = rospy.get_param("~grid_mm", 1.0)
        if self.fx <= 0.0 or self.baseline <= 0.0:
            raise rospy.ROSInitException(
                "~fx and ~baseline_m must both be positive")
        in_topic = rospy.get_param("~input_topic", "/disparity")
        out_topic = rospy.get_param("~output_topic", "/disparity_quantized")
        self.publisher = rospy.Publisher(out_topic, Image, queue_size=2)
        self.subscriber = rospy.Subscriber(
            in_topic, Image, self.on_image, queue_size=2)
        self.count = 0

    def on_image(self, message):
        if message.encoding not in ("32FC1", "64FC1"):
            rospy.logwarn_throttle(
                5.0, "quantizer expects 32FC1 or 64FC1, got %s", message.encoding)
            return
        dtype = np.float32 if message.encoding == "32FC1" else np.float64
        source = np.frombuffer(message.data, dtype=dtype).reshape(
            message.height, message.width).astype(np.float64)

        # Disparity -> depth -> whole millimetres -> back to disparity. This is
        # the same round trip the depth input mode performs, so the result is
        # quantized identically.
        out = np.zeros(source.shape, dtype=np.float32)
        live = np.abs(source) > 0.0
        depth_m = np.zeros(source.shape, dtype=np.float64)
        depth_m[live] = self.fx * self.baseline / np.abs(source[live])
        in_band = live & (depth_m >= self.min_m) & (depth_m <= self.max_m)
        mm = np.rint(depth_m[in_band] * 1000.0 / self.grid_mm) * self.grid_mm
        mm[mm == 0.0] = self.grid_mm  # 0 mm is a hole even after rounding
        out[in_band] = (self.fx * self.baseline / (mm / 1000.0)).astype(np.float32)

        message.data = out.tobytes()
        self.publisher.publish(message)
        self.count += 1
        if self.count % 25 == 0:
            rospy.loginfo("quantized %d frames", self.count)


if __name__ == "__main__":
    rospy.init_node("disparity_quantizer")
    Quantizer()
    rospy.spin()
