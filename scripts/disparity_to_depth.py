#!/usr/bin/env python3
"""Republish /disparity as a synthetic bx_msgs/DepthImage.

A testing aid, not part of the pipeline. It exists to separate two failure modes
that look identical from the outside: an error in the depth *conversion*, and a
disagreement between the two encodings in a bag. Feeding the node a DepthImage
built from the very disparity image it already reproduces correctly means the
depth path's input is known exactly, so any output difference is the adapter's
fault rather than the data's.

  rosrun axis_grasp disparity_to_depth.py _fx:=479.266 _baseline_m:=0.0960152

Parameters:
  ~input_topic   (str,   /disparity)  source disparity image
  ~output_topic  (str,   /depth_synthetic)
  ~fx            (float, required)    focal length in pixels
  ~baseline_m    (float, required)    stereo baseline in metres
  ~sensor_type   (int,   115)         115 is the VOYIS stereo camera
  ~depth_min_m   (float, 0.05)        band floor
  ~depth_max_m   (float, 100.0)       band ceiling
"""

import cv2
import numpy as np
import rospy
from bx_msgs.msg import DepthImage
from sensor_msgs.msg import Image


class ToDepth:
    def __init__(self):
        self.fx = rospy.get_param("~fx")
        self.baseline = rospy.get_param("~baseline_m")
        self.sensor_type = rospy.get_param("~sensor_type", 115)
        self.min_m = rospy.get_param("~depth_min_m", 0.05)
        self.max_m = rospy.get_param("~depth_max_m", 100.0)
        if self.fx <= 0.0 or self.baseline <= 0.0:
            raise rospy.ROSInitException(
                "~fx and ~baseline_m must both be positive")
        in_topic = rospy.get_param("~input_topic", "/disparity")
        out_topic = rospy.get_param("~output_topic", "/depth_synthetic")
        self.publisher = rospy.Publisher(out_topic, DepthImage, queue_size=2)
        self.subscriber = rospy.Subscriber(
            in_topic, Image, self.on_image, queue_size=2)
        self.count = 0

    def on_image(self, message):
        if message.encoding != "32FC1":
            rospy.logwarn_throttle(
                5.0, "expected 32FC1, got %s", message.encoding)
            return
        disparity = np.frombuffer(message.data, np.float32).reshape(
            message.height, message.width).astype(np.float64)

        # The inverse of the node's own reprojection, so a round trip through
        # this relay and back returns the input image.
        mm = np.zeros(disparity.shape, dtype=np.uint16)
        live = np.abs(disparity) > 0.0
        depth_m = np.zeros(disparity.shape, dtype=np.float64)
        depth_m[live] = self.fx * self.baseline / np.abs(disparity[live])
        in_band = live & (depth_m >= self.min_m) & (depth_m <= self.max_m)
        mm[in_band] = np.rint(depth_m[in_band] * 1000.0).astype(np.uint16)

        ok, buffer = cv2.imencode(".png", mm)
        if not ok:
            rospy.logwarn_throttle(5.0, "PNG encode failed")
            return

        out = DepthImage()
        out.unix_time_ms = message.header.stamp.to_nsec() // 1000000
        out.sensor_type = self.sensor_type
        out.focal_length = self.fx
        out.depth_data = buffer.tobytes()
        self.publisher.publish(out)
        self.count += 1
        if self.count % 25 == 0:
            rospy.loginfo("republished %d frames", self.count)


if __name__ == "__main__":
    rospy.init_node("disparity_to_depth")
    ToDepth()
    rospy.spin()
