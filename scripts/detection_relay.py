#!/usr/bin/env python3
"""Testing aid: republish a label image as bx_msgs/DetectedInstances.

Not part of the pipeline. It exists so `label_source: detections` can be tried
without a live perception stack: replay a bag holding `/disparity` and `/label`,
and this node turns each `/label` frame into the `DetectedInstances` message the
real detector would have produced.

It republishes on `/disparity` arrival so the two land within a few ms of each
other, which is what arrival-time synchronization pairs up. Contours are also
scaled down into the detector's own frame on the way out, so the node's
per-instance `image_width`/`image_height` scaling is genuinely exercised.

    rosrun axis_grasp detection_relay.py _detector_name:=sim_front_cam_seg

Parameters (all private):

  ~detections_topic  where to publish       (default /ikan/vision/ml/detections)
  ~label_topic       label image to read    (default /label)
  ~trigger_topic     frame to publish on    (default /disparity)
  ~detector_name     name to report         (default sim_front_cam_seg)
  ~class_name        class for every object (default limpet)
  ~confidence        confidence, 1..100     (default 90)
  ~detector_width    detector frame width   (default 128)
  ~detector_height   detector frame height  (default 72)
  ~combine_contours  true to emit all polygons in one instance, split by (0, 0)
                                            (default false)
"""
import cv2
import numpy as np
import rospy
from bx_msgs.msg import DetectedInstance, DetectedInstances
from sensor_msgs.msg import Image


class DetectionRelay:
    def __init__(self):
        self.detector_width = rospy.get_param("~detector_width", 128)
        self.detector_height = rospy.get_param("~detector_height", 72)
        self.detector_name = rospy.get_param("~detector_name", "sim_front_cam_seg")
        self.class_name = rospy.get_param("~class_name", "limpet")
        self.confidence = rospy.get_param("~confidence", 90)
        self.combine = rospy.get_param("~combine_contours", False)
        self.mask = None
        self.published = 0

        self.publisher = rospy.Publisher(
            rospy.get_param("~detections_topic", "/ikan/vision/ml/detections"),
            DetectedInstances,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~label_topic", "/label"),
            Image,
            self.on_label,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~trigger_topic", "/disparity"),
            Image,
            self.on_trigger,
            queue_size=1,
        )
        rospy.loginfo(
            "detection relay ready: publishing to %s as '%s' (%dx%d frame)",
            self.publisher.name,
            self.detector_name,
            self.detector_width,
            self.detector_height,
        )

    def on_label(self, message):
        self.mask = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width
        )

    def scaled_contours(self, frame):
        """Return the label's external contours in the detector's frame."""
        scale_x = float(self.detector_width) / frame.width
        scale_y = float(self.detector_height) / frame.height
        contours, _ = cv2.findContours(
            (self.mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            if cv2.contourArea(contour) < 4:
                continue
            points = contour.reshape(-1, 2).astype(np.float64)
            points[:, 0] = np.clip(
                np.round(points[:, 0] * scale_x), 0, self.detector_width - 1
            )
            points[:, 1] = np.clip(
                np.round(points[:, 1] * scale_y), 0, self.detector_height - 1
            )
            yield [int(v) for v in points.astype(np.uint16).ravel()]

    def make_instance(self, flat_contour):
        instance = DetectedInstance()
        instance.name = self.class_name
        instance.confidence = self.confidence
        instance.sensor_source = 1  # SENSOR_MAIN_MONOCULAR
        instance.image_width = self.detector_width
        instance.image_height = self.detector_height
        instance.contour = flat_contour
        return instance

    def on_trigger(self, frame):
        if self.mask is None:
            rospy.logwarn_throttle(5.0, "no label received yet; skipping frame")
            return

        message = DetectedInstances()
        message.image_id = self.published
        message.detector_name = self.detector_name

        contours = list(self.scaled_contours(frame))
        if not contours:
            rospy.logwarn_throttle(5.0, "no usable contours in this label frame")
            return

        if self.combine:
            # One instance carrying several polygons, split by (0, 0).
            flat = []
            for contour in contours:
                if flat:
                    flat += [0, 0]
                flat += contour
            message.detections.append(self.make_instance(flat))
        else:
            for contour in contours:
                message.detections.append(self.make_instance(contour))

        self.publisher.publish(message)
        self.published += 1
        rospy.loginfo(
            "published %d instances for a %ux%u frame",
            len(message.detections),
            frame.width,
            frame.height,
        )


if __name__ == "__main__":
    rospy.init_node("detection_relay", anonymous=True)
    DetectionRelay()
    rospy.spin()
