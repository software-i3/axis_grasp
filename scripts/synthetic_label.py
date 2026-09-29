#!/usr/bin/env python3
"""Testing aid: publish a filled rectangle as the node's ROI.

Not part of the pipeline. It is the inverse of `detection_relay.py`: instead of
turning a label image into detections, it turns a hard-coded rectangle into the
ROI the node expects, so a known region can be voted on without a perception
stack.

    rosrun axis_grasp synthetic_label.py \
      _x0:=308 _y0:=308 _x1:=521 _y1:=493

The defaults mirror `config/default.yaml`, so on a standard build a bare run is
the explore3d depth image and needs nothing but the box. Anywhere else -- an
offline disparity bag, a differently named depth topic -- name the range input:

    rosrun axis_grasp synthetic_label.py _input_kind:=disparity \
      _range_topic:=/disparity _x0:=308 _y0:=308 _x1:=521 _y1:=493

Why it republishes per range frame rather than latching once: the node pairs one
ROI with one range frame and then consumes it. In `input_kind: disparity` the
pairing is on header stamps, so a label latched at a fixed stamp falls more than
`sync_slop_seconds` behind every later disparity and is discarded as stale; in
`input_kind: depth` it is a receipt time, and one label cannot be within the slop
of two different frames. Publishing on each range frame is what keeps the ROI
attached, so run this alongside the node, not before it.

That also means this script does nothing at all until a range frame arrives: with
nothing published on ~range_topic, ~label_topic stays open and empty, which
`rostopic echo` reports as "no new messages" rather than as an error. The five
second watchdog below says so out loud instead of leaving that silent.

Pixels only. This script never produces or consumes a 3D point, so the package's
pose frame -- standard optical, +Z forward -- does not reach it and a
pose-convention change changes nothing here.

The `label` output is drawn at the *range* frame's resolution, because the mask
source rejects a label of any other size as a hard shape mismatch and drops that
frame -- with `~source_width`/`~source_height` left at 0 the coordinates are
range-frame pixels. If they are not -- because the box was read off a colour
image, say -- set those two to the frame the coordinates came from and they are
scaled exactly as the detections path scales contours. The `detections` output
does not need that: the node rescales the contour itself from the instance's own
`image_width`/`image_height`.

Parameters (all private):

  ~output_kind     `label` publishes mono8 `sensor_msgs/Image` with the rectangle
                   filled; `detections` publishes `bx_msgs/DetectedInstances`
                   with the rectangle as a four-vertex contour. Anything but
                   `label` is a fatal error rather than a silent default
                                                          (default label)
  ~label_topic     where to publish                     (default /label)
  ~range_topic     the frame to republish on  (default /ikan/explore3d/depth_image)
  ~input_kind      disparity | depth, the range type     (default depth)
  ~x0 ~y0 ~x1 ~y1  the rectangle's diagonal, in ~source_width x ~source_height
                   pixels                                 (default 308 308 521 493)
  ~source_width    the frame ~x*/~y* are in; 0 means the
  ~source_height   range frame's own size               (default 0, 0)
  ~fill_value      label value inside the rectangle     (default 255)
  ~detector_name   detections output only               (default synthetic_label)
  ~class_name      detections output only               (default synthetic_roi)

`input_kind: depth` decodes the DepthImage payload to learn its resolution, since
that message carries no width or height field.
"""
import numpy as np
import rospy
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:  # Only the depth input kind needs it.
    cv2 = None


class SyntheticLabel:
    def __init__(self):
        self.output_kind = rospy.get_param("~output_kind", "label")
        if self.output_kind not in ("label", "detections"):
            rospy.logfatal(
                "~output_kind must be 'label' or 'detections', got '%s'",
                self.output_kind,
            )
            raise SystemExit(1)

        self.x0 = rospy.get_param("~x0", 308)
        self.y0 = rospy.get_param("~y0", 308)
        self.x1 = rospy.get_param("~x1", 521)
        self.y1 = rospy.get_param("~y1", 493)
        self.source_width = rospy.get_param("~source_width", 0)
        self.source_height = rospy.get_param("~source_height", 0)
        self.fill_value = rospy.get_param("~fill_value", 255)
        self.detector_name = rospy.get_param("~detector_name", "synthetic_label")
        self.class_name = rospy.get_param("~class_name", "synthetic_roi")

        self.published = 0
        self.frames_seen = 0
        # Both of these mirror config/default.yaml, so the script's defaults are
        # the topic the node is actually fed by on a standard build.
        self.depth_kind = rospy.get_param("~input_kind", "depth") == "depth"

        label_topic = rospy.get_param("~label_topic", "/label")
        self.range_topic = rospy.get_param(
            "~range_topic", "/ikan/explore3d/depth_image"
        )
        range_topic = self.range_topic
        if self.output_kind == "detections":
            # Imported here rather than at module scope: a build with
            # -DAXIS_GRASP_WITH_DETECTIONS=OFF has no bx_msgs, and this script
            # should still be usable there in its label form.
            from bx_msgs.msg import DetectedInstances

            self.publisher = rospy.Publisher(label_topic, DetectedInstances, queue_size=1)
        else:
            self.publisher = rospy.Publisher(label_topic, Image, queue_size=1)

        if self.depth_kind:
            if cv2 is None:
                rospy.logfatal("input_kind is 'depth' but cv2 is not importable")
                raise SystemExit(1)
            try:
                from bx_msgs.msg import DepthImage
            except ImportError:
                # DepthImage is the only message this script needs from that
                # package. A build with -DAXIS_GRASP_WITH_DETECTIONS=OFF has no
                # bx_msgs at all, and the failure would otherwise surface later as
                # a name error inside on_depth, so name the flag here.
                rospy.logfatal(
                    "input_kind is 'depth' but bx_msgs is not importable: this "
                    "build has no detections support, so rebuild with "
                    "-DAXIS_GRASP_WITH_DETECTIONS=ON, or point the script at a "
                    "disparity topic with _input_kind:=disparity"
                )
                raise SystemExit(1)

            rospy.Subscriber(range_topic, DepthImage, self.on_depth, queue_size=1)
        else:
            rospy.Subscriber(range_topic, Image, self.on_image, queue_size=1)

        rospy.loginfo(
            "synthetic label ready: %s output, box (%s, %s)-(%s, %s) from a %s "
            "frame, publishing to %s",
            self.output_kind,
            self.x0,
            self.y0,
            self.x1,
            self.y1,
            "%dx%d" % (self.source_width, self.source_height)
            if self.source_width and self.source_height
            else "range-sized",
            self.publisher.name,
        )

        # This script publishes only in response to a range frame, so a
        # ~range_topic that is not the topic the node is fed by leaves it
        # advertising ~label_topic and publishing nothing -- which the outside
        # world sees as an empty topic, `rostopic echo` reporting "no new
        # messages" rather than an error. Name the cause instead of the symptom.
        self.started = rospy.Time.now()
        self.watch_timer = rospy.Timer(rospy.Duration(5.0), self.report_silence)

    def report_silence(self, _event):
        if self.frames_seen:
            return
        rospy.logwarn(
            "no %s frame on %s in %.0fs, so %s is staying empty: this script "
            "publishes one label per range frame and nothing has triggered it. "
            "Set ~range_topic (and ~input_kind) to the range topic the node "
            "subscribes to. %s",
            "depth" if self.depth_kind else "disparity",
            self.range_topic,
            (rospy.Time.now() - self.started).to_sec(),
            self.publisher.name,
            self.topic_hint(),
        )

    def topic_hint(self):
        """What the master knows about ~range_topic, when it can be reached.

        Registration and publication are separate facts: a topic can be listed
        because something is subscribed to it, or because a node died without
        unregistering, and still have no publisher. That is the case this hint
        exists to tell apart, so both are reported.
        """
        try:
            # Imported here so the script's other paths do not depend on it.
            import rosgraph

            master = rosgraph.Master("/synthetic_label")
            types = dict(master.getTopicTypes())
            publishers = dict(master.getSystemState()[0])
        except Exception as error:  # A master that is unreachable is its own fault.
            return "The master's topic list could not be read (%s)." % error
        if self.range_topic in types:
            nodes = publishers.get(self.range_topic) or []
            return "%s is registered as %s, published by %s." % (
                self.range_topic,
                types[self.range_topic],
                ", ".join(nodes) if nodes else "nothing",
            )
        near = sorted(
            name for name in types if "depth" in name or "disparity" in name
        )
        return "No topic named %s is registered; the master has: %s." % (
            self.range_topic,
            ", ".join(near) if near else "nothing matching 'depth' or 'disparity'",
        )

    def box_at(self, width, height):
        """The rectangle in range-frame pixels, clipped to the frame.

        Scaling is a straight multiplication of each coordinate by
        destination/source, matching ScalePolygon, so a box given in another
        frame lands where the detections path would put it.
        """
        source_width = self.source_width or width
        source_height = self.source_height or height
        sx = float(width) / source_width
        sy = float(height) / source_height
        # A diagonal is unordered: sort so either corner pair works.
        left, right = sorted((self.x0, self.x1))
        top, bottom = sorted((self.y0, self.y1))
        return (
            max(0, int(np.floor(left * sx))),
            max(0, int(np.floor(top * sy))),
            min(width - 1, int(np.ceil(right * sx))),
            min(height - 1, int(np.ceil(bottom * sy))),
        )

    def on_image(self, message):
        if not message.width or not message.height:
            rospy.logwarn_throttle(5.0, "range frame has no size; skipping")
            return
        self.emit(message.header, message.width, message.height)

    def on_depth(self, message):
        # DepthImage has no Header and no width/height: the payload is an encoded
        # image, so its resolution exists only once it is decoded.
        if not message.depth_data:
            rospy.logwarn_throttle(5.0, "depth_data is empty; skipping frame")
            return
        encoded = np.frombuffer(bytearray(message.depth_data), dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            rospy.logwarn_throttle(
                5.0, "depth_data did not decode as an image; skipping frame"
            )
            return
        # The node pairs this input by receipt time, so only the size is used.
        self.emit(None, decoded.shape[1], decoded.shape[0])

    def emit(self, header, width, height):
        # Counted before the box is judged: a box that covers no pixel is still
        # a frame arriving, and report_silence is about the topic, not the box.
        self.frames_seen += 1
        x0, y0, x1, y1 = self.box_at(width, height)
        if x0 > x1 or y0 > y1:
            rospy.logwarn_throttle(
                5.0,
                "the box covers no pixel of the %dx%d range frame; check "
                "~x0/~y0/~x1/~y1 and ~source_width/~source_height",
                width,
                height,
            )
            return

        if self.output_kind == "detections":
            self.publish_detections(x0, y0, x1, y1, width, height)
        else:
            self.publish_label(header, x0, y0, x1, y1, width, height)

        self.published += 1
        if self.published == 1:
            rospy.loginfo(
                "box resolved to (%d, %d)-(%d, %d) in a %dx%d frame",
                x0,
                y0,
                x1,
                y1,
                width,
                height,
            )

    def publish_label(self, header, x0, y0, x1, y1, width, height):
        # Zero outside, nonzero inside. The class identity of a pixel is
        # discarded by the node; only foreground counts.
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[y0 : y1 + 1, x0 : x1 + 1] = self.fill_value
        message = Image()
        if header is not None:
            # Disparity mode pairs on header stamps, so the label carries the
            # range frame's own stamp; copying it makes the pair exact.
            message.header = header
        else:
            message.header.stamp = rospy.Time.now()
        message.height = height
        message.width = width
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = width
        message.data = mask.tobytes()
        self.publisher.publish(message)

    def publish_detections(self, x0, y0, x1, y1, width, height):
        from bx_msgs.msg import DetectedInstance, DetectedInstances

        instance = DetectedInstance()
        instance.name = self.class_name
        instance.confidence = 100
        instance.sensor_source = 1  # SENSOR_MAIN_MONOCULAR
        # image_width/image_height describe the contour's own frame -- the frame
        # the coordinates were given in -- because the node rescales from them.
        instance.image_width = self.source_width or width
        instance.image_height = self.source_height or height
        # A rectangle as four vertices. (0, 0) is the node's polygon separator,
        # so a box touching the top-left corner loses that vertex; the other
        # three still rasterize the same rectangle.
        instance.contour = [
            int(v)
            for v in (
                self.x0,
                self.y0,
                self.x1,
                self.y0,
                self.x1,
                self.y1,
                self.x0,
                self.y1,
            )
        ]

        message = DetectedInstances()
        message.image_id = self.published
        message.detector_name = self.detector_name
        message.detections.append(instance)
        self.publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("synthetic_label", anonymous=True)
    SyntheticLabel()
    rospy.spin()
