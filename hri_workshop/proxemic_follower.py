#!/usr/bin/env python3
"""
proxemic_follower.py  –  HRI Workshop: Proxemics Student Template
=================================================================
University of Lincoln  |  CMP3103M / HRI Workshop

Overview
--------
This node subscribes to the Limo robot's onboard RGB and depth cameras,
detects a walking human using OpenCV's HOG detector, estimates the distance
to that person from the depth image, and drives the robot so that it
maintains a target *proxemic distance* as defined by Edward Hall (1966).

Your job
--------
Work through the five tasks below (marked with TODO blocks).  Each task
builds on the previous one.  Read every comment carefully before writing
any code.

Proxemic Zones (Hall, 1966)
---------------------------
  Zone        | Distance Range  | Typical relationship
  ------------|-----------------|------------------------------
  Intimate    | 0  – 0.45 m     | Physical contact / close intimacy
  Personal    | 0.45 – 1.2 m    | Family and close friends
  Social      | 1.2 – 3.6 m     | Acquaintances / casual interaction
  Public      | > 3.6 m         | Strangers / public speaking

The robot should follow the detected human and hold the *social zone*
(default: 1.5 m).  Students are encouraged to experiment with other zones.

Topics used
-----------
  Subscribed  : /limo_camera/image              (sensor_msgs/Image)
                  – RGB image  (640×480 @ 10 Hz)
                /limo_camera/depth/image_raw    (sensor_msgs/Image)
                  – Depth image in metres (float32, same resolution)
  Published   : /cmd_vel                        (geometry_msgs/Twist)
                  – Velocity command for the differential-drive controller

Quick-start
-----------
After building the package and sourcing the workspace:

  # Terminal 1 – launch the simulation
  ros2 launch hri_workshop hri_workshop.launch.py

  # Terminal 2 – run your node
  ros2 run hri_workshop proxemic_follower

  # Override the target distance from the command line (e.g. personal zone)
  ros2 run hri_workshop proxemic_follower --ros-args -p target_distance:=0.8

  # Useful inspection commands
  ros2 topic list
  ros2 topic echo /cmd_vel
  ros2 topic hz /limo_camera/depth/image_raw

References
----------
  • Hall, E.T. (1966). The Hidden Dimension. Doubleday.
  • OpenCV HOG people detector:
      https://docs.opencv.org/4.x/d5/d33/structcv_1_1HOGDescriptor.html
  • cv_bridge ROS ↔ OpenCV conversion:
      https://docs.ros.org/en/humble/p/cv_bridge/
  • message_filters.ApproximateTimeSynchronizer:
      https://docs.ros.org/en/humble/p/message_filters/
"""

# ---------------------------------------------------------------------------
# Standard library
import math

# ROS 2
import rclpy
from rclpy.node import Node

# ROS 2 message types
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# message_filters lets us synchronise two topics with slightly different
# timestamps (the RGB and depth streams rarely arrive at *exactly* the
# same time).
import message_filters

# cv_bridge converts between ROS Image messages and OpenCV (numpy) arrays
from cv_bridge import CvBridge, CvBridgeError

# OpenCV and NumPy
import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Proxemic zone boundaries (metres) – Hall (1966)
# Feel free to use these constants in your implementation.
# ---------------------------------------------------------------------------
INTIMATE_MAX  = 0.45   # 0 – 45 cm
PERSONAL_MAX  = 1.20   # 45 cm – 1.2 m
SOCIAL_MAX    = 3.60   # 1.2 m – 3.6 m
# Public zone starts at SOCIAL_MAX and has no upper bound.

# Friendly labels used for logging and display
def proxemic_zone_label(distance_m: float) -> str:
    """Return the proxemic zone name for a given distance in metres."""
    if distance_m <= INTIMATE_MAX:
        return 'INTIMATE'
    elif distance_m <= PERSONAL_MAX:
        return 'PERSONAL'
    elif distance_m <= SOCIAL_MAX:
        return 'SOCIAL'
    else:
        return 'PUBLIC'


# ---------------------------------------------------------------------------
class ProxemicFollowerNode(Node):
    """
    ROS 2 node that detects a walking human, measures their distance from
    the depth camera, and drives the Limo robot to maintain a target
    proxemic distance.

    Node parameters (settable from the command line):
        rgb_topic        (str)   – RGB image topic
        depth_topic      (str)   – Depth image topic  (float32, metres)
        cmd_vel_topic    (str)   – Output velocity topic
        target_distance  (float) – Desired distance to the human [m]
        linear_gain      (float) – Proportional gain for forward/backward motion
        angular_gain     (float) – Proportional gain for left/right steering
        max_linear_speed (float) – Speed cap for linear motion [m/s]
        max_angular_speed(float) – Speed cap for angular motion [rad/s]
    """

    def __init__(self):
        super().__init__('proxemic_follower')

        # ------------------------------------------------------------------
        # Declare ROS parameters
        # ------------------------------------------------------------------
        self.declare_parameter('rgb_topic',         '/limo_camera/image')
        self.declare_parameter('depth_topic',       '/limo_camera/depth/image_raw')
        self.declare_parameter('cmd_vel_topic',     '/cmd_vel')
        self.declare_parameter('target_distance',   1.5)    # social zone centre [m]
        self.declare_parameter('linear_gain',       0.5)    # m/s per metre error
        self.declare_parameter('angular_gain',      0.005)  # rad/s per pixel error
        self.declare_parameter('max_linear_speed',  0.3)    # m/s
        self.declare_parameter('max_angular_speed', 0.8)    # rad/s

        rgb_topic          = self.get_parameter('rgb_topic').value
        depth_topic        = self.get_parameter('depth_topic').value
        cmd_vel_topic      = self.get_parameter('cmd_vel_topic').value
        self.target_dist   = self.get_parameter('target_distance').value
        self.linear_gain   = self.get_parameter('linear_gain').value
        self.angular_gain  = self.get_parameter('angular_gain').value
        self.max_lin       = self.get_parameter('max_linear_speed').value
        self.max_ang       = self.get_parameter('max_angular_speed').value

        # ------------------------------------------------------------------
        # cv_bridge and HOG detector
        # ------------------------------------------------------------------
        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Image dimensions (set on first callback)
        self.image_width  = None
        self.image_height = None

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ------------------------------------------------------------------
        # Synchronised subscribers
        # We use ApproximateTimeSynchronizer to pair each RGB frame with
        # the nearest depth frame (within 0.1 s).
        # ------------------------------------------------------------------
        self.rgb_sub   = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1          # maximum allowed time difference [s]
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info(
            f'ProxemicFollower started.\n'
            f'  RGB topic       : {rgb_topic}\n'
            f'  Depth topic     : {depth_topic}\n'
            f'  Target distance : {self.target_dist} m  '
            f'({proxemic_zone_label(self.target_dist)} zone)\n'
            f'  Linear gain     : {self.linear_gain}\n'
            f'  Angular gain    : {self.angular_gain}'
        )

    # -----------------------------------------------------------------------
    def synced_callback(self, rgb_msg: Image, depth_msg: Image):
        """
        Called whenever a matching pair of (RGB, depth) frames arrives.

        Pipeline:
          1. Convert ROS messages → OpenCV arrays          [provided]
          2. Detect people with HOG                        [Task 2]
          3. Select the best detection                     [Task 3]
          4. Estimate distance from the depth image        [Task 4]
          5. Drive the robot with a proxemics-aware controller [Task 5]
        """

        # ==================================================================
        # Step 1 – Convert ROS messages to OpenCV / NumPy arrays
        # ==================================================================
        try:
            rgb_image   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
            # Depth image is float32 where each pixel value is distance in metres.
            # NaN or 0 means "no measurement at that pixel".
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        self.image_height, self.image_width = rgb_image.shape[:2]

        # ==================================================================
        # TODO – Task 1: Explore the image streams
        # ==================================================================
        # Before implementing detection verify you can see both images:
        #
        #   a) Log the shape of rgb_image and depth_image once every ~30
        #      frames:
        #        self.get_logger().info(
        #            f'RGB: {rgb_image.shape}  Depth: {depth_image.shape}')
        #
        #   b) Display the RGB image:
        #        cv2.imshow('RGB', rgb_image)
        #        cv2.waitKey(1)
        #
        #   c) Visualise the depth image.  Because pixel values are floats
        #      (metres) you need to normalise them first:
        #        depth_vis = cv2.normalize(
        #            depth_image, None, 0, 255,
        #            cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        #        cv2.imshow('Depth', depth_vis)
        #        cv2.waitKey(1)
        #
        # Once you can see both images, move on to Task 2.
        # ------------------------------------------------------------------
        # ↓ Write your Task 1 code below this line ↓

        # ↑ End of Task 1 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 2: Detect people with the HOG descriptor
        # ==================================================================
        # Use self.hog.detectMultiScale() exactly as in the human_detector
        # exercise.
        #
        # Hint: for speed, consider running HOG on a half-resolution copy of
        # rgb_image, then multiply all rect coordinates by 2.
        #
        # Signature:
        #   rects, weights = self.hog.detectMultiScale(
        #       image,
        #       winStride = (8, 8),
        #       padding   = (4, 4),
        #       scale     = 1.05,
        #   )
        #
        # rects  – array of (x, y, w, h) bounding boxes
        # weights – confidence score per detection
        # ------------------------------------------------------------------
        # ↓ Write your Task 2 code below this line ↓

        rects   = []   # replace with detectMultiScale output
        weights = []

        # ↑ End of Task 2 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 3: Select the best detection and annotate the image
        # ==================================================================
        # a) If rects is empty → stop the robot and return.
        #
        # b) Select the detection with the highest confidence weight.
        #    Use np.argmax(weights).
        #
        # c) Draw all bounding boxes in green:
        #      cv2.rectangle(rgb_image, (x,y), (x+w, y+h), (0,255,0), 2)
        #
        # d) Draw the best bounding box in red.
        #
        # e) Compute the horizontal pixel centre of the best box:
        #      cx = bx + bw // 2
        #
        # f) Show the annotated rgb_image with cv2.imshow().
        #
        # ------------------------------------------------------------------
        # ↓ Write your Task 3 code below this line ↓

        cx = None   # horizontal pixel coordinate of the detected person

        if cx is None:
            # No detection – stop the robot
            self.cmd_vel_pub.publish(Twist())
            return

        # ↑ End of Task 3 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 4: Estimate the distance to the detected person
        # ==================================================================
        # Now use the DEPTH image to measure how far away the person is.
        #
        # Strategy:
        #   a) Define a small Region Of Interest (ROI) centred on the
        #      middle of the best bounding box (bx, by, bw, bh):
        #
        #        roi_cx = bx + bw // 2
        #        roi_cy = by + bh // 2
        #        half   = 10   # sample a 20×20 pixel patch
        #        roi = depth_image[roi_cy-half : roi_cy+half,
        #                          roi_cx-half : roi_cx+half]
        #
        #   b) Some pixels in the depth image may be NaN (no reading) or
        #      zero (sensor range exceeded).  Filter them out:
        #        valid_depths = roi[np.isfinite(roi) & (roi > 0)]
        #
        #   c) If valid_depths is empty, the sensor has no reading for that
        #      patch.  Stop the robot and return.
        #
        #   d) Compute the median distance:
        #        distance = float(np.median(valid_depths))
        #
        #   e) Log the estimated distance and proxemic zone:
        #        self.get_logger().info(
        #            f'Distance: {distance:.2f} m  '
        #            f'Zone: {proxemic_zone_label(distance)}')
        #
        #   f) Overlay the distance on the annotated image:
        #        label = f'{distance:.2f} m  [{proxemic_zone_label(distance)}]'
        #        cv2.putText(rgb_image, label, (bx, by - 10),
        #                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        # ------------------------------------------------------------------
        # ↓ Write your Task 4 code below this line ↓

        distance = None   # replace with your depth estimate [m]

        if distance is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # ↑ End of Task 4 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 5: Proxemics-aware velocity controller
        # ==================================================================
        # You now know:
        #   cx       – horizontal pixel position of the person
        #   distance – distance to the person in metres
        #
        # Design a proportional controller with TWO components:
        #
        # 1. ANGULAR (heading) control  — same as in human_detector:
        #      pixel_error  = (self.image_width // 2) - cx
        #      angular_z    = self.angular_gain * pixel_error
        #    Clamp to [-self.max_ang, +self.max_ang].
        #
        # 2. LINEAR (distance) control  — new for this exercise:
        #      dist_error   = distance - self.target_dist
        #        • Positive dist_error → person is further than target
        #          → move FORWARD  (positive linear.x)
        #        • Negative dist_error → person is closer than target
        #          → move BACKWARD (negative linear.x)
        #      linear_x = self.linear_gain * dist_error
        #    Clamp to [-self.max_lin, +self.max_lin].
        #
        # Build and publish the Twist:
        #    twist = Twist()
        #    twist.linear.x  = linear_x
        #    twist.angular.z = angular_z
        #    self.cmd_vel_pub.publish(twist)
        #
        # Experiments to try once it is working:
        #   • Change target_distance to 0.8 m (personal zone). What changes?
        #   • Change target_distance to 3.0 m (far social zone). 
        #     Does the robot still follow reliably?
        #   • What is the minimum target distance that is safe for
        #     human-robot interaction in this environment?
        #   • Add a dead-band: only apply linear control when
        #     |dist_error| > 0.1 m so the robot does not jitter.
        #
        # ------------------------------------------------------------------
        # ↓ Write your Task 5 code below this line ↓

        image_center_x = self.image_width // 2
        pixel_error    = image_center_x - cx

        twist = Twist()
        # TODO: fill in twist.linear.x and twist.angular.z using the
        #       proportional controller described above, then publish.
        self.cmd_vel_pub.publish(twist)

        # ↑ End of Task 5 ↑
        # ==================================================================

        cv2.waitKey(1)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ProxemicFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
