#!/usr/bin/env python3
"""
human_detector.py  –  HRI Workshop Student Template
====================================================
University of Lincoln  |  CMP3103M / HRI Workshop

Overview
--------
This node subscribes to the Limo robot's onboard depth camera, detects a
walking human using OpenCV, and drives the robot towards that person by
publishing velocity commands to /cmd_vel.

Your job
--------
Work through the four tasks below (marked with TODO blocks).  Each task
builds on the previous one.  Read every comment carefully before writing
any code.

Topics used
-----------
  Subscribed  : /limo_camera/image   (sensor_msgs/Image)
                  – RGB image from the Limo depth camera (640×480 @ 10 Hz)
  Published   : /cmd_vel             (geometry_msgs/Twist)
                  – velocity command for the differential-drive controller

Quick-start
-----------
After building the package and sourcing the workspace:

  # Terminal 1 – launch the simulation
  ros2 launch hri_workshop hri_workshop.launch.py

  # Terminal 2 – run your node
  ros2 run hri_workshop human_detector

  # Useful inspection commands
  ros2 topic list                          # see all active topics
  ros2 topic hz /limo_camera/image         # check camera publish rate
  ros2 topic echo /cmd_vel                 # verify your velocity commands

References
----------
  • OpenCV HOG people detector:
      https://docs.opencv.org/4.x/d5/d33/structcv_1_1HOGDescriptor.html
  • cv_bridge ROS ↔ OpenCV conversion:
      https://docs.ros.org/en/humble/p/cv_bridge/
  • geometry_msgs/Twist message:
      https://docs.ros2.org/humble/api/geometry_msgs/msg/Twist.html
  • ROS 2 Python client library (rclpy):
      https://docs.ros.org/en/humble/p/rclpy/
"""

# ---------------------------------------------------------------------------
# Standard library imports
import math

# ROS 2 Python client library
import rclpy
from rclpy.node import Node

# ROS 2 message types
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# cv_bridge converts between ROS Image messages and OpenCV (numpy) arrays
from cv_bridge import CvBridge, CvBridgeError

# OpenCV – already installed system-wide in this dev container
import cv2
import numpy as np


# ---------------------------------------------------------------------------
class HumanDetectorNode(Node):
    """
    ROS 2 node that detects a walking human in the camera feed and steers
    the Limo robot towards them.

    Node parameters (settable from the launch file or command line):
        camera_topic  (str)   – ROS topic for the incoming camera image
        cmd_vel_topic (str)   – ROS topic for outgoing velocity commands
        linear_speed  (float) – forward speed while approaching (m/s)
        angular_gain  (float) – proportional gain for heading correction
    """

    def __init__(self):
        super().__init__('human_detector')

        # ------------------------------------------------------------------
        # Node parameters – students can override from the command line:
        #   ros2 run hri_workshop human_detector \
        #       --ros-args -p camera_topic:=/limo_camera/image
        # ------------------------------------------------------------------
        self.declare_parameter('camera_topic',  '/limo_camera/image')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed',  0.2)   # m/s forward speed
        self.declare_parameter('angular_gain',  0.005)  # rad/s per pixel error

        camera_topic  = self.get_parameter('camera_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_gain  = self.get_parameter('angular_gain').value

        # ------------------------------------------------------------------
        # cv_bridge instance – reuse this object for every conversion
        # ------------------------------------------------------------------
        self.bridge = CvBridge()

        # ------------------------------------------------------------------
        # HOG (Histogram of Oriented Gradients) people detector.
        # This is a classical computer vision detector built into OpenCV –
        # no external model files required.
        #
        # getDefaultPeopleDetector() returns the SVM weights trained on the
        # INRIA pedestrian dataset.  It works best for upright, fully
        # visible people at moderate scale (roughly 64–128 px tall in the
        # image).
        # ------------------------------------------------------------------
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Image dimensions (filled in on first callback)
        self.image_width  = None
        self.image_height = None

        # ------------------------------------------------------------------
        # ROS subscriber and publisher
        # ------------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10          # queue size
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.get_logger().info(
            f'HumanDetector started.\n'
            f'  Camera topic  : {camera_topic}\n'
            f'  Cmd_vel topic : {cmd_vel_topic}\n'
            f'  Linear speed  : {self.linear_speed} m/s\n'
            f'  Angular gain  : {self.angular_gain} rad/s per pixel'
        )

    # -----------------------------------------------------------------------
    def image_callback(self, msg: Image):
        """
        Called every time a new camera frame arrives.

        The pipeline is:
          1. Convert ROS Image → OpenCV BGR image     (already done for you)
          2. Detect people using HOG                  (Task 2)
          3. Select the largest / best detection      (Task 3)
          4. Compute heading error & drive the robot  (Task 4)
          5. Visualise detections                     (Task 3 / optional)
        """

        # ==================================================================
        # Step 1  –  Convert ROS Image message to an OpenCV array
        # ==================================================================
        # cv_bridge does this conversion for us.  We ask for 'bgr8' which
        # is the standard 3-channel 8-bit format expected by most OpenCV
        # functions.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Store image dimensions once (used later for centring calculation)
        self.image_height, self.image_width = cv_image.shape[:2]

        # ==================================================================
        # TODO – Task 1: Explore the image stream
        # ==================================================================
        # Before implementing detection, verify you are receiving images:
        #
        #   a) Log the image shape to the terminal every ~30 frames so you
        #      can confirm the camera is working:
        #
        #      self.get_logger().info(f'Image shape: {cv_image.shape}')
        #
        #   b) Display the raw image in a window using cv2.imshow():
        #
        #      cv2.imshow('Limo Camera', cv_image)
        #      cv2.waitKey(1)   # must call this for the window to refresh
        #
        # Once you can see the live image, move on to Task 2.
        # ------------------------------------------------------------------
        # ↓ Write your Task 1 code below this line ↓

        # ↑ End of Task 1 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 2: Detect people with the HOG detector
        # ==================================================================
        # self.hog.detectMultiScale() runs the sliding-window HOG detector
        # over the image at multiple scales.
        #
        # Signature:
        #   rects, weights = self.hog.detectMultiScale(
        #       image,
        #       winStride   = (8, 8),   # step size of the sliding window
        #       padding     = (4, 4),   # padding around each window
        #       scale       = 1.05,     # pyramid scaling factor
        #   )
        #
        # Returns:
        #   rects   – numpy array of shape (N, 4) with columns [x, y, w, h]
        #             where (x, y) is the top-left corner of each bounding box
        #   weights – confidence score for each detection (higher = better)
        #
        # Hint: if you get too many false positives, increase 'scale' to
        # e.g. 1.1, or add a minimum confidence threshold on 'weights'.
        #
        # IMPORTANT: HOG works on grayscale or colour images, but runs
        # faster on a smaller image.  Consider resizing cv_image to half
        # size before detection, then scaling rects back up.
        # ------------------------------------------------------------------
        # ↓ Write your Task 2 code below this line ↓

        rects   = []   # replace this with the output of detectMultiScale()
        weights = []

        # ↑ End of Task 2 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 3: Select the best detection and draw bounding boxes
        # ==================================================================
        # After calling detectMultiScale you will (usually) have multiple
        # bounding boxes.  Follow these steps:
        #
        #   a) If rects is empty (no person detected):
        #        – publish a zero Twist to stop the robot
        #        – return early from this callback
        #
        #   b) If there are multiple detections, pick the one with the
        #      highest confidence score (use numpy argmax on 'weights').
        #
        #   c) Draw ALL bounding boxes in green (BGR: 0, 255, 0):
        #        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,255,0), 2)
        #
        #   d) Draw the BEST bounding box in red  (BGR: 0, 0, 255):
        #        cv2.rectangle(cv_image, (bx, by), (bx+bw, by+bh), (0,0,255), 3)
        #
        #   e) Compute the horizontal centre of the best bounding box:
        #        cx = bx + bw // 2
        #
        #   f) Show the annotated image with cv2.imshow()
        #
        # ------------------------------------------------------------------
        # ↓ Write your Task 3 code below this line ↓

        cx = None   # horizontal pixel coordinate of the detected person centre

        # ↑ End of Task 3 ↑
        # ==================================================================

        # ==================================================================
        # TODO – Task 4: Drive the robot towards the detected person
        # ==================================================================
        # Now that you know WHERE in the image the person is (cx), you can
        # steer the robot towards them using a simple proportional controller.
        #
        # The idea:
        #   • If the person is in the CENTRE of the image → drive straight
        #   • If the person is to the LEFT → turn left (positive angular.z)
        #   • If the person is to the RIGHT → turn right (negative angular.z)
        #
        # Steps:
        #   a) Compute the error = image_centre_x - cx
        #      (positive error  → person is left of centre  → turn left)
        #
        #   b) Compute angular velocity:
        #        angular_z = self.angular_gain * error
        #
        #   c) Set linear velocity to self.linear_speed
        #      (optional: slow down when |error| is large, e.g. multiply
        #       linear_speed by a factor that decreases with |error|)
        #
        #   d) Fill in and publish a Twist message:
        #        twist = Twist()
        #        twist.linear.x  = ...
        #        twist.angular.z = ...
        #        self.cmd_vel_pub.publish(twist)
        #
        # Tip: add a dead-band – only correct heading when |error| > 20 px.
        #
        # Safety note:  the twist_watchdog node will zero the velocity if no
        # /cmd_vel message is received for 0.5 s, so always publish a Twist
        # even if the robot should be stationary (publish a zero Twist).
        # ------------------------------------------------------------------
        # ↓ Write your Task 4 code below this line ↓

        if cx is not None:
            image_center_x = self.image_width // 2
            error = image_center_x - cx

            twist = Twist()
            # TODO: set twist.linear.x and twist.angular.z
            self.cmd_vel_pub.publish(twist)
        else:
            # No detection – stop the robot
            self.cmd_vel_pub.publish(Twist())

        # ↑ End of Task 4 ↑
        # ==================================================================

        # Keep any open OpenCV windows updated
        cv2.waitKey(1)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectorNode()
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
