# HRI Workshop – Human Detection and Robot Following with the Limo Robot

> **Module:** CMP3103M – Autonomous Mobile Robotics  
> **Platform:** ROS 2 Humble · Gazebo Classic 11 · Ubuntu 22.04  
> **Robot:** AgileX Limo (differential-drive mode)

---

## Table of Contents

1. [Learning Objectives](#1-learning-objectives)
2. [Background](#2-background)
3. [Environment Setup](#3-environment-setup)
4. [Building the Package](#4-building-the-package)
5. [Running the Simulation](#5-running-the-simulation)
6. [Workshop Tasks](#6-workshop-tasks)
   - [Task 1 – Subscribe to the Camera and Display Images](#task-1--subscribe-to-the-camera-and-display-images)
   - [Task 2 – Detect People with the HOG Detector](#task-2--detect-people-with-the-hog-detector)
   - [Task 3 – Select and Annotate the Best Detection](#task-3--select-and-annotate-the-best-detection)
   - [Task 4 – Drive the Robot Towards the Person](#task-4--drive-the-robot-towards-the-person)
7. [Expected Results](#7-expected-results)
8. [Hints and Common Issues](#8-hints-and-common-issues)
9. [Stretch Tasks](#9-stretch-tasks)
10. [Key References](#10-key-references)

---

## 1. Learning Objectives

By the end of this workshop you will be able to:

| # | Skill |
|---|-------|
| 1 | Receive and display images from a simulated robot camera using **ROS 2 Python** and **cv_bridge** |
| 2 | Apply the **HOG (Histogram of Oriented Gradients)** pedestrian detector from OpenCV to find people in a scene |
| 3 | Select the most confident detection and extract its **bounding box** |
| 4 | Implement a simple **proportional controller** to steer a differential-drive robot towards a detected person |
| 5 | Understand the structure of a ROS 2 Python node (subscriber, publisher, callback) |

---

## 2. Background

### The Robot: AgileX Limo

The Limo is a compact, differential-drive ground robot.  In this simulation it is equipped with:

- A **depth camera** (`/limo_camera/image`) – 640 × 480 RGB at ~10 Hz
- A **2D LiDAR** (`/scan`)
- An **IMU** (`/imu`)
- Velocity control via `/cmd_vel` (`geometry_msgs/Twist`)

The robot is spawned at the origin facing the +X direction.

### The Scene

The `hri_world.world` file defines a 10 × 10 m walled arena containing a single **walking human actor**.  The actor is scaled to approximately 0.9 m (child-sized) so it is clearly visible in the camera at 2–4 m range without filling the entire frame.  It patrols a rectangular route:

```
(-3, -3) → (-3, 3) → (3, 3) → (3, -3) → repeat
```

### HOG People Detection

The **Histogram of Oriented Gradients (HOG)** descriptor, combined with a linear SVM, is one of the classical computer vision methods for detecting upright pedestrians.  OpenCV ships with a pre-trained SVM (`HOGDescriptor_getDefaultPeopleDetector()`) so no model download is required.  The detector works at its best when the person occupies roughly 64–128 pixels of height in the image.

---

## 3. Environment Setup

This workshop runs inside the provided **VS Code dev container**.  All required software is pre-installed:

| Software | Version |
|----------|---------|
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11.x |
| OpenCV | 4.x (system) |
| Python | 3.10 |
| cv_bridge | ros-humble-cv-bridge |

Open VS Code and the dev container will start automatically.  The integrated terminal gives you a shell with ROS 2 already sourced.

### Verify the environment

```bash
# Check ROS 2
ros2 --version

# Confirm OpenCV is available in Python
python3 -c "import cv2; print(cv2.__version__)"

# Check the Limo simulation package exists
ros2 pkg list | grep limo_gazebosim
```

---

## 4. Building the Package

All commands should be run from the **workspace root** (`/workspaces/hri`).

```bash
cd /workspaces/hri

# Build with symlink install so Python file changes are picked up immediately
colcon build --symlink-install --packages-select hri_workshop

# Source the workspace overlay
source install/setup.bash
```

> **Tip:** After the first build you only need to re-run `source install/setup.bash` when you add new entry points.  Because of `--symlink-install`, edits to Python files in `src/` are reflected immediately without rebuilding.

---

## 5. Running the Simulation

Open **two terminals** inside the dev container (use the `+` button in the VS Code terminal panel or split the terminal).

### Terminal 1 – Launch Gazebo + Limo

```bash
source /workspaces/hri/install/setup.bash
ros2 launch hri_workshop hri_workshop.launch.py
```

Wait until you see `Gazebo started successfully` in the log (can take 20–30 seconds on first run).  A Gazebo window should open showing the arena with the walking human actor.

Optional flags:

```bash
# Launch with RViz2 for sensor visualisation
ros2 launch hri_workshop hri_workshop.launch.py use_rviz:=true

# Headless (no GUI) – useful on slow machines
ros2 launch hri_workshop hri_workshop.launch.py headless:=true
```

### Terminal 2 – Inspect topics

```bash
source /workspaces/hri/install/setup.bash

# List all active topics
ros2 topic list

# Verify the camera is publishing
ros2 topic hz /limo_camera/image

# Inspect a single camera message (press Ctrl+C to stop)
ros2 topic echo /limo_camera/image --once
```

You should see `/limo_camera/image` publishing at approximately 10 Hz.

---

## 6. Workshop Tasks

Open the template node in your editor:

```
src/hri_workshop/hri_workshop/human_detector.py
```

All four tasks are marked with `TODO` blocks inside the `image_callback` method.  Complete them in order.

---

### Task 1 – Subscribe to the Camera and Display Images

**Goal:** Confirm you are receiving images from the simulation.

**What to do:**

1. Find the `TODO – Task 1` block in `image_callback`.
2. Add a `cv2.imshow()` call to display the raw camera frame.

```python
cv2.imshow('Limo Camera', cv_image)
cv2.waitKey(1)
```

3. Optionally log the image shape every N frames using a counter:

```python
# In __init__:
self.frame_count = 0

# In image_callback:
self.frame_count += 1
if self.frame_count % 30 == 0:
    self.get_logger().info(f'Image shape: {cv_image.shape}')
```

4. Run the node:

```bash
ros2 run hri_workshop human_detector
```

**Checkpoint ✓**  An OpenCV window opens and you can see the live camera feed.  The walking human should be visible when the actor passes in front of the robot.

---

### Task 2 – Detect People with the HOG Detector

**Goal:** Use `self.hog.detectMultiScale()` to find bounding boxes around people.

**What to do:**

1. Find the `TODO – Task 2` block.
2. Call `detectMultiScale` on `cv_image`:

```python
rects, weights = self.hog.detectMultiScale(
    cv_image,
    winStride=(8, 8),
    padding=(4, 4),
    scale=1.05,
)
```

3. Log the number of detections:

```python
self.get_logger().info(f'Detections: {len(rects)}')
```

**Parameters explained:**

| Parameter | Meaning | Effect of increasing |
|-----------|---------|----------------------|
| `winStride` | Step size of the sliding window | Faster, but may miss detections |
| `padding` | Border around each window | More context, slightly slower |
| `scale` | Pyramid downscaling factor (>1) | Fewer scales → faster, coarser |

> **Performance tip:** If the node is too slow, resize the image to half size before detection:
> ```python
> small = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
> rects, weights = self.hog.detectMultiScale(small, ...)
> # Scale bounding boxes back up
> rects = rects * 2  # multiply all coordinates by 2
> ```

**Checkpoint ✓**  When the actor walks past you see log messages like `Detections: 1` or `Detections: 2`.

---

### Task 3 – Select and Annotate the Best Detection

**Goal:** Pick the highest-confidence bounding box, draw it on the image, and compute the horizontal centre of the person.

**What to do:**

1. Find the `TODO – Task 3` block.
2. Handle the case where no person is detected:

```python
if len(rects) == 0:
    self.get_logger().warn('No person detected – stopping robot')
    self.cmd_vel_pub.publish(Twist())
    cv2.imshow('Limo Camera', cv_image)
    cv2.waitKey(1)
    return
```

3. Select the best (highest-weight) detection:

```python
best_idx = int(np.argmax(weights))
bx, by, bw, bh = rects[best_idx]
```

4. Draw all detections in green, the best one in red:

```python
for (x, y, w, h) in rects:
    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

cv2.rectangle(cv_image, (bx, by), (bx + bw, by + bh), (0, 0, 255), 3)
```

5. Compute the horizontal centre:

```python
cx = bx + bw // 2
```

6. Mark the centre on the image:

```python
cv2.circle(cv_image, (cx, by + bh // 2), 6, (255, 0, 0), -1)
cv2.imshow('Limo Camera', cv_image)
```

**Checkpoint ✓**  The camera window shows green boxes around all detections and a red box with a blue dot at the centre of the best one.

---

### Task 4 – Drive the Robot Towards the Person

**Goal:** Use the horizontal pixel error to steer the robot so it faces and approaches the detected person.

**What to do:**

1. Find the `TODO – Task 4` block.
2. Compute the horizontal error (distance from image centre to detection centre):

```python
image_center_x = self.image_width // 2
error = image_center_x - cx
# Positive error → person is to the LEFT  → turn left  (positive angular.z)
# Negative error → person is to the RIGHT → turn right (negative angular.z)
```

3. Compute angular velocity using the proportional gain:

```python
angular_z = self.angular_gain * error
```

4. Publish the Twist command:

```python
twist = Twist()
twist.linear.x  = self.linear_speed
twist.angular.z = angular_z
self.cmd_vel_pub.publish(twist)
```

5. Add a dead-band to avoid jitter when the person is already centred:

```python
if abs(error) < 20:   # 20 pixel threshold
    angular_z = 0.0
```

**Checkpoint ✓**  The Limo robot rotates towards the detected person and then drives forward, maintaining the person roughly in the centre of its camera view.

---

## 7. Expected Results

After completing all four tasks your node should:

1. Display a live annotated camera window showing bounding boxes around the person.
2. Stop the robot (`Twist()` with all zeros) when no person is visible.
3. Rotate the robot towards the person when they are detected off-centre.
4. Drive the robot forward while keeping the person centred.

You can verify the velocity commands being sent:

```bash
ros2 topic echo /cmd_vel
```

Expected output when person is visible and centred:

```
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: ~0.0   # close to zero when centred
```

---

## 8. Hints and Common Issues

### The OpenCV window does not appear

Make sure the devcontainer's virtual display is running.  Open a browser to [http://localhost:5801](http://localhost:5801) (noVNC) to see the graphical desktop, or set the `DISPLAY` variable:

```bash
export DISPLAY=:1
```

### `cv2.error: (-215) ...` when calling detectMultiScale

The image may have an unexpected format.  Print `cv_image.dtype` and `cv_image.shape` to check.  HOG expects `uint8` data.

### No detections even when human is visible

- The actor may be too small (< 40 px height) or too close (> ~300 px height).  Try adjusting the `scale` parameter to `1.02` for finer multi-scale search.
- HOG performs best on upright pedestrians.  Make sure the camera is level.
- Lower the minimum detection size: add `minSize=(30, 60)` to `detectMultiScale`.

### Robot does not move

- Check `ros2 topic echo /cmd_vel` – are you publishing?
- Make sure the simulation is running (Terminal 1).
- The `twist_watchdog` node stops the robot if no message arrives for 0.5 s – keep publishing even when the robot should be stationary.

### `ModuleNotFoundError: No module named 'hri_workshop'`

You need to rebuild and re-source:

```bash
colcon build --symlink-install --packages-select hri_workshop
source install/setup.bash
```

---

## 9. Stretch Tasks

Completed the four main tasks early?  Try these extensions:

### A – Slow down as the robot gets closer
The depth image (topic `/limo_camera/depth/camera_image_depth`) gives per-pixel depth in metres.  Sample the depth at the detection centre to estimate distance and reduce `linear.x` as the robot approaches.

### B – Non-maximum suppression
`detectMultiScale` may return overlapping boxes for the same person.  Apply OpenCV's `groupRectangles` or implement IoU-based NMS to clean up the detections.

### C – Switch to a deep-learning detector
Replace HOG with the pre-trained **YOLOv8-nano** or **MobileNet SSD** (available via `ultralytics` or `cv2.dnn`).  Compare detection quality at different scales.

### D – Track across frames
Instead of detecting from scratch every frame, implement a tracker (e.g. `cv2.TrackerCSRT_create()`) to follow the bounding box between detections and reduce CPU load.

### E – Use the laser scanner
Subscribe to `/scan` (`sensor_msgs/LaserScan`) and use the closest obstacle distance in the forward arc to stop the robot before colliding with the person.

---

## 10. Key References

| Resource | URL |
|----------|-----|
| OpenCV HOG Descriptor | https://docs.opencv.org/4.x/d5/d33/structcv_1_1HOGDescriptor.html |
| cv_bridge ROS 2 | https://docs.ros.org/en/humble/p/cv_bridge/ |
| geometry_msgs/Twist | https://docs.ros2.org/humble/api/geometry_msgs/msg/Twist.html |
| sensor_msgs/Image | https://docs.ros2.org/humble/api/sensor_msgs/msg/Image.html |
| rclpy API | https://docs.ros.org/en/humble/p/rclpy/ |
| ROS 2 Humble tutorials | https://docs.ros.org/en/humble/Tutorials.html |
| Dalal & Triggs HOG paper | https://lear.inrialpes.fr/people/triggs/pubs/Dalal-cvpr05.pdf |
| AgileX Limo robot | https://global.agilex.ai/products/limo |
