# CMP3103M – HRI Workshop: Proxemics-Aware Robot Following

**Module:** CMP3103M Autonomous Mobile Robotics  
**Topic:** Human-Robot Interaction – Part 2 (Human-Aware Navigation)  
**Package:** `hri_workshop`  
**Node to implement:** `proxemic_follower`

---

## 1. Background

### What is Proxemics?

*Proxemics* is the study of human use of space as a form of non-verbal communication. The term was coined by anthropologist **Edward T. Hall** in his 1966 book *The Hidden Dimension*.

Hall identified a set of **personal space zones** around each individual. These zones are not fixed physical barriers, but socially agreed distances that govern the comfort level of an interaction:

| Zone         | Distance Range | Relationship / Context |
|--------------|---------------|------------------------|
| **Intimate** | 0 – 45 cm     | Physical contact, comforting, close intimacy |
| **Personal** | 45 cm – 1.2 m | Family, close friends, peripersonal space |
| **Social**   | 1.2 – 3.6 m   | Acquaintances, casual interaction, normal voice |
| **Public**   | > 3.6 m       | Strangers, public speaking, large audiences |

Each zone is further divided into a *close phase* and a *far phase*. Detailed characteristics:

- **Intimate – Close phase (0–15 cm):** vision is blurred; senses of smell and heat are effective.  
- **Personal – Far phase (75–120 cm):** arm's length; features clearly visible, moderate voice volume.  
- **Social – Close phase (1.2–2.1 m):** no touching without special effort; normal voice audible.  
- **Public – Close phase (3.7–7.6 m):** evasive action is possible; loud but not full voice volume.

> **Note:** Hall's research was conducted in the United States. Zone sizes are known to vary across cultures and contexts (e.g., public transport enforces intimate-space violations).

### Why Does Proxemics Matter for Robots?

When a robot navigates in a shared space with humans it is **not just another obstacle**. The human-robot spatial relationship directly affects:

- **Comfort** – avoiding the intimate and personal zones of strangers reduces stress.
- **Naturalness** – respecting social norms makes the robot feel less intrusive.
- **Sociability** – adhering to cultural distance conventions signals awareness of the human.

One way to encode proxemics in a robot's navigation stack is to add a **Gaussian cost function** centred on the person into the robot's cost-map (Lu et al., 2014). Path-planners (Dijkstra, A\*) then naturally route the robot away from humans unless a specific social task requires getting closer.

In this workshop we take a simpler but equivalent approach: **direct proxemic distance control** using a proportional velocity controller.

---

## 2. Learning Objectives

By the end of this workshop you will be able to:

1. Explain Hall's four proxemic zones and their relevance to HRI.
2. Subscribe to and synchronise a robot's RGB and depth camera streams in ROS 2.
3. Detect a walking human in an image using the OpenCV HOG people detector.
4. Estimate the distance to a detected person using a depth image.
5. Implement a proportional controller that makes the robot follow a human while maintaining a target proxemic distance.
6. Evaluate the effect of changing the target zone on robot behaviour.

---

## 3. Prerequisites

- You have completed the **human_detector** workshop (you can detect people and steer towards them with pixel-level control).
- The `hri_workshop` package is built and sourced:

```bash
cd ~/ros2_ws
colcon build --packages-select hri_workshop
source install/setup.bash
```

- You are familiar with `sensor_msgs/Image`, `geometry_msgs/Twist`, and `rclpy`.

---

## 4. Package Overview

```
hri_workshop/
├── hri_workshop/
│   ├── human_detector.py       ← previous workshop (reference)
│   └── proxemic_follower.py    ← YOUR FILE FOR THIS WORKSHOP
├── launch/
│   └── hri_workshop.launch.py  ← launches Gazebo + Limo + walking human
├── worlds/
│   └── hri_world.world         ← Gazebo world with human actor
└── config/
    └── hri_workshop.rviz       ← RViz2 configuration
```

Your work goes entirely inside **`hri_workshop/proxemic_follower.py`**.

---

## 5. Running the Simulation

**Terminal 1** – launch Gazebo with the Limo robot and a walking human actor:

```bash
ros2 launch hri_workshop hri_workshop.launch.py
```

**Terminal 2** – run your node (once you have implemented it):

```bash
ros2 run hri_workshop proxemic_follower
```

**Useful inspection commands:**

```bash
# List all active topics
ros2 topic list

# Check that the depth camera is publishing
ros2 topic hz /limo_camera/depth/image_raw

# Inspect the velocity commands your node is publishing
ros2 topic echo /cmd_vel

# Check the message type of the depth topic
ros2 topic info /limo_camera/depth/image_raw

# Override the target distance to 0.8 m (personal zone)
ros2 run hri_workshop proxemic_follower --ros-args -p target_distance:=0.8
```

---

## 6. Workshop Tasks

Open `hri_workshop/proxemic_follower.py`. The file contains five `TODO` blocks – one per task. Work through them in order.

---

### Task 1 – Explore the Image Streams

**Goal:** Confirm that both the RGB and depth images are arriving correctly before writing any detection code.

1. Inside `synced_callback`, log the shape of both images every ~30 frames.
2. Display the RGB image using `cv2.imshow('RGB', rgb_image)`.
3. Visualise the depth image. Because pixel values are floating-point metres you must normalise them for display:

```python
depth_vis = cv2.normalize(
    depth_image, None, 0, 255,
    cv2.NORM_MINMAX, dtype=cv2.CV_8U)
cv2.imshow('Depth', depth_vis)
cv2.waitKey(1)
```

**Expected result:** Two windows appear – one showing the colour camera feed and one showing a grey-scale depth map where closer objects are darker (or lighter, depending on normalisation direction).

**Questions to consider:**
- What is the resolution of both images?
- Are there any pixels in the depth image that appear very bright or very dark? What do these represent?

---

### Task 2 – Detect People with the HOG Descriptor

**Goal:** Identify the bounding boxes of walking humans in the RGB image.

Use the pre-configured `self.hog` detector (already set up at the top of `__init__`):

```python
rects, weights = self.hog.detectMultiScale(
    rgb_image,
    winStride = (8, 8),
    padding   = (4, 4),
    scale     = 1.05,
)
```

> **Speed tip:** Run HOG on a half-resolution image, then multiply all rectangle coordinates by 2.  
> **Accuracy tip:** If you get many false positives, increase `scale` to `1.1` or filter detections by a minimum weight threshold (e.g. `weights > 0.5`).

**Expected result:** When you print `rects` and `weights` to the terminal you should see bounding boxes appearing around the simulated human actor.

---

### Task 3 – Select the Best Detection and Annotate the Image

**Goal:** Choose the most confident detection and compute its horizontal centre pixel `cx`.

Steps:
1. If `rects` is empty, stop the robot (`self.cmd_vel_pub.publish(Twist())`) and `return`.
2. Pick the detection with the highest weight: `best = int(np.argmax(weights))`.
3. Draw **all** bounding boxes in **green** (BGR `(0, 255, 0)`).
4. Draw the **best** bounding box in **red** (BGR `(0, 0, 255)`).
5. Compute the horizontal pixel centre: `cx = bx + bw // 2`.
6. Display the annotated image.

**Expected result:** A window shows the camera feed with green boxes around all people and a red box around the best detection.

---

### Task 4 – Estimate Distance with the Depth Camera

**Goal:** Use the depth image to measure how far the detected person is from the robot in metres.

Strategy – sample a small patch of the depth image centred on the best bounding box:

```python
roi_cx = bx + bw // 2
roi_cy = by + bh // 2
half   = 10   # 20×20 pixel patch

roi = depth_image[roi_cy - half : roi_cy + half,
                  roi_cx - half : roi_cx + half]

valid_depths = roi[np.isfinite(roi) & (roi > 0)]

if len(valid_depths) == 0:
    self.cmd_vel_pub.publish(Twist())
    return

distance = float(np.median(valid_depths))
```

> **Why median and not mean?** Depth sensors often produce outlier readings (NaN, 0, or very large values). The median is more robust to these outliers than the mean.

After computing the distance:
- Log it with the proxemic zone label: `proxemic_zone_label(distance)` (already defined at the top of the file).
- Overlay it on the image using `cv2.putText`.

**Expected result:** The terminal shows messages such as `Distance: 2.34 m  Zone: SOCIAL`. The annotated image shows the distance label next to the bounding box.

**Questions to consider:**
- What happens to the depth reading as the human moves further away?
- At what distance does the depth sensor start returning invalid readings?

---

### Task 5 – Proxemics-Aware Velocity Controller

**Goal:** Drive the robot so that it follows the human while staying within the target proxemic zone.

Implement a **two-axis proportional controller**:

#### Angular control (heading)

Keep the person centred horizontally in the image:

```
pixel_error = (image_width / 2) − cx
angular_z   = angular_gain × pixel_error
```

Clamp `angular_z` to `[−max_angular_speed, +max_angular_speed]`.

#### Linear control (distance)

Maintain the target proxemic distance:

```
dist_error = distance − target_distance
linear_x   = linear_gain × dist_error
```

- `dist_error > 0` → person is **further** than target → move **forward**.  
- `dist_error < 0` → person is **closer** than target → move **backward**.

Clamp `linear_x` to `[−max_linear_speed, +max_linear_speed]`.

Build and publish the Twist message:

```python
twist = Twist()
twist.linear.x  = linear_x
twist.angular.z = angular_z
self.cmd_vel_pub.publish(twist)
```

#### Default parameters

| Parameter          | Default | Description                        |
|--------------------|---------|-------------------------------------|
| `target_distance`  | 1.5 m   | Target proxemic distance (social zone centre) |
| `linear_gain`      | 0.5     | m/s per metre of distance error    |
| `angular_gain`     | 0.005   | rad/s per pixel of heading error   |
| `max_linear_speed` | 0.3 m/s | Linear speed cap                   |
| `max_angular_speed`| 0.8 rad/s | Angular speed cap                |

All parameters can be overridden at launch:

```bash
ros2 run hri_workshop proxemic_follower \
    --ros-args \
    -p target_distance:=0.9 \
    -p linear_gain:=0.6
```

---

## 7. Experiments and Discussion Questions

Once your node is working at the default social-zone distance, try the following experiments:

1. **Zone comparison:** Run the node with `target_distance:=0.3` (intimate), `0.9` (personal), `1.5` (social, default), `4.0` (public). How does the robot's behaviour change? Which zone feels most natural?

2. **Dead-band:** Add a condition that only applies the linear correction when `|dist_error| > 0.1 m`. Does the robot become more stable?

3. **Speed scaling:** Multiply `linear_x` by a factor that decreases as `|pixel_error|` increases, so the robot slows down while it is turning. Does this improve tracking?

4. **Cultural context:** Hall's original research was conducted in the United States. How might proxemic zones differ in other cultures, and how would you parameterise your node to accommodate this?

5. **Safety:** Is it safe for the robot to move backward? What sensor or logic would you add to prevent collisions behind the robot?

6. **Gaussian cost-map (extension):** Instead of direct velocity control, how could you encode the proxemic zones as a cost function layered into the ROS 2 Nav2 cost-map? Sketch the Gaussian function you would use.

---

## 8. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `No module named 'message_filters'` | Package not installed | `sudo apt install ros-humble-message-filters` |
| Depth window is completely black | Depth values are very large floats | `cv2.normalize` with `cv2.NORM_MINMAX` |
| Robot does not stop when human disappears | No fallback `Twist()` on empty detection | Add `publish(Twist())` in the empty-rects branch |
| HOG detects many false positives | Scale too fine | Increase `scale` to `1.1` or add a weight threshold |
| `ApproximateTimeSynchronizer` never fires | Topics not publishing simultaneously | Check `ros2 topic hz` for both topics |
| Robot drives backward into obstacles | No rear obstacle check | Add a laser-scan guard (extension task) |

---

## 9. References

- Hall, E.T. (1966). *The Hidden Dimension*. Doubleday, New York.  
- Kruse, T., Pandey, A.K., Alami, R. and Kirsch, A. (2013). Human-aware robot navigation: A survey. *Robotics and Autonomous Systems*, 61(12), pp.1726–1743.  
- Lu, D.V., Hershberger, D. and Smart, W.D. (2014). Layered costmaps for context-sensitive navigation. *IEEE/RSJ IROS*, pp.709–715.  
- OpenCV HOG People Detector: <https://docs.opencv.org/4.x/d5/d33/structcv_1_1HOGDescriptor.html>  
- ROS 2 `message_filters`: <https://docs.ros.org/en/humble/p/message_filters/>  
- ROS 2 `cv_bridge`: <https://docs.ros.org/en/humble/p/cv_bridge/>
