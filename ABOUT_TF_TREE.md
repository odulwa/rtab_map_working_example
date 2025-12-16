# RTAB-Map TF Tree Configuration

## Overview

This document details the Transform (TF) tree hierarchy used in this RTAB-Map SLAM example with D435i RealSense camera and Madgewick-filtered IMU data.

## Transform Hierarchy

```
map (Global reference, FIXED)
  │
  └─────────────────────────────── [RTAB-Map: loop closure] ──────────────────────────────
                                                │
                                                ▼
                                           odom (Local odometry frame)
                                                │
                                                └─ [RTAB-Map: visual odometry]
                                                   │
                                                   ▼
                                            camera_link ◄─── ROBOT BASE FRAME
                                                   │
                                                   │ (STATIC transforms from bag's /tf_static)
                                                   ▼
                                            camera_color_frame
                                                   │
                                                   ▼
                                            camera_color_optical_frame
                                                   │
                                                   └─ camera_imu_optical_frame ◄─ /imu/data_filtered published here

                                            (Other static branches)
                                                   │
                                            camera_depth_frame
                                            camera_depth_optical_frame
                                            camera_gyro_frame
                                            camera_accel_frame
                                            etc...
```

## Key Frame Descriptions

### Dynamic Transforms (Published by RTAB-Map)

| Transform | Publisher | Frequency | Purpose |
|-----------|-----------|-----------|---------|
| `map → odom` | RTAB-Map (loop closure) | Variable (~0.1-1 Hz) | Global map corrections from loop closure detection |
| `odom → camera_link` | RTAB-Map (RGB-D odometry) | ~30 Hz | Visual odometry from depth/color image tracking |

### Static Transforms (From Bag's /tf_static)

All sensor-to-camera transforms are static and come from the RealSense camera hardware calibration stored in the bag:

| Transform | Source | Purpose |
|-----------|--------|---------|
| `camera_link → camera_color_frame` | Bag /tf_static | Color sensor offset |
| `camera_color_frame → camera_color_optical_frame` | Bag /tf_static | Optical frame rotation (90° pitch) |
| `camera_color_optical_frame → camera_imu_optical_frame` | Bag /tf_static | IMU sensor alignment to color camera |
| `camera_link → camera_depth_frame` | Bag /tf_static | Depth sensor offset |
| `camera_link → camera_gyro_frame` | Bag /tf_static | Gyro sensor offset |
| `camera_link → camera_accel_frame` | Bag /tf_static | Accelerometer sensor offset |
| (+ all sensor frame optical frame rotations) | Bag /tf_static | Various sensor-specific rotations |

## Why camera_link (Not camera_color_optical_frame)?

### The Problem with camera_color_optical_frame

If we set `frame_id:=camera_color_optical_frame`, RTAB-Map would try to publish:
- `odom → camera_color_optical_frame` (dynamic, from visual odometry)

But `camera_color_optical_frame` **already has a parent** in `/tf_static`:
- `camera_color_optical_frame` has parent `camera_color_frame` (static, from bag)

**Result:** TF conflict! A frame cannot have two parents - one static, one dynamic.

### The Solution: Use camera_link

`camera_link` is the **root of all sensor chains** from the RealSense hardware:
- It has **NO static parent** in the bag's `/tf_static`
- All sensors branch from it: color, depth, gyro, accel
- Making it dynamic via `odom → camera_link` doesn't conflict with anything
- All child transforms automatically move with camera_link

**Correct hierarchy:**
```
odom (dynamic)
  └─ camera_link (dynamic: odom → camera_link)
      └─ camera_color_frame (static: from bag)
          └─ camera_color_optical_frame (static: from bag)
              └─ camera_imu_optical_frame (static: from bag)
```

## IMU Data Integration

### Madgewick Filter

The bag contains:
- **Raw IMU:** `/camera/camera/imu` (6000+ messages, raw gyro + accel)
- **Filtered IMU:** `/imu/data_filtered` (5000+ messages, Madgewick filter output)

The Madgewick filter was pre-computed during bag recording and produces:
- **Orientation** (quaternion): Estimated gravity-aligned orientation
- **Angular velocity**: Raw gyro measurements
- **Linear acceleration**: Raw accelerometer measurements
- **Frame ID**: `camera_imu_optical_frame` (aligned with color camera frame)

### IMU in TF Tree

The orientation data from `/imu/data_filtered` is published in `camera_imu_optical_frame`, which sits in the TF tree under `camera_color_optical_frame`:

```
camera_color_optical_frame
    └─ camera_imu_optical_frame ◄─ IMU orientation estimate here
```

When RTAB-Map requests the IMU data (with `subscribe_imu:=true`), it:
1. Reads `/imu/data_filtered` messages (frame_id: `camera_imu_optical_frame`)
2. Looks up the TF transform from `camera_link` to `camera_imu_optical_frame`
3. Transforms the IMU orientation into the `camera_link` frame
4. Uses it to refine odometry estimates and improve gravity alignment

### Why NOT /camera/camera/imu?

We use `/imu/data_filtered` (not `/camera/camera/imu`) because:
1. **Madgewick filter already computed** - no need to re-filter during playback
2. **Orientation estimates** - the filter provides gravity-aligned orientation quaternions
3. **Better SLAM performance** - oriented IMU hints help RTAB-Map's visual odometry
4. **Matches online examples** - this is the standard practice (raw → madgewick → SLAM)

## Data Flow During Playback

```
1. BAG PLAYBACK (@0.2x speed, ~31 second recording)
   │
   ├─ Publishes sensor data:
   │  ├─ /camera/camera/color/image_raw
   │  ├─ /camera/camera/depth/image_rect_raw
   │  ├─ /camera/camera/color/camera_info
   │  ├─ /camera/camera/depth/camera_info
   │  ├─ /camera/camera/imu (raw)
   │  └─ /imu/data_filtered (Madgewick-filtered)
   │
   └─ Publishes static transforms:
      └─ /tf_static (RealSense hardware calibration)
         ├─ camera_link → camera_color_frame
         ├─ camera_color_frame → camera_color_optical_frame
         ├─ camera_color_optical_frame → camera_imu_optical_frame
         ├─ camera_link → camera_depth_frame
         └─ ... (all other sensor transforms)

2. RTAB-Map RGB-D Odometry Node
   │
   ├─ Inputs:
   │  ├─ /camera/camera/color/image_raw
   │  ├─ /camera/camera/depth/image_rect_raw
   │  ├─ /camera/camera/color/camera_info
   │  └─ /imu/data_filtered (gravity-aligned orientation hints)
   │
   └─ Outputs:
      └─ /tf (dynamic):
         └─ odom → camera_link (visual odometry, @30 Hz)

3. RTAB-Map SLAM Node
   │
   ├─ Inputs:
   │  ├─ Visual odometry from RGB-D node
   │  ├─ Loop closure detection results
   │  └─ Occupancy grid data
   │
   └─ Outputs:
      └─ /tf (dynamic):
         └─ map → odom (loop closure corrections, variable freq)

4. Final TF Tree (in RViz)
   └─ map
      └─ odom
         └─ camera_link (moves smoothly via visual odometry)
            └─ camera_color_optical_frame (renders color image here)
               └─ camera_imu_optical_frame (virtual IMU frame for reference)
```

## ROS 2 Configuration

### Key Parameters Set in rtabmap_clean_launch.sh

```bash
# Basic sensor topics
rgb_topic:=/camera/camera/color/image_raw
depth_topic:=/camera/camera/depth/image_rect_raw
camera_info_topic:=/camera/camera/color/camera_info
depth_camera_info_topic:=/camera/camera/depth/camera_info
imu_topic:=/imu/data_filtered

# CRITICAL: Frame naming
frame_id:=camera_link           # Robot base frame (was: camera_color_optical_frame)
map_frame_id:=map               # Global reference frame
odom_frame_id:=odom             # Odometry frame

# IMU integration
subscribe_imu:=true             # Use IMU data
wait_imu_to_init:=true          # Wait for gravity alignment before SLAM starts

# Simulation/playback
use_sim_time:=true              # Use bag clock instead of wall clock
wait_for_transform:=5.0         # Grace period for transforms to be available

# Synchronization
approx_sync:=true               # Approximate time sync for RGB-D
```

### RTAB-Map Algorithm Parameters

```bash
rtabmap_args:="
  --delete_db_on_start           # Fresh start (no previous map)
  -DRGBD/CreateOccupancyGrid=true # Build 3D occupancy grid
  -DGrid/3D=true                 # 3D grid (not 2D projection)
  -DGrid/FromDepth=true          # Build from depth camera
  -DGrid/Sensor=1                # RealSense sensor profile
  -DGrid/RayTracing=true         # Ray-trace for free space
  -DGrid/CellSize=0.05           # 5cm grid cells
  -DTf/Tolerance=1.0             # 1 second TF buffer tolerance
"
```

## Testing the TF Tree

### View All Frames Graphically

```bash
# In a separate terminal (while bag is playing):
ros2 run tf2_tools view_frames
# This generates ~/.ros/frames.pdf with the complete TF tree
```

### Check Specific Transform

```bash
# Look up transform from odom to camera_link
ros2 run tf2_ros tf_echo odom camera_link

# Look up transform from camera_link to camera_imu_optical_frame
ros2 run tf2_ros tf_echo camera_link camera_imu_optical_frame
```

### Monitor Frame Publication Rate

```bash
# See how often transforms are published
ros2 run tf2_tools tf_monitor

# Or subscribe to /tf topic
ros2 topic echo /tf
```

## Common Issues and Solutions

### Issue: "Could not find a transformation between camera_link and X"

**Cause:** Not waiting for static transforms to load from bag

**Solution:** The script already handles this with `wait_for_transform:=5.0`

---

### Issue: "IMU data in wrong frame"

**Cause:** IMU frame_id doesn't match what RTAB-Map expects

**Solution:** IMU must be in a frame reachable from `camera_link` via TF. Our setup:
- `camera_link` → (static) → `camera_imu_optical_frame` ✓

---

### Issue: "Visual odometry drifts badly"

**Cause:** IMU not providing gravity hints

**Solution:** Ensure `subscribe_imu:=true` and bag has valid filtered IMU data. Check:
```bash
ros2 topic echo /imu/data_filtered --once
# Should show quaternion with reasonable values (not all zeros)
```

---

### Issue: "Bag times don't match system time"

**Cause:** `use_sim_time:=false` when bag is running

**Solution:** Script sets `use_sim_time:=true` globally. Verify:
```bash
ros2 param get /use_sim_time
# Should return: Boolean value is: True
```

## References

- RealSense ROS2 Wrapper: https://github.com/IntelRealSense/realsense-ros
- RTAB-Map Documentation: http://wiki.ros.org/rtabmap_ros
- TF2 Documentation: https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html
- Madgewick IMU Filter: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
