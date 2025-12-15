#!/bin/bash
set -e

echo ""
echo "======================================="
echo "RTAB-Map SLAM - Clean Launch"
echo "Configuration: Map STATIONARY, Odom MOVING"
echo "======================================="
echo ""

# Ensure we're in the correct directory
cd /home/magnus/realsense
BAG_PATH="./d455_slam_30sec_madgewick_60hz_color_30hz_depth"

# STEP 1: Verify bag file exists
echo "[1/4] Verifying bag file exists..."
if [ ! -d "$BAG_PATH" ]; then
    echo "ERROR: Bag file not found at $BAG_PATH"
    exit 1
fi
echo "✓ Bag file verified"
echo ""

# STEP 2: Publish static TF transforms
echo "[2/4] Publishing static TF transforms..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id camera_link > /dev/null 2>&1 &
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x 0 --y -0.059 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id camera_link --child-frame-id camera_color_frame > /dev/null 2>&1 &
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0.7071 --qy 0 --qz 0.7071 --qw 0 --frame-id camera_color_frame --child-frame-id camera_color_optical_frame > /dev/null 2>&1 &
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x -0.01602 --y -0.03022 --z 0.0074 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id camera_link --child-frame-id camera_gyro_frame > /dev/null 2>&1 &
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id camera_gyro_frame --child-frame-id camera_imu_frame > /dev/null 2>&1 &
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id camera_imu_frame --child-frame-id camera_imu_optical_frame > /dev/null 2>&1 &
sleep 2
echo "✓ 6 static TF transforms published"
echo ""

# STEP 3: Start bag playback at 0.2x speed
echo "[3/4] Starting bag playback at 0.2x speed..."
ros2 bag play "$BAG_PATH" --loop --rate 0.2 > /dev/null 2>&1 &
BAG_PID=$!
sleep 3

# Verify bag is publishing topics
echo "Verifying bag is publishing topics..."
for attempt in {1..5}; do
    if ros2 topic list | grep -q "camera/camera/color/image_raw"; then
        echo "✓ Bag is publishing data (attempt $attempt/5)"
        break
    else
        if [ $attempt -eq 5 ]; then
            echo "ERROR: Bag did not start publishing topics after 5 attempts"
            kill $BAG_PID 2>/dev/null || true
            exit 1
        fi
        echo "  Waiting for bag to publish... (attempt $attempt/5)"
        sleep 2
    fi
done
echo ""

# STEP 4: Launch RTAB-Map
echo "[4/4] Launching RTAB-Map with map-static configuration..."
echo ""
echo "Frame Configuration:"
echo "  • map_frame_id:=map           → Global reference frame (STATIONARY)"
echo "  • odom_frame_id:=odom         → Local odometry frame (MOVES)"
echo "  • frame_id:=camera_*          → Camera sensor frame"
echo ""
echo "This creates the transform hierarchy:"
echo "  map (fixed) ← [RTAB-Map updates] ← odom ← [odometry] ← camera"
echo ""
echo "Result: The 3D global map in RViz will appear FIXED, while the"
echo "camera position moves relative to it as odometry accumulates."
echo ""

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start -DRGBD/CreateOccupancyGrid=true -DGrid/3D=true -DGrid/FromDepth=true -DGrid/Sensor=1 -DGrid/RayTracing=true -DGrid/CellSize=0.05" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  depth_camera_info_topic:=/camera/camera/depth/camera_info \
  imu_topic:=/imu/data_filtered \
  frame_id:=camera_color_optical_frame \
  map_frame_id:=map \
  odom_frame_id:=odom \
  approx_sync:=true \
  stereo:=false \
  subscribe_imu:=true
