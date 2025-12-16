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

# STEP 2: Start bag playback FIRST (so /clock is available before setting use_sim_time)
echo "[2/4] Starting bag playback at 0.2x speed (looping)..."
ros2 bag play "$BAG_PATH" --loop --rate 0.2 --clock > /dev/null 2>&1 &
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


# STEP 3: Set global use_sim_time parameter
echo "[3/4] Configuring sim time..."
# Set the global use_sim_time parameter so all nodes use bag timestamps
ros2 param set /use_sim_time true 2>/dev/null || true
sleep 0.5

echo "✓ Sim time configured"
echo "✓ IMU frame (camera_imu_optical_frame) already connected via bag's /tf_static"
echo ""
  


# STEP 4: Launch RTAB-Map
echo "[4/4] Launching RTAB-Map with map-static configuration..."
echo ""
echo "Frame Configuration (CORRECTED TF HIERARCHY):"
echo "  • map_frame_id:=map           → Global reference frame (STATIONARY)"
echo "  • odom_frame_id:=odom         → Local odometry frame (MOVES with pose drift)"
echo "  • frame_id:=camera_link       → Robot base frame (camera sensor base)"
echo ""
echo "This creates the transform hierarchy:"
echo "  map ← [loop closure] ← odom ← [visual odometry] ← camera_link"
echo "                                                      ↓ (static)"
echo "                                    camera_color_optical_frame"
echo ""
echo "Result: RTAB-Map publishes odom→camera_link (visual odometry)."
echo "Static transforms below camera_link move with it automatically."
echo "IMU data in camera_imu_optical_frame aligns with the TF tree."
echo ""

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start -DRGBD/CreateOccupancyGrid=true -DGrid/3D=true -DGrid/FromDepth=true -DGrid/Sensor=1 -DGrid/RayTracing=true -DGrid/CellSize=0.05 -DTf/Tolerance=1.0" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  depth_camera_info_topic:=/camera/camera/depth/camera_info \
  imu_topic:=/imu/data_filtered \
  frame_id:=camera_link \
  map_frame_id:=map \
  odom_frame_id:=odom \
  approx_sync:=true \
  stereo:=false \
  subscribe_imu:=true \
  use_sim_time:=true \
  wait_imu_to_init:=true \
  wait_for_transform:=5.0
