# RTAB-Map SLAM Working Example - DO NOT CHANGE

This folder contains the minimal setup to launch RTAB-Map with RealSense bag data.

## Files

- `rtabmap_clean_launch.sh` - Main launch script with optimized parameters
- `rtabmapGUI.ini` - Optimized rtabmap_viz configuration (performance settings)
- `octomap_config.rviz` - rviz2 configuration for viewing octomap topics
- `bag/` - Symbolic link to the RealSense RGB-D bag file

## How to Launch

From this directory:

```bash
source /opt/ros/humble/setup.bash
bash rtabmap_clean_launch.sh
```

This will start:
1. ROS 2 daemon
2. Static TF transforms
3. Bag playback (looping at 0.2x speed)
4. RTAB-Map SLAM (rgbd_odometry + rtabmap)
5. rtabmap_viz (native visualizer with optimized settings)

## Performance Optimizations

The rtabmapGUI.ini includes these settings for faster rendering:
- octomap_depth: 10 (reduced from 16)
- octomap_rendering_type: 2 (points mode, faster than solid)
- octomap_point_size: 2 (smaller points)
- decimation: 8 (show fewer points)

## Visualization

### rtabmap_viz (built-in)
- Automatically starts with optimized settings
- Shows 3D map, octomap, trajectory

### rviz2 (alternative)
```bash
source /opt/ros/humble/setup.bash
rviz2 -d octomap_config.rviz
```

## Topics Generated

**Point Clouds:**
- `/rtabmap/cloud_map` - Full 3D map
- `/rtabmap/cloud_obstacles` - Obstacle points
- `/rtabmap/cloud_ground` - Ground plane points
- `/rtabmap/octomap_occupied_space` - Occupied voxels
- `/rtabmap/octomap_empty_space` - Free space voxels
- `/rtabmap/octomap_global_frontier_space` - Frontier boundaries

**Octomap:**
- `/rtabmap/octomap_binary` - Binary octomap
- `/rtabmap/octomap_full` - Full octomap structure

**Grid:**
- `/rtabmap/map` - 2D occupancy grid
- `/rtabmap/octomap_grid` - 3D octomap as 2D grid

## Shutdown

```bash
pkill -9 ros2 rtabmap rviz
```

