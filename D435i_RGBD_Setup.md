# RealSense D435i RGB-D Setup Guide for ORB-SLAM3

## Overview

This guide explains how to set up and use the Intel RealSense D435i camera with ORB-SLAM3 in RGB-D mode.

## Prerequisites

1. **RealSense D435i Camera** connected to your system
2. **RealSense ROS2 Driver** installed and running
3. **ORB-SLAM3 ROS2 Wrapper** built and ready
4. **Message Filters** for synchronized RGB-D processing

## Installation Steps

### 1. Install RealSense ROS2 Driver

```bash
# Install RealSense ROS2 driver
sudo apt-get install ros-humble-realsense2-camera ros-humble-realsense2-description

# Or build from source
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros2-development
colcon build
source install/setup.bash
```

### 2. Verify D435i Connection

```bash
# Check if D435i is detected
rs-enumerate-devices

# Should show D435i with firmware version
```

### 3. Launch RealSense Driver

```bash
# Launch D435i with RGB and depth streams
ros2 launch realsense2_camera rs_launch.py

# Or with custom parameters
ros2 launch realsense2_camera rs_launch.py \
  depth_width:=640 \
  depth_height:=480 \
  color_width:=640 \
  color_height:=480 \
  depth_fps:=30 \
  color_fps:=30
```

## Configuration Files

### 1. D435i RGB-D Configuration

The configuration file `orb_slam3/config/RGBD/RealSense_D435i.yaml` has been created with:

- **Camera Parameters**: D435i intrinsic calibration
- **RGB-D Parameters**: Depth scaling and thresholds
- **ORB Parameters**: Optimized for D435i image quality

### 2. Key Parameters Explained

```yaml
# Depth scaling factor for D435i (depth in mm)
RGBD.DepthMapFactor: 1000.0

# Depth threshold for feature matching
Stereo.ThDepth: 40.0

# ORB features (D435i has good quality)
ORBextractor.nFeatures: 1250
```

## Usage Instructions

### 1. Start RealSense Driver

```bash
# Terminal 1: Launch D435i
ros2 launch realsense2_camera rs_launch.py
```

### 2. Start ORB-SLAM3 RGB-D Node

```bash
# Terminal 2: Start ORB-SLAM3 RGB-D node
ros2 run ros2_orb_slam3 rgbd_node_cpp
```

### 3. Verify Topics

```bash
# Check if topics are published
ros2 topic list | grep camera
ros2 topic echo /camera/color/image_raw --once
ros2 topic echo /camera/depth/image_raw --once
```

## Topic Mapping

The current setup expects these topics:

| Expected Topic | RealSense Topic | Description |
|----------------|-----------------|-------------|
| `/rgbd_py_driver/rgb_img_msg` | `/camera/color/image_raw` | RGB images |
| `/rgbd_py_driver/depth_img_msg` | `/camera/depth/image_raw` | Depth images |
| `/rgbd_py_driver/timestep_msg` | N/A | Timestamp data |

## Custom Topic Mapping

If you want to use different topic names, modify the RGB-D node:

```cpp
// In src/rgbd_common.cpp
subRGBImgMsgName = "/camera/color/image_raw";  // Your RGB topic
subDepthImgMsgName = "/camera/depth/image_raw"; // Your depth topic
```

## Calibration

### 1. Camera Calibration

For best results, calibrate your specific D435i:

```bash
# Install calibration tools
sudo apt-get install ros-humble-camera-calibration

# Calibrate RGB camera
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --camera_name camera_color \
  --image_topic /camera/color/image_raw
```

### 2. Update Configuration

After calibration, update `RealSense_D435i.yaml` with your camera parameters.

## Troubleshooting

### 1. No Depth Data

```bash
# Check depth stream
ros2 topic echo /camera/depth/image_raw --once

# Verify depth format
ros2 topic info /camera/depth/image_raw
```

### 2. Synchronization Issues

```bash
# Check message timing
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw

# Verify message filters are working
ros2 topic echo /rgbd_py_driver/rgb_img_msg --once
```

### 3. Poor Tracking

- **Increase features**: Set `ORBextractor.nFeatures: 1500`
- **Adjust depth threshold**: Modify `Stereo.ThDepth`
- **Check lighting**: Ensure good illumination
- **Verify calibration**: Recalibrate if needed

## Performance Optimization

### 1. Frame Rate

D435i supports multiple frame rates:
- 30 FPS (recommended for ORB-SLAM3)
- 60 FPS (higher computational load)
- 15 FPS (lower accuracy)

### 2. Resolution

D435i supports multiple resolutions:
- 640x480 (recommended, good balance)
- 848x480 (wider field of view)
- 1280x720 (higher detail, slower processing)

### 3. Depth Quality

D435i depth quality settings:
- **Visual Preset**: High Accuracy
- **Laser Power**: 360 (default)
- **Depth Units**: 1mm

## Advanced Configuration

### 1. Custom Launch File

Create `d435i_rgbd.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # RealSense D435i
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'depth_width': 640,
                'depth_height': 480,
                'color_width': 640,
                'color_height': 480,
                'depth_fps': 30,
                'color_fps': 30,
                'enable_sync': True,
                'depth_qos': 'SENSOR_DATA',
                'color_qos': 'SENSOR_DATA'
            }]
        ),
        
        # ORB-SLAM3 RGB-D Node
        Node(
            package='ros2_orb_slam3',
            executable='rgbd_node_cpp',
            name='rgbd_node_cpp',
            parameters=[{
                'node_name_arg': 'd435i_rgbd',
                'voc_file_arg': '~/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin',
                'settings_file_path_arg': '~/ros2_orb_slam3/orb_slam3/config/RGBD/'
            }]
        )
    ])
```

### 2. TF Configuration

D435i provides proper TF frames:
- `camera_color_optical_frame`: RGB camera frame
- `camera_depth_optical_frame`: Depth camera frame
- `camera_link`: Base camera frame

## Testing

### 1. Quick Test

```bash
# Terminal 1: Launch D435i
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: Start ORB-SLAM3
ros2 run ros2_orb_slam3 rgbd_node_cpp

# Terminal 3: Monitor topics
ros2 topic echo /rgbd_py_driver/rgb_img_msg --once
```

### 2. Visualization

```bash
# View camera streams
ros2 run rqt_image_view rqt_image_view

# View TF tree
ros2 run tf2_tools view_frames
```

## Expected Results

With proper setup, you should see:

1. **ORB-SLAM3 window** showing tracking visualization
2. **Console output** with pose information
3. **Synchronized RGB-D processing** without timing issues
4. **Stable tracking** with metric scale

## Next Steps

1. **Test with your environment**
2. **Adjust parameters** based on your setup
3. **Calibrate camera** for best results
4. **Optimize performance** for your use case

## Support

For issues:
1. Check RealSense driver logs
2. Verify topic connectivity
3. Test with simple RGB-D datasets first
4. Ensure proper camera calibration 