# RGB-D Mode for ORB-SLAM3 ROS2 Wrapper

This document provides instructions on how to use the newly added RGB-D mode in the ORB-SLAM3 ROS2 wrapper.

## Overview

The RGB-D mode allows you to use ORB-SLAM3 with both RGB and depth images simultaneously, providing better accuracy and robustness compared to monocular mode.

## Features

- **Synchronized RGB-D Processing**: Handles both RGB and depth images with proper synchronization
- **Multiple Sensor Support**: Supports both pure RGB-D and RGB-D with IMU fusion
- **Configurable Parameters**: Easy configuration through YAML files
- **Real-time Processing**: Designed for real-time SLAM applications

## File Structure

```
ros2_orb_slam3/
├── include/ros2_orb_slam3/
│   ├── common.hpp          # Original monocular mode
│   └── rgbd_common.hpp     # New RGB-D mode header
├── src/
│   ├── common.cpp          # Original monocular implementation
│   ├── rgbd_common.cpp     # New RGB-D implementation
│   ├── mono_example.cpp    # Monocular executable
│   └── rgbd_example.cpp    # RGB-D executable
├── scripts/
│   ├── mono_driver_node.py     # Original monocular driver
│   └── rgbd_driver_node.py     # New RGB-D driver
├── orb_slam3/config/
│   ├── Monocular/          # Monocular configurations
│   ├── Stereo/             # Stereo configurations
│   └── RGBD/               # New RGB-D configurations
│       └── TUM1.yaml       # Example RGB-D config
├── launch/
│   └── rgbd_launch.py      # RGB-D launch file
└── TEST_DATASET/
    └── sample_tum_rgbd/    # Example dataset structure
        ├── rgb/             # RGB images
        └── depth/           # Depth images
```

## Installation

1. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_orb_slam3
   source install/setup.bash
   ```

2. **Verify installation**:
   ```bash
   ros2 pkg list | grep ros2_orb_slam3
   ```

## Usage

### Method 1: Using Launch File (Recommended)

1. **Prepare your dataset**:
   ```
   TEST_DATASET/sample_tum_rgbd/
   ├── rgb/
   │   ├── 000001.png
   │   ├── 000002.png
   │   └── ...
   └── depth/
       ├── 000001.png
       ├── 000002.png
       └── ...
   ```

2. **Update the dataset path** in `scripts/rgbd_driver_node.py`:
   ```python
   self.dataset_path = os.path.expanduser("~/path/to/your/dataset/")
   ```

3. **Launch the RGB-D system**:
   ```bash
   ros2 launch ros2_orb_slam3 rgbd_launch.py
   ```

### Method 2: Manual Node Execution

1. **Start the C++ ORB-SLAM3 RGB-D node**:
   ```bash
   ros2 run ros2_orb_slam3 rgbd_node_cpp
   ```

2. **Start the Python RGB-D driver node**:
   ```bash
   ros2 run ros2_orb_slam3 rgbd_driver_node.py
   ```

### Method 3: Using Real-time Camera

To use with a real RGB-D camera (like RealSense or Kinect):

1. **Modify the Python driver** to subscribe to camera topics:
   ```python
   # In rgbd_driver_node.py, replace the dataset loading with:
   self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
   self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
   ```

2. **Launch your camera driver** (e.g., RealSense):
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

3. **Launch the RGB-D system**:
   ```bash
   ros2 launch ros2_orb_slam3 rgbd_launch.py
   ```

## Configuration

### RGB-D Configuration File

The RGB-D mode uses configuration files in `orb_slam3/config/RGBD/`. Example `TUM1.yaml`:

```yaml
%YAML:1.0

# Camera Parameters
File.version: "1.0"
Camera.type: "PinHole"

# Camera calibration
Camera1.fx: 517.306408
Camera1.fy: 516.469215
Camera1.cx: 318.643040
Camera1.cy: 255.313989

# Distortion parameters
Camera1.k1: 0.262383
Camera1.k2: -0.953104
Camera1.p1: -0.005358
Camera1.p2: 0.002628
Camera1.k3: 1.163314

# Camera resolution
Camera.width: 640
Camera.height: 480
Camera.fps: 30
Camera.RGB: 1

# RGB-D specific parameters
RGBD.DepthMapFactor: 5000.0  # Depth scaling factor
Stereo.ThDepth: 40.0         # Depth threshold

# ORB Parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

### Important Parameters

- **`RGBD.DepthMapFactor`**: Scaling factor for depth values. For TUM datasets, typically 5000.0
- **`Stereo.ThDepth`**: Maximum depth threshold for feature matching
- **`Camera.RGB`**: Set to 1 for RGB color order, 0 for BGR

## Dataset Format

### TUM RGB-D Dataset Format

For TUM RGB-D datasets, organize your data as follows:

```
dataset/
├── rgb/
│   ├── 1305031102.175304.png
│   ├── 1305031102.211214.png
│   └── ...
├── depth/
│   ├── 1305031102.160407.png
│   ├── 1305031102.226738.png
│   └── ...
└── groundtruth.txt
```

### Custom Dataset Format

For custom datasets, ensure:

1. **Synchronized images**: RGB and depth images should be timestamp-synchronized
2. **Consistent naming**: RGB and depth images should have corresponding names
3. **Proper depth format**: Depth images should be 16-bit or 32-bit float

## Troubleshooting

### Common Issues

1. **"No device connected" error**:
   - Ensure your RGB-D camera is properly connected
   - Check camera drivers are installed

2. **"Failed to open settings file"**:
   - Verify the configuration file path is correct
   - Check file permissions

3. **"Error reading depth image"**:
   - Ensure depth images are in the correct format (32FC1)
   - Check depth image encoding

4. **Poor tracking performance**:
   - Adjust `RGBD.DepthMapFactor` for your dataset
   - Check camera calibration parameters
   - Ensure good lighting conditions

### Debugging

1. **Enable debug output**:
   ```bash
   ros2 run ros2_orb_slam3 rgbd_node_cpp --ros-args --log-level debug
   ```

2. **Check topic communication**:
   ```bash
   ros2 topic list
   ros2 topic echo /rgbd_py_driver/rgb_img_msg
   ros2 topic echo /rgbd_py_driver/depth_img_msg
   ```

3. **Monitor system performance**:
   ```bash
   ros2 topic hz /rgbd_py_driver/rgb_img_msg
   ```

## Performance Tips

1. **Optimize image resolution**: Lower resolutions run faster but may reduce accuracy
2. **Adjust ORB parameters**: More features provide better tracking but slower performance
3. **Use appropriate depth threshold**: Balance between accuracy and computational cost
4. **Ensure good synchronization**: Poor sync between RGB and depth can degrade performance

## Comparison with Monocular Mode

| Feature | Monocular | RGB-D |
|---------|-----------|-------|
| Scale estimation | No (scale drift) | Yes (metric scale) |
| Initialization | Requires motion | Instant |
| Accuracy | Lower | Higher |
| Computational cost | Lower | Higher |
| Robustness | Lower | Higher |

## Advanced Usage

### IMU Integration

To use RGB-D with IMU data, modify the sensor type in `rgbd_common.cpp`:

```cpp
sensorType = ORB_SLAM3::System::IMU_RGBD;
```

And add IMU configuration to your YAML file:

```yaml
# IMU parameters
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]

IMU.NoiseGyro: 1e-3
IMU.NoiseAcc: 1e-2
IMU.GyroWalk: 1e-6
IMU.AccWalk: 1e-4
IMU.Frequency: 200.0
```

### Custom Topics

To use custom topic names, modify the topic variables in `rgbd_common.cpp`:

```cpp
subRGBImgMsgName = "/your/custom/rgb/topic";
subDepthImgMsgName = "/your/custom/depth/topic";
```

## Contributing

To extend the RGB-D functionality:

1. **Add new sensor types**: Modify `System.h` and add corresponding processing methods
2. **Support new datasets**: Create new configuration files and update the Python driver
3. **Improve synchronization**: Enhance the image synchronization algorithm
4. **Add visualization**: Implement additional visualization tools for RGB-D data

## License

This RGB-D mode follows the same license as the original ORB-SLAM3 project. 