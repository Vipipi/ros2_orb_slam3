# Quick Start Guide: RGB-D Mode

This guide will help you get the RGB-D mode running quickly.

## Prerequisites

- ROS2 Humble installed
- OpenCV, Eigen3, Pangolin installed
- ORB-SLAM3 dependencies installed

## Step 1: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_orb_slam3
source install/setup.bash
```

## Step 2: Test with Synthetic Data

1. **Start the C++ RGB-D node**:
   ```bash
   ros2 run ros2_orb_slam3 rgbd_node_cpp
   ```

2. **In another terminal, start the test node**:
   ```bash
   ros2 run ros2_orb_slam3 test_rgbd.py
   ```

3. **Verify it's working**:
   - You should see Pangolin window with ORB-SLAM3 visualization
   - Check terminal output for tracking messages
   - Look for "Published test frame X" messages

## Step 3: Use with Real Dataset

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

2. **Update dataset path** in `scripts/rgbd_driver_node.py`:
   ```python
   self.dataset_path = os.path.expanduser("~/path/to/your/dataset/")
   ```

3. **Launch the system**:
   ```bash
   ros2 launch ros2_orb_slam3 rgbd_launch.py
   ```

## Step 4: Use with Real Camera

1. **Install camera drivers** (e.g., RealSense):
   ```bash
   sudo apt install ros-humble-realsense2-camera
   ```

2. **Launch camera**:
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

3. **Modify the Python driver** to subscribe to camera topics:
   ```python
   # In rgbd_driver_node.py, replace dataset loading with:
   self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
   self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
   ```

4. **Launch RGB-D system**:
   ```bash
   ros2 launch ros2_orb_slam3 rgbd_launch.py
   ```

## Troubleshooting

### Common Issues

1. **"No device connected"**:
   - Check camera connection
   - Verify camera drivers are installed

2. **"Failed to open settings file"**:
   - Check configuration file path
   - Verify file permissions

3. **Poor tracking**:
   - Adjust `RGBD.DepthMapFactor` in config file
   - Check camera calibration
   - Ensure good lighting

### Debug Commands

```bash
# Check topics
ros2 topic list

# Monitor RGB images
ros2 topic echo /rgbd_py_driver/rgb_img_msg

# Monitor depth images
ros2 topic echo /rgbd_py_driver/depth_img_msg

# Check node status
ros2 node list
ros2 node info /rgbd_cpp_node
```

## Configuration

Edit `orb_slam3/config/RGBD/TUM1.yaml` for your camera:

```yaml
# Camera calibration
Camera1.fx: 517.306408
Camera1.fy: 516.469215
Camera1.cx: 318.643040
Camera1.cy: 255.313989

# RGB-D parameters
RGBD.DepthMapFactor: 5000.0  # Adjust for your depth camera
Stereo.ThDepth: 40.0         # Depth threshold
```

## Expected Output

- **Pangolin window**: Shows 3D map and camera trajectory
- **Terminal output**: Tracking status and frame processing info
- **Topics**: RGB and depth images being published and processed

## Next Steps

1. **Tune parameters** for your specific setup
2. **Add IMU support** if available
3. **Integrate with your robot** for autonomous navigation
4. **Save trajectories** for analysis

For detailed documentation, see `RGBD_README.md`. 