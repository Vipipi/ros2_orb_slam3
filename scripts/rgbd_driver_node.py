#!/usr/bin/env python3

"""
RGB-D Driver Node for ORB-SLAM3 ROS2 wrapper

This node handles RGB and depth image synchronization and sends them to the C++ ORB-SLAM3 node.

Author: Assistant
Date: 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import sys
from cv_bridge import CvBridge

class RGBDDriver(Node):
    def __init__(self, node_name="rgbd_py_node"):
        super().__init__(node_name)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Configuration parameters (defaults)
        self.settings_name = "RealSense_D435i"  # Use D435i RGB-D configuration
        self.image_seq = "sample_realsense_rgbd"  # Unused but kept for parity
        self.dataset_path = "/output_images/TEST_DATASET/sample_realsense_rgbd/"
        self.fixed_publish_rate = 30.0  # Hz
        
        # Declare ROS2 parameters (allow override via --ros-args -p ...)
        self.declare_parameter('settings_name', self.settings_name)
        self.declare_parameter('dataset_path', self.dataset_path)
        self.declare_parameter('fixed_publish_rate', float(self.fixed_publish_rate))
        self.declare_parameter('show_imgz', False)
        self.declare_parameter('sync_tolerance_sec', 0.02)  # 20 ms default tolerance
        self.declare_parameter('skip_handshake', True)
        self.declare_parameter('debug_logging', True)
        
        # Override defaults from parameters
        self.settings_name = str(self.get_parameter('settings_name').value)
        self.dataset_path = str(self.get_parameter('dataset_path').value)
        self.fixed_publish_rate = float(self.get_parameter('fixed_publish_rate').value)
        self.show_imgz = bool(self.get_parameter('show_imgz').value)
        self.sync_tolerance_sec = float(self.get_parameter('sync_tolerance_sec').value)
        self.skip_handshake = bool(self.get_parameter('skip_handshake').value)
        self.debug_logging = bool(self.get_parameter('debug_logging').value)
        
        # Topic names
        self.pub_exp_config_name = "/rgbd_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/rgbd_py_driver/exp_settings_ack"
        self.pub_rgb_img_name = "/rgbd_py_driver/rgb_img_msg"
        self.pub_depth_img_name = "/rgbd_py_driver/depth_img_msg"
        self.pub_timestep_name = "/rgbd_py_driver/timestep_msg"
        
        self.send_config = True  # Set False once handshake is completed with the cpp node
        
        # Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        
        # Build the configuration string to be sent out
        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")
        
        # Subscriber to get acknowledgement from CPP node that it received experiment settings
        self.subscribe_exp_ack_ = self.create_subscription(String, 
                                                           self.sub_exp_ack_name, 
                                                           self.ack_callback, 10)

        self.subscribe_exp_ack_
        
        # Publishers to send RGB and depth images
        self.publish_rgb_img_ = self.create_publisher(Image, self.pub_rgb_img_name, 1)
        self.publish_depth_img_ = self.create_publisher(Image, self.pub_depth_img_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_name, 1)
        
        # Initialize work variables for main logic
        self.start_frame = 0  # Default 0
        self.end_frame = -1  # Default -1
        self.frame_stop = -1  # Set -1 to use the whole sequence
        self.show_imgz = False  # Default, False, set True to see the output directly from this node
        self.frame_id = 0  # Integer id of an image frame
        self.frame_count = 0  # Ensure we are consistent with the count number of the frame
        self.inference_time = []  # List to compute average time
        
        # RGB-D specific variables
        self.rgb_images = []
        self.depth_images = []
        self.timestamps = []
        self.current_frame_idx = 0
        
        print()
        print("RGB-D Driver Node Started")
        print("=" * 50)
        print(f"Dataset path: {self.dataset_path}")
        print(f"Settings: {self.settings_name}")
        print("=" * 50)
        
        # Load dataset
        self.load_dataset()
        
        # Not using timers; publishing is driven by a rate-controlled loop in main()
        print(f"RGB-D driver ready. Target publish rate: {self.fixed_publish_rate} Hz")
    
    def load_dataset(self):
        """Load RGB and depth images from dataset with timestamp-based synchronization"""
        try:
            # Load RGB images
            rgb_path = os.path.join(self.dataset_path, "rgb")
            depth_path = os.path.join(self.dataset_path, "depth")
            
            if not os.path.exists(rgb_path) or not os.path.exists(depth_path):
                print(f"Error: RGB or depth directory not found in {self.dataset_path}")
                return
            
            # Get sorted list of RGB images with timestamps
            rgb_files = sorted([f for f in os.listdir(rgb_path) if f.endswith(('.png', '.jpg', '.jpeg'))])
            depth_files = sorted([f for f in os.listdir(depth_path) if f.endswith(('.png', '.jpg', '.jpeg'))])
            
            print(f"Found {len(rgb_files)} RGB images and {len(depth_files)} depth images")
            
            # Parse timestamps
            def parse_ts(name: str) -> float:
                return float(os.path.splitext(name)[0])
            rgb_ts = [(parse_ts(f), f) for f in rgb_files]
            depth_ts = [(parse_ts(f), f) for f in depth_files]
            rgb_ts.sort(key=lambda x: x[0])
            depth_ts.sort(key=lambda x: x[0])
            
            # Approximate matching within tolerance
            i = j = 0
            matched = 0
            while i < len(rgb_ts) and j < len(depth_ts):
                tr, fr = rgb_ts[i]
                td, fd = depth_ts[j]
                diff = tr - td
                if abs(diff) <= self.sync_tolerance_sec:
                    # Match
                    rgb_img = cv2.imread(os.path.join(rgb_path, fr))
                    depth_img = cv2.imread(os.path.join(depth_path, fd), cv2.IMREAD_ANYDEPTH)
                    if rgb_img is not None and depth_img is not None:
                        self.rgb_images.append(rgb_img)
                        self.depth_images.append(depth_img)
                        # Use average timestamp
                        self.timestamps.append(0.5 * (tr + td))
                        matched += 1
                    i += 1
                    j += 1
                elif diff < 0:
                    i += 1
                else:
                    j += 1
            
            print(f"Successfully matched {matched} RGB-D pairs within ±{self.sync_tolerance_sec*1000:.0f} ms tolerance")
            if matched == 0:
                print("Warning: No synchronized pairs found. Consider increasing sync_tolerance_sec")
            
        except Exception as e:
            print(f"Error loading dataset: {e}")
            import traceback
            traceback.print_exc()
    
    def ack_callback(self, msg):
        """Callback for acknowledgement from C++ node"""
        if msg.data == "ACK":
            print("Received ACK from C++ node")
            self.send_config = False
            print("Handshake completed! Starting to send images...")
    
    def publish_next_frame(self):
        """Publish the next RGB-D frame and associated timestamp."""
        if self.current_frame_idx >= len(self.rgb_images):
            return False
        
        rgb_img = self.rgb_images[self.current_frame_idx]
        depth_img = self.depth_images[self.current_frame_idx]
        timestamp = self.timestamps[self.current_frame_idx]
        
        try:
            if self.debug_logging:
                print(f"[DEBUG] Preparing frame {self.current_frame_idx+1}/{len(self.rgb_images)} | rgb: {None if rgb_img is None else rgb_img.shape} | depth: {None if depth_img is None else depth_img.shape} | ts: {timestamp:.6f}", flush=True)

            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            
            # Depth: convert 16UC1 (mm) -> 32FC1 (meters)
            if depth_img.dtype == np.uint16:
                depth_img_float = depth_img.astype(np.float32) / 1000.0
                depth_msg = self.bridge.cv2_to_imgmsg(depth_img_float, "32FC1")
            else:
                depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "32FC1")
            
            now = self.get_clock().now()
            rgb_msg.header.stamp = now.to_msg()
            depth_msg.header.stamp = now.to_msg()
            rgb_msg.header.frame_id = "camera_color_optical_frame"
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            
            self.publish_rgb_img_.publish(rgb_msg)
            self.publish_depth_img_.publish(depth_msg)
            
            ts_msg = Float64()
            ts_msg.data = float(timestamp)
            self.publish_timestep_msg_.publish(ts_msg)
            
            if self.debug_logging:
                print(f"[DEBUG] Published frame {self.current_frame_idx+1}/{len(self.rgb_images)} at {self.fixed_publish_rate}Hz | stamp: {now.nanoseconds/1e9:.6f}", flush=True)

            self.current_frame_idx += 1
            return True
        except Exception as e:
            print(f"Error publishing frame {self.current_frame_idx}: {e}")
            import traceback
            traceback.print_exc()
            return False

def main(args=None):
    rclpy.init(args=args)
    node = RGBDDriver()

    if len(node.rgb_images) == 0 or len(node.depth_images) == 0:
        print("❌ No images loaded. Check dataset_path and folder structure (rgb/ and depth/).")
        node.destroy_node()
        rclpy.shutdown()
        return

    if len(node.timestamps) == 0:
        print("❌ No synchronized pairs found. Consider increasing -p sync_tolerance_sec (e.g., 0.03 or 0.05).")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Handshake loop (optional)
    if not node.skip_handshake:
        print("Waiting for C++ node ACK (set -p skip_handshake:=true to skip)...")
        rate_handshake = node.create_rate(20)
        while node.send_config:
            msg = String()
            msg.data = node.exp_config_msg
            node.publish_exp_config_.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            rate_handshake.sleep()
        print("Handshake complete")
    else:
        # Send config once (like mono) and start
        msg = String()
        msg.data = node.exp_config_msg
        node.publish_exp_config_.publish(msg)
        print("Skipping handshake: sent single config message and starting stream...")

    # Streaming loop at fixed rate
    rate_stream = node.create_rate(node.fixed_publish_rate)
    tick = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.debug_logging and (tick % 10 == 0):
                print(f"[DEBUG] Loop tick={tick}, idx={node.current_frame_idx}/{len(node.rgb_images)}", flush=True)
            if not node.publish_next_frame():
                print("Dataset finished", flush=True)
                break
            rate_stream.sleep()
            tick += 1
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 