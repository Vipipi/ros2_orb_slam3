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
        
        # Configuration parameters
        self.settings_name = "RealSense_D435i"  # Use D435i RGB-D configuration
        self.image_seq = "sample_realsense_rgbd"  # Change this to match your dataset path
        
        # Path to your dataset
        self.dataset_path = "/output_images/TEST_DATASET/sample_realsense_rgbd/"
        
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
        
        # Start the main loop
        self.timer = self.create_timer(0.033, self.main_loop)  # ~30 FPS
        
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
            
            # Create a mapping of timestamps to file pairs
            rgb_timestamps = {}
            depth_timestamps = {}
            
            # Extract timestamps from RGB filenames
            for rgb_file in rgb_files:
                try:
                    # Extract timestamp from filename (assuming format: timestamp.png)
                    timestamp_str = rgb_file.split('.')[0]  # Remove extension
                    timestamp = float(timestamp_str)
                    rgb_timestamps[timestamp] = rgb_file
                except (ValueError, IndexError):
                    print(f"Warning: Could not parse timestamp from RGB file: {rgb_file}")
                    continue
            
            # Extract timestamps from depth filenames
            for depth_file in depth_files:
                try:
                    # Extract timestamp from filename (assuming format: timestamp.png)
                    timestamp_str = depth_file.split('.')[0]  # Remove extension
                    timestamp = float(timestamp_str)
                    depth_timestamps[timestamp] = depth_file
                except (ValueError, IndexError):
                    print(f"Warning: Could not parse timestamp from depth file: {depth_file}")
                    continue
            
            # Find common timestamps (synchronized pairs)
            common_timestamps = sorted(set(rgb_timestamps.keys()) & set(depth_timestamps.keys()))
            
            if not common_timestamps:
                print("Error: No synchronized RGB-D pairs found!")
                return
            
            print(f"Found {len(common_timestamps)} synchronized RGB-D pairs")
            
            # Load synchronized pairs
            for timestamp in common_timestamps:
                rgb_file = rgb_timestamps[timestamp]
                depth_file = depth_timestamps[timestamp]
                
                rgb_img = cv2.imread(os.path.join(rgb_path, rgb_file))
                depth_img = cv2.imread(os.path.join(depth_path, depth_file), cv2.IMREAD_ANYDEPTH)
                
                if rgb_img is not None and depth_img is not None:
                    self.rgb_images.append(rgb_img)
                    self.depth_images.append(depth_img)
                    self.timestamps.append(timestamp)  # Use actual timestamp
                    
            print(f"Successfully loaded {len(self.rgb_images)} synchronized RGB-D pairs")
            
            # Print timestamp range for debugging
            if self.timestamps:
                print(f"Timestamp range: {min(self.timestamps):.6f} to {max(self.timestamps):.6f}")
                print(f"Average frame interval: {(max(self.timestamps) - min(self.timestamps)) / (len(self.timestamps) - 1):.6f}s")
            
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
    
    def main_loop(self):
        """Main loop to send RGB-D images"""
        if self.send_config:
            # Send configuration
            config_msg = String()
            config_msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(config_msg)
            print(f"Sent config: {config_msg.data}")
            return
        
        if self.current_frame_idx >= len(self.rgb_images):
            print("Dataset finished")
            return
        
        # Get current frame
        rgb_img = self.rgb_images[self.current_frame_idx]
        depth_img = self.depth_images[self.current_frame_idx]
        timestamp = self.timestamps[self.current_frame_idx]
        
        # Convert images to ROS messages
        try:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "32FC1")
            
            # Note: ORB-SLAM3 automatically applies DepthMapFactor (1/1000.0) 
            # from the configuration file to convert mm to meters
            # No additional conversion needed here
            
            # Set synchronized timestamps for D435i compatibility
            current_time = self.get_clock().now()
            rgb_msg.header.stamp = current_time.to_msg()
            depth_msg.header.stamp = current_time.to_msg()
            
            # Set frame IDs for proper TF tree
            rgb_msg.header.frame_id = "camera_color_optical_frame"
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            
            # Publish images with minimal delay between them
            self.publish_rgb_img_.publish(rgb_msg)
            self.publish_depth_img_.publish(depth_msg)
            
            # Publish timestamp
            timestamp_msg = Float64()
            timestamp_msg.data = timestamp
            self.publish_timestep_msg_.publish(timestamp_msg)
            
            print(f"Published synchronized frame {self.current_frame_idx + 1}/{len(self.rgb_images)}")
            
            # Show images if enabled
            if self.show_imgz:
                cv2.imshow("RGB Image", rgb_img)
                cv2.imshow("Depth Image", depth_img)
                cv2.waitKey(1)
            
            self.current_frame_idx += 1
            
        except Exception as e:
            print(f"Error publishing frame {self.current_frame_idx}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBDDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 