#!/usr/bin/env python3

"""
Test script for RGB-D mode

This script creates synthetic RGB-D data to test the RGB-D implementation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
from cv_bridge import CvBridge

class RGBDTestNode(Node):
    def __init__(self):
        super().__init__('rgbd_test_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create publishers
        self.rgb_pub = self.create_publisher(Image, '/rgbd_py_driver/rgb_img_msg', 10)
        self.depth_pub = self.create_publisher(Image, '/rgbd_py_driver/depth_img_msg', 10)
        self.timestamp_pub = self.create_publisher(Float64, '/rgbd_py_driver/timestep_msg', 10)
        
        # Create timer for publishing test data
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10 FPS
        
        # Test data counter
        self.frame_count = 0
        
        print("RGB-D Test Node Started")
        print("Publishing synthetic RGB-D data...")
    
    def create_synthetic_rgb_image(self, frame_num):
        """Create a synthetic RGB image"""
        # Create a simple pattern that changes over time
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add some moving shapes
        center_x = 320 + int(50 * np.sin(frame_num * 0.1))
        center_y = 240 + int(30 * np.cos(frame_num * 0.15))
        
        # Draw a circle
        cv2.circle(img, (center_x, center_y), 50, (0, 255, 0), -1)
        
        # Draw a rectangle
        rect_x = 100 + int(20 * np.sin(frame_num * 0.2))
        cv2.rectangle(img, (rect_x, 100), (rect_x + 80, 180), (255, 0, 0), -1)
        
        # Add text
        cv2.putText(img, f'Frame {frame_num}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return img
    
    def create_synthetic_depth_image(self, frame_num):
        """Create a synthetic depth image"""
        # Create depth image with some 3D structure
        depth = np.zeros((480, 640), dtype=np.float32)
        
        # Add a plane at different depths
        for y in range(480):
            for x in range(640):
                # Create a simple depth pattern
                depth[y, x] = 1000.0 + 500.0 * np.sin(x * 0.01) + 300.0 * np.cos(y * 0.01)
                
                # Add some variation based on frame number
                depth[y, x] += 100.0 * np.sin(frame_num * 0.1)
        
        # Ensure depth is positive
        depth = np.maximum(depth, 100.0)
        
        return depth
    
    def publish_test_data(self):
        """Publish synthetic RGB-D data"""
        try:
            # Create synthetic images
            rgb_img = self.create_synthetic_rgb_image(self.frame_count)
            depth_img = self.create_synthetic_depth_image(self.frame_count)
            
            # Convert to ROS messages
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "32FC1")
            
            # Set timestamps
            current_time = self.get_clock().now()
            rgb_msg.header.stamp = current_time.to_msg()
            depth_msg.header.stamp = current_time.to_msg()
            
            # Publish messages
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
            
            # Publish timestamp
            timestamp_msg = Float64()
            timestamp_msg.data = time.time()
            self.timestamp_pub.publish(timestamp_msg)
            
            print(f"Published test frame {self.frame_count}")
            
            # Show images for debugging
            cv2.imshow("Test RGB", rgb_img)
            cv2.imshow("Test Depth", depth_img / 2000.0)  # Normalize for display
            cv2.waitKey(1)
            
            self.frame_count += 1
            
            # Stop after 100 frames
            if self.frame_count >= 100:
                print("Test completed - 100 frames published")
                self.destroy_node()
                rclpy.shutdown()
                
        except Exception as e:
            print(f"Error publishing test data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBDTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 