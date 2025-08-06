#!/usr/bin/env python3
"""
Test script to verify dataset loading for RGB-D driver
"""

import os
import cv2
import numpy as np

def test_dataset_loading():
    """Test dataset loading with the actual dataset path"""
    
    dataset_path = "/output_images/TEST_DATASET/sample_realsense_rgbd/"
    rgb_path = os.path.join(dataset_path, "rgb")
    depth_path = os.path.join(dataset_path, "depth")
    
    print(f"Testing dataset loading...")
    print(f"Dataset path: {dataset_path}")
    print(f"RGB path: {rgb_path}")
    print(f"Depth path: {depth_path}")
    
    # Check if directories exist
    if not os.path.exists(rgb_path):
        print(f"❌ RGB directory not found: {rgb_path}")
        return False
    
    if not os.path.exists(depth_path):
        print(f"❌ Depth directory not found: {depth_path}")
        return False
    
    print("✅ Directories found")
    
    # Get file lists
    rgb_files = sorted([f for f in os.listdir(rgb_path) if f.endswith(('.png', '.jpg', '.jpeg'))])
    depth_files = sorted([f for f in os.listdir(depth_path) if f.endswith(('.png', '.jpg', '.jpeg'))])
    
    print(f"Found {len(rgb_files)} RGB images and {len(depth_files)} depth images")
    
    if len(rgb_files) == 0:
        print("❌ No RGB images found")
        return False
    
    if len(depth_files) == 0:
        print("❌ No depth images found")
        return False
    
    # Test timestamp extraction
    rgb_timestamps = {}
    depth_timestamps = {}
    
    print("\nTesting timestamp extraction...")
    
    # Extract timestamps from RGB filenames
    for rgb_file in rgb_files[:5]:  # Test first 5 files
        try:
            timestamp_str = rgb_file.split('.')[0]
            timestamp = float(timestamp_str)
            rgb_timestamps[timestamp] = rgb_file
            print(f"✅ RGB: {rgb_file} -> {timestamp}")
        except (ValueError, IndexError):
            print(f"❌ Could not parse timestamp from RGB file: {rgb_file}")
    
    # Extract timestamps from depth filenames
    for depth_file in depth_files[:5]:  # Test first 5 files
        try:
            timestamp_str = depth_file.split('.')[0]
            timestamp = float(timestamp_str)
            depth_timestamps[timestamp] = depth_file
            print(f"✅ Depth: {depth_file} -> {timestamp}")
        except (ValueError, IndexError):
            print(f"❌ Could not parse timestamp from depth file: {depth_file}")
    
    # Find common timestamps
    common_timestamps = sorted(set(rgb_timestamps.keys()) & set(depth_timestamps.keys()))
    
    print(f"\nFound {len(common_timestamps)} synchronized pairs in test sample")
    
    if len(common_timestamps) == 0:
        print("❌ No synchronized pairs found")
        return False
    
    # Test image loading
    print("\nTesting image loading...")
    
    for timestamp in common_timestamps[:3]:  # Test first 3 pairs
        rgb_file = rgb_timestamps[timestamp]
        depth_file = depth_timestamps[timestamp]
        
        rgb_img = cv2.imread(os.path.join(rgb_path, rgb_file))
        depth_img = cv2.imread(os.path.join(depth_path, depth_file), cv2.IMREAD_ANYDEPTH)
        
        if rgb_img is not None:
            print(f"✅ RGB image loaded: {rgb_img.shape}")
        else:
            print(f"❌ Failed to load RGB image: {rgb_file}")
            return False
        
        if depth_img is not None:
            print(f"✅ Depth image loaded: {depth_img.shape}")
            print(f"   Depth range: {depth_img.min()} to {depth_img.max()}")
        else:
            print(f"❌ Failed to load depth image: {depth_file}")
            return False
    
    print("\n🎉 Dataset loading test completed successfully!")
    return True

if __name__ == "__main__":
    success = test_dataset_loading()
    if success:
        print("\n✅ Dataset is ready for RGB-D driver!")
    else:
        print("\n❌ Dataset has issues that need to be fixed!") 