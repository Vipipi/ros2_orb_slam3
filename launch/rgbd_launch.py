#!/usr/bin/env python3

"""
Launch file for RGB-D ORB-SLAM3 ROS2 wrapper

This launch file starts both the C++ ORB-SLAM3 node and the Python driver node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name_arg',
        default_value='rgbd_node',
        description='Name of the RGB-D node'
    )
    
    voc_file_arg = DeclareLaunchArgument(
        'voc_file_arg',
        default_value='file_not_set',
        description='Path to ORB vocabulary file'
    )
    
    settings_file_path_arg = DeclareLaunchArgument(
        'settings_file_path_arg',
        default_value='file_path_not_set',
        description='Path to settings file directory'
    )
    
    # Create the C++ ORB-SLAM3 RGB-D node
    rgbd_cpp_node = Node(
        package='ros2_orb_slam3',
        executable='rgbd_node_cpp',
        name='rgbd_cpp_node',
        parameters=[{
            'node_name_arg': LaunchConfiguration('node_name_arg'),
            'voc_file_arg': LaunchConfiguration('voc_file_arg'),
            'settings_file_path_arg': LaunchConfiguration('settings_file_path_arg')
        }],
        output='screen'
    )
    
    # Create the Python RGB-D driver node
    rgbd_py_node = Node(
        package='ros2_orb_slam3',
        executable='rgbd_driver_node.py',
        name='rgbd_py_node',
        output='screen'
    )
    
    return LaunchDescription([
        node_name_arg,
        voc_file_arg,
        settings_file_path_arg,
        rgbd_cpp_node,
        rgbd_py_node
    ]) 