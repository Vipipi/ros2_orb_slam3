/*

RGB-D example node for ORB-SLAM3 ROS2 wrapper

Author: Assistant
Date: 2024

*/

#include "ros2_orb_slam3/rgbd_common.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RGBDMode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 