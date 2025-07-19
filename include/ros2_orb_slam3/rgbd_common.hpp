// Include file for RGB-D mode
#ifndef RGBD_COMMON_HPP
#define RGBD_COMMON_HPP

// C++ includes
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <cstring>
#include <sstream>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
using std::placeholders::_1;

// Include Eigen
#include <Eigen/Dense>

// Include cv-bridge
#include <cv_bridge/cv_bridge.h>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.h>

// ORB SLAM 3 includes
#include "System.h"

// Global defs
#define pass (void)0

// RGB-D Node specific definitions
class RGBDMode : public rclcpp::Node
{   
    public:
    std::string experimentConfig = ""; // String to receive settings sent by the python driver
    double timeStep; // Timestep data received from the python node
    std::string receivedConfig = "";

    // Class constructor
    RGBDMode(); // Constructor 

    ~RGBDMode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_test/src/ros2_orb_slam3/"; //! Change to match path to your workspace
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package
        bool bSettingsFromPython = false; // Flag set once when experiment setting from python node is received
        
        std::string subexperimentconfigName = ""; // Subscription topic name
        std::string pubconfigackName = ""; // Publisher topic name
        std::string subRGBImgMsgName = ""; // Topic to subscribe to receive RGB images
        std::string subDepthImgMsgName = ""; // Topic to subscribe to receive depth images
        std::string subTimestepMsgName = ""; // Topic to subscribe to receive the timestep

        // Definitions of publisher and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRGBImgMsg_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subDepthImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

        // ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        // ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg); // Callback to process settings sent over by Python node
        void Timestep_callback(const std_msgs::msg::Float64& time_msg); // Callback to process the timestep for this image
        void RGBImg_callback(const sensor_msgs::msg::Image& msg); // Callback to process RGB image
        void DepthImg_callback(const sensor_msgs::msg::Image& msg); // Callback to process depth image
        
        // Helper functions
        void initializeVSLAM(std::string& configString); // Method to bind an initialized VSLAM framework to this node

        // RGB-D specific variables
        cv::Mat lastRGBImage;
        cv::Mat lastDepthImage;
        bool rgbImageReceived = false;
        bool depthImageReceived = false;
        std::mutex imageMutex;
        void processRGBDImages(); // Process synchronized RGB-D images
};

#endif 