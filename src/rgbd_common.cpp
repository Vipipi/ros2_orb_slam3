/*

RGB-D mode implementation for ORB-SLAM3 ROS2 wrapper

Author: Assistant
Date: 2024

REQUIREMENTS
* Make sure to set path to your workspace in rgbd_common.hpp file

*/

// Includes
#include "ros2_orb_slam3/rgbd_common.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Constructor
RGBDMode::RGBDMode() :Node("rgbd_node_cpp")
{
    // Declare parameters to be passed from command line
    homeDir = getenv("HOME");
    
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 RGB-D NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overridden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    // Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    // Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();
    
    // HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGBD/";
    }
    
    // DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    
    subexperimentconfigName = "/rgbd_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp node
    pubconfigackName = "/rgbd_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subRGBImgMsgName = "/rgbd_py_driver/rgb_img_msg"; // topic to receive RGB image messages
    subDepthImgMsgName = "/rgbd_py_driver/depth_img_msg"; // topic to receive depth image messages
    // No separate timestep topic; timestamps will be read from image headers

    // subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&RGBDMode::experimentSetting_callback, this, _1));

    // publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    // Setup message filters for synchronized RGB-D
    rgb_sub_.subscribe(this, subRGBImgMsgName);
    depth_sub_.subscribe(this, subDepthImgMsgName);
    
    // Create synchronizer with 10-frame queue and 0.1s time tolerance
    sync_ = std::make_shared<Sync>(approximate_sync_policy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&RGBDMode::RGBDCallback, this, std::placeholders::_1, std::placeholders::_2));

    // No separate timestep subscription

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

// Destructor
RGBDMode::~RGBDMode()
{   
    // Save trajectories (timestamps match the Float64 timeStep you published)
    try {
        if (pAgent) {
            // Ensure output directory exists and build target paths
            const std::string frame_path = trajectoryOutputDir + "/FrameTrajectory.txt";
            const std::string kf_path = trajectoryOutputDir + "/KeyFrameTrajectory.txt";
            pAgent->SaveTrajectoryTUM(frame_path);
            pAgent->SaveKeyFrameTrajectoryTUM(kf_path);
        }
    } catch (...) {}

    // Release resources and cleanly shutdown
    if (pAgent) {
        pAgent->Shutdown();

        // After shutdown, dump corrected trajectories (post loop-closure/pose-graph updates)
        try {
            const std::string frame_path_corrected = trajectoryOutputDir + "/FrameTrajectory-corrected.txt";
            const std::string kf_path_corrected = trajectoryOutputDir + "/KeyFrameTrajectory-corrected.txt";
            pAgent->SaveTrajectoryTUM(frame_path_corrected);
            pAgent->SaveKeyFrameTrajectoryTUM(kf_path_corrected);
        } catch (...) {}
    }
    pass;
}

// Callback which accepts experiment parameters from the Python node
void RGBDMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    // Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    // Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);
}

// Method to bind an initialized VSLAM framework to this node
void RGBDMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabulary and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    // Build .yaml file path
    std::string fullSettingsPath = settingsFilePath + configString + ".yaml";
    
    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", fullSettingsPath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::RGBD; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, fullSettingsPath, sensorType, enablePangolinWindow);
    std::cout << "RGBDMode node initialized" << std::endl;
}

// Removed Timestep_callback; we use header.stamp from image messages

// Synchronized RGB-D callback using message filters
void RGBDMode::RGBDCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                           const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    // Convert RGB image
    cv_bridge::CvImagePtr cv_rgb_ptr;
    try {
        cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading RGB image");
        return;
    }
    
    // Convert depth image
    cv_bridge::CvImagePtr cv_depth_ptr;
    try {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        
        // Note: ORB-SLAM3 automatically applies DepthMapFactor (1/1000.0) 
        // from the configuration file to convert mm to meters
        // No additional conversion needed here
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading depth image");
        return;
    }
    
    // Use the dataset timestamp from the synchronized header stamp
    const double ts = rclcpp::Time(rgb_msg->header.stamp).seconds();
    Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_rgb_ptr->image, cv_depth_ptr->image, ts);
    
    // Debug output for tracking status
    if (Tcw.log().norm() < 1e-10) {
        RCLCPP_WARN(this->get_logger(), "ORB-SLAM3 RGB-D: No valid pose computed");
    } else {
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGB-D: Valid pose computed - Translation: [%.3f, %.3f, %.3f]", 
                    Tcw.translation().x(), Tcw.translation().y(), Tcw.translation().z());
    }
} 