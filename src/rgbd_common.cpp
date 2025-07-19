/*

RGB-D mode implementation for ORB-SLAM3 ROS2 wrapper

Author: Assistant
Date: 2024

REQUIREMENTS
* Make sure to set path to your workspace in rgbd_common.hpp file

*/

// Includes
#include "ros2_orb_slam3/rgbd_common.hpp"

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
    subTimestepMsgName = "/rgbd_py_driver/timestep_msg"; // topic to receive timestep messages

    // subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&RGBDMode::experimentSetting_callback, this, _1));

    // publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    // subscribe to the RGB image messages coming from the Python driver node
    subRGBImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subRGBImgMsgName, 1, std::bind(&RGBDMode::RGBImg_callback, this, _1));

    // subscribe to the depth image messages coming from the Python driver node
    subDepthImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subDepthImgMsgName, 1, std::bind(&RGBDMode::DepthImg_callback, this, _1));

    // subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&RGBDMode::Timestep_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
}

// Destructor
RGBDMode::~RGBDMode()
{   
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
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
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/RGBD/TUM1.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::RGBD; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "RGBDMode node initialized" << std::endl;
}

// Callback that processes timestep sent over ROS
void RGBDMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    timeStep = time_msg.data;
}

// Callback to process RGB image message
void RGBDMode::RGBImg_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr;
    
    // Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading RGB image");
        return;
    }
    
    // Store RGB image
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        lastRGBImage = cv_ptr->image.clone();
        rgbImageReceived = true;
    }
    
    // Process if both images are available
    processRGBDImages();
}

// Callback to process depth image message
void RGBDMode::DepthImg_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr;
    
    // Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading depth image");
        return;
    }
    
    // Store depth image
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        lastDepthImage = cv_ptr->image.clone();
        depthImageReceived = true;
    }
    
    // Process if both images are available
    processRGBDImages();
}

// Process synchronized RGB-D images
void RGBDMode::processRGBDImages()
{
    std::lock_guard<std::mutex> lock(imageMutex);
    
    if (rgbImageReceived && depthImageReceived)
    {
        // Perform all ORB-SLAM3 operations in RGB-D mode
        Sophus::SE3f Tcw = pAgent->TrackRGBD(lastRGBImage, lastDepthImage, timeStep);
        
        // Reset flags for next frame
        rgbImageReceived = false;
        depthImageReceived = false;
    }
} 