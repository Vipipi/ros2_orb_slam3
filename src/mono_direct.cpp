/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Vipipi 
Date: 07/30/2025

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/mono_direct_common.hpp"

//* Constructor
MonocularDirectMode::MonocularDirectMode() :Node("mono_direct_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/isaac-sim.yaml";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());

    // Directly subscribe to /camera/rgb/image_raw
    subImgMsgName = "/camera/rgb/image_raw";
    subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgMsgName, 1, std::bind(&MonocularDirectMode::Img_callback, this, _1)
    );

    // Set default timestep to 0.0
    timeStep = 0.0;

    // Initialize VSLAM immediately with the default settings file
    initializeVSLAM(settingsFilePath);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s and initialized VSLAM.", subImgMsgName.c_str());
}

//* Destructor
MonocularDirectMode::~MonocularDirectMode()
{   
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    if (pAgent) {
        pAgent->Shutdown();
    }
    pass;
}

//* Method to bind an initialized VSLAM framework to this node
void MonocularDirectMode::initializeVSLAM(std::string& configString){
    // Watchdog, if the paths to vocabulary and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 

    // If configString is a path to a .yaml, use as is, else append .yaml
    if (configString.size() < 5 || configString.substr(configString.size() - 5) != ".yaml") {
        settingsFilePath = configString + ".yaml";
    } else {
        settingsFilePath = configString;
    }

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "MonocularDirectMode node initialized with Pangolin viewer: %s", 
                 enablePangolinWindow ? "ENABLED" : "DISABLED");
    
    // Initialize OpenCV window for current frame display
    cv::namedWindow("Current Frame", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "OpenCV window 'Current Frame' initialized");
}

//* Callback to process image message and run SLAM node
void MonocularDirectMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // DEBUG: Log message reception
    processedImageCount++;
    // RCLCPP_INFO(this->get_logger(), "Received image message #%d - Size: %dx%d, Encoding: %s", 
    //              processedImageCount, msg.width, msg.height, msg.encoding.c_str());
    
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUG: Log successful conversion
        // RCLCPP_INFO(this->get_logger(), "Successfully converted image to OpenCV format - Size: %dx%d", 
        //             cv_ptr->image.cols, cv_ptr->image.rows);
        
        // DEBUG: Check image properties for tracking
        if (processedImageCount % 30 == 0) { // Log every 30th frame to avoid spam
            RCLCPP_INFO(this->get_logger(), "Image #%d - Size: %dx%d, Channels: %d, Type: %d", 
                        processedImageCount, cv_ptr->image.cols, cv_ptr->image.rows, 
                        cv_ptr->image.channels(), cv_ptr->image.type());
        }
        
        // DEBUGGING, Show image
        // Update GUI Window
        cv::imshow("Current Frame", cv_ptr->image);
        cv::waitKey(1); // Wait 1ms for key press, allows window to update
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image: %s", e.what());
        return;
    }
    
    // Use ROS time stamp as timeStep if available, else fallback to 0.0
    double img_time = 0.0;
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
        img_time = rclcpp::Time(msg.header.stamp).seconds();
    } else {
        img_time = timeStep;
    }

    // DEBUG: Log timestamp info
    // RCLCPP_INFO(this->get_logger(), "Processing image with timestamp: %.6f", img_time);

    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, img_time); 
    
    // DEBUG: Check tracking state and pose validity
    if (Tcw.log().norm() < 1e-10) {
        RCLCPP_WARN(this->get_logger(), "ORB-SLAM3: No valid pose computed (identity matrix) - Image #%d", processedImageCount);
        
        // Additional debugging for tracking issues
        if (processedImageCount < 10) {
            RCLCPP_INFO(this->get_logger(), "ORB-SLAM3: Still initializing... (first 10 frames)");
        } else if (processedImageCount < 50) {
            RCLCPP_INFO(this->get_logger(), "ORB-SLAM3: Waiting for sufficient camera motion...");
        } else {
            RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3: Tracking failed - possible issues: insufficient features, no motion, or poor image quality");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3: Valid pose computed - Translation: [%.3f, %.3f, %.3f]", 
                    Tcw.translation().x(), Tcw.translation().y(), Tcw.translation().z());
    }
    
    // DEBUG: Check system state
    // ORB_SLAM3::System::eSensor sensor = pAgent->GetSensor();
    // RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 sensor type: %d", static_cast<int>(sensor));
    
    // Force viewer update (if needed)
    // Note: ORB-SLAM3 viewer updates are typically handled internally
    // RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 tracking completed - Viewer should be updated");
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use
}
