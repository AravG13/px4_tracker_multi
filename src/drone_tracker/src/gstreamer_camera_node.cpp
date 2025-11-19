// src/gstreamer_camera_node.cpp - Multi-drone support with dynamic port selection
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class GstreamerCameraNode : public rclcpp::Node
{
public:
    GstreamerCameraNode() : Node("gstreamer_camera_node")
    {
        // Declare parameters
        this->declare_parameter<int>("px4_instance", 0);
        this->declare_parameter<std::string>("camera_source", "auto");
        this->declare_parameter<int>("base_port", 5600);
        this->declare_parameter<int>("frame_width", 640);
        this->declare_parameter<int>("frame_height", 480);
        this->declare_parameter<int>("fps", 30);
        
        // Get parameters
        int px4_instance = this->get_parameter("px4_instance").as_int();
        std::string camera_source = this->get_parameter("camera_source").as_string();
        int base_port = this->get_parameter("base_port").as_int();
        
        // Calculate port for this instance
        // For typhoon_h480: base_port + px4_instance
        // Instance 0: 5600, Instance 1: 5601, Instance 2: 5602, etc.
        // For iris (uncomment if using iris): base_port + (px4_instance * 10)
        int video_port = base_port + px4_instance;
        
        RCLCPP_INFO(this->get_logger(), "=== Multi-Drone Camera Node ===");
        RCLCPP_INFO(this->get_logger(), "PX4 Instance: %d", px4_instance);
        RCLCPP_INFO(this->get_logger(), "Video Port: %d", video_port);
        RCLCPP_INFO(this->get_logger(), "Camera Source: %s", camera_source.c_str());
        
        // Create ROS2 publisher with namespace support
        std::string topic_name = "/camera/image_raw";
        if (px4_instance > 0) {
            topic_name = "/px4_" + std::to_string(px4_instance) + topic_name;
        }
        
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", topic_name.c_str());
        
        // Try to open camera with fallback options
        bool camera_opened = false;
        
        if (camera_source == "auto" || camera_source == "gstreamer") {
            camera_opened = try_gstreamer_pipeline(video_port);
        }
        
        if (!camera_opened && (camera_source == "auto" || camera_source == "usb")) {
            RCLCPP_WARN(this->get_logger(), "Trying USB camera fallback...");
            camera_opened = try_usb_camera();
        }
        
        if (!camera_opened && (camera_source == "auto" || camera_source == "test")) {
            RCLCPP_WARN(this->get_logger(), "Using test pattern fallback...");
            use_test_pattern_ = true;
            camera_opened = true;
        }
        
        if (!camera_opened) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open any camera source!");
            RCLCPP_ERROR(this->get_logger(), "Troubleshooting:");
            RCLCPP_ERROR(this->get_logger(), "1. Check PX4 SITL is running with correct instance");
            RCLCPP_ERROR(this->get_logger(), "2. Verify QGroundControl is closed");
            RCLCPP_ERROR(this->get_logger(), "3. Check port %d is not in use: netstat -tuln | grep %d", 
                        video_port, video_port);
            RCLCPP_ERROR(this->get_logger(), "4. Try: camera_source:=test for testing");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera successfully initialized!");
        
        // Timer to capture and publish frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&GstreamerCameraNode::capture_and_publish, this));
        
        frame_count_ = 0;
    }

private:
    bool try_gstreamer_pipeline(int port)
    {
        // Enhanced GStreamer pipeline with better error handling
        std::string pipeline = 
            "udpsrc port=" + std::to_string(port) + 
            " timeout=3000000000 ! "  // 3 second timeout
            "application/x-rtp,media=(string)video,clock-rate=90000,encoding-name=(string)H264 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=1 max-buffers=2";
        
        RCLCPP_INFO(this->get_logger(), "Trying GStreamer pipeline:");
        RCLCPP_INFO(this->get_logger(), "%s", pipeline.c_str());
        
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "GStreamer pipeline failed to open");
            return false;
        }
        
        // Test read to verify it's working
        cv::Mat test_frame;
        if (!cap_.read(test_frame) || test_frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "GStreamer opened but no frames received");
            cap_.release();
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ GStreamer pipeline working!");
        RCLCPP_INFO(this->get_logger(), "Frame size: %dx%d", test_frame.cols, test_frame.rows);
        return true;
    }
    
    bool try_usb_camera()
    {
        // Try USB camera
        for (int i = 0; i < 4; i++) {
            cap_.open(i);
            if (cap_.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "✓ Opened USB camera /dev/video%d", i);
                
                // Set resolution
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
                cap_.set(cv::CAP_PROP_FPS, 30);
                
                return true;
            }
        }
        return false;
    }
    
    void capture_and_publish()
    {
        cv::Mat frame;
        
        if (use_test_pattern_) {
            // Generate test pattern
            frame = cv::Mat(480, 640, CV_8UC3);
            
            // Draw gradient background
            for (int y = 0; y < frame.rows; y++) {
                for (int x = 0; x < frame.cols; x++) {
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
                        (x * 255) / frame.cols,
                        (y * 255) / frame.rows,
                        128
                    );
                }
            }
            
            // Draw moving circle for testing tracking
            int cx = 320 + static_cast<int>(150 * std::sin(frame_count_ * 0.05));
            int cy = 240 + static_cast<int>(100 * std::cos(frame_count_ * 0.03));
            cv::circle(frame, cv::Point(cx, cy), 40, cv::Scalar(0, 255, 255), -1);
            
            // Add text
            cv::putText(frame, "TEST PATTERN", cv::Point(20, 40), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            
            char frame_text[100];
            snprintf(frame_text, sizeof(frame_text), "Frame: %d", frame_count_);
            cv::putText(frame, frame_text, cv::Point(20, 80), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
            
        } else {
            // Read from actual camera
            if (!cap_.read(frame)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                     "Failed to read frame from camera");
                return;
            }
            
            if (frame.empty()) {
                return;
            }
        }
        
        // Convert to ROS2 message
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_link";
        
        image_pub_->publish(*msg);
        
        frame_count_++;
        
        // Log periodically
        if (frame_count_ % 90 == 0) {  // Every 3 seconds at 30fps
            RCLCPP_INFO(this->get_logger(), 
                       "Published %d frames [%dx%d]", 
                       frame_count_, frame.cols, frame.rows);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    bool use_test_pattern_{false};
    int frame_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GstreamerCameraNode>();
    
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}