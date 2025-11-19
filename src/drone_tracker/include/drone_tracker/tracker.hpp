// include/drone_tracker/tracker.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <memory>
#include "drone_tracker/object_tracker.hpp"
#include <geometry_msgs/msg/vector3.hpp>

// PID Controller
class StablePIDController {
public:
    StablePIDController(double kp, double ki, double kd, double max_output = 1.0, double max_integral = 0.3) 
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), max_integral_(max_integral),
          prev_error_(0), integral_(0), prev_derivative_(0) {}
    
    double compute(double error, double dt) {
        double proportional = kp_ * error;
        
        integral_ += error * dt;
        integral_ = std::max(std::min(integral_, max_integral_), -max_integral_);
        double integral_term = ki_ * integral_;
        
        double derivative = (error - prev_error_) / dt;
        derivative = 0.8 * prev_derivative_ + 0.2 * derivative;
        prev_derivative_ = derivative;
        double derivative_term = kd_ * derivative;
        
        double output = proportional + integral_term + derivative_term;
        output = std::max(std::min(output, max_output_), -max_output_);
        
        prev_error_ = error;
        return output;
    }
    
    void reset() {
        prev_error_ = 0;
        integral_ = 0;
        prev_derivative_ = 0;
    }
    
private:
    double kp_, ki_, kd_, max_output_, max_integral_;
    double prev_error_, integral_, prev_derivative_;
};

// Distance Estimator
class Distance3DEstimator {
public:
    Distance3DEstimator(double known_object_size_m = 1.0, double camera_focal_length_px = 350.0)
        : known_object_size_m_(known_object_size_m), focal_length_px_(camera_focal_length_px) {}
    
    double estimate_distance(const cv::Rect& bbox) const {
        if (bbox.width <= 0 || bbox.height <= 0) return -1.0;
        
        double bbox_diagonal_px = std::sqrt(bbox.width * bbox.width + bbox.height * bbox.height);
        if (bbox_diagonal_px < 10.0) return -1.0;
        
        double estimated_distance = (known_object_size_m_ * focal_length_px_) / bbox_diagonal_px;
        return std::max(1.0, std::min(50.0, estimated_distance));
    }
    
private:
    double known_object_size_m_;
    double focal_length_px_;
};

class TrackerNode : public rclcpp::Node {
public:
  TrackerNode();
  
  static void mouse_callback(int event, int x, int y, int flags, void* userdata);

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // Helper methods
  bool is_roi_fully_visible(const cv::Rect& bbox, int frame_width, int frame_height, int margin = 30);
  double calculate_bbox_size(const cv::Rect& bbox);
  void draw_enhanced_feedback(cv::Mat& frame, const cv::Point& obj_center, 
                            const cv::Rect& bbox, double vx, double vy, double vz,
                            double distance, bool roi_safe);
  void draw_status(cv::Mat& frame);
  void handle_key_input(int key);
  
  // Scale adaptation methods
  void attempt_tracker_reinitialization(cv::Mat& frame, const cv::Rect& last_known_bbox);
  bool validate_bbox(const cv::Rect& bbox, int frame_width, int frame_height);
  cv::Rect apply_scale_estimation(const cv::Rect& current_bbox, const cv::Mat& frame);
  double estimate_scale_change(const cv::Mat& frame, const cv::Rect& bbox);

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr enhanced_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr bbox_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Core components
  std::unique_ptr<ObjectTracker> tracker_;
  std::unique_ptr<Distance3DEstimator> distance_estimator_;
  
  // PID Controllers
  std::unique_ptr<StablePIDController> pid_x_;
  std::unique_ptr<StablePIDController> pid_y_;
  std::unique_ptr<StablePIDController> pid_z_;
  std::unique_ptr<StablePIDController> pid_distance_;

  // State variables
  bool tracker_initialized_{false};
  bool enable_3d_tracking_{true};
  bool roi_selection_active_{false};
  bool roi_ready_{false};
  bool roi_drawing_{false};
  bool stabilization_active_{false};
  bool backing_away_mode_{false};
  
  int tracking_failures_{0};
  int consecutive_failures_{0};
  double target_distance_{3.5};
  
  // Diagnostic variables
  bool first_frame_received_{false};
  int frame_count_{0};
  rclcpp::TimerBase::SharedPtr camera_check_timer_;
  
  // Scale adaptation
  cv::Rect last_valid_bbox_;
  std::deque<double> scale_history_;
  static const size_t MAX_SCALE_HISTORY = 10;
  cv::Ptr<cv::ORB> feature_detector_;
  std::vector<cv::KeyPoint> reference_keypoints_;
  cv::Mat reference_descriptors_;
  
  // ROI selection
  cv::Point roi_start_, roi_end_;
  cv::Rect roi_rect_;
  cv::Mat current_frame_;
  
  // Timing
  rclcpp::Time last_time_;
  rclcpp::Time stabilization_start_;
  rclcpp::Time backup_start_time_;
  rclcpp::Time last_reinit_attempt_;
};