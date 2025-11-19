// src/tracker.cpp - Complete enhanced version with scale adaptation
#include "drone_tracker/tracker.hpp"
#include <cv_bridge/cv_bridge.h>

static TrackerNode* g_tracker_node_instance = nullptr;

TrackerNode::TrackerNode() : Node("tracker_node") {
    this->declare_parameter<int>("px4_instance", 0);
    int px4_instance = this->get_parameter("px4_instance").as_int();
    
    std::string camera_topic = "/camera/image_raw";
    if (px4_instance > 0) {
        camera_topic = "/px4_" + std::to_string(px4_instance) + "/camera/image_raw";
    }
    
    RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_target", 10);
    enhanced_target_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/enhanced_target_data", 10);
    bbox_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/target_bbox_info", 10);
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,  // Use camera_topic instead of hardcoded "/camera/image_raw"
        std::bind(&TrackerNode::image_callback, this, std::placeholders::_1));
    
    tracker_ = std::make_unique<ObjectTracker>();
    distance_estimator_ = std::make_unique<Distance3DEstimator>(1.0, 350.0);
    feature_detector_ = cv::ORB::create(500);
    
    pid_x_ = std::make_unique<StablePIDController>(0.8, 0.01, 0.1, 1.5, 0.3);
    pid_y_ = std::make_unique<StablePIDController>(0.8, 0.01, 0.1, 1.5, 0.3);
    pid_z_ = std::make_unique<StablePIDController>(0.3, 0.005, 0.02, 0.8, 0.1);
    pid_distance_ = std::make_unique<StablePIDController>(0.4, 0.02, 0.08, 1.2, 0.2);
    
    last_time_ = this->now();
    last_reinit_attempt_ = this->now();
    g_tracker_node_instance = this;
    
    RCLCPP_INFO(this->get_logger(), "Enhanced Tracker Node with Scale Adaptation initialized");
}

bool TrackerNode::validate_bbox(const cv::Rect& bbox, int frame_width, int frame_height) {
    return (bbox.width >= 5 && bbox.height >= 5 &&
            bbox.x >= 0 && bbox.y >= 0 &&
            bbox.x + bbox.width <= frame_width &&
            bbox.y + bbox.height <= frame_height);
}

double TrackerNode::estimate_scale_change(const cv::Mat& frame, const cv::Rect& bbox) {
    if (reference_keypoints_.empty() || reference_descriptors_.empty()) {
        return 1.0;
    }
    
    if (bbox.x < 0 || bbox.y < 0 || 
        bbox.x + bbox.width > frame.cols || 
        bbox.y + bbox.height > frame.rows) {
        return 1.0;
    }
    
    cv::Mat roi = frame(bbox);
    if (roi.empty()) return 1.0;
    
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat current_descriptors;
    
    try {
        feature_detector_->detectAndCompute(roi, cv::noArray(), current_keypoints, current_descriptors);
    } catch (...) {
        return 1.0;
    }
    
    if (current_keypoints.size() < 5 || current_descriptors.empty()) {
        return 1.0;
    }
    
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    
    try {
        matcher.match(reference_descriptors_, current_descriptors, matches);
    } catch (...) {
        return 1.0;
    }
    
    if (matches.size() < 10) {
        return 1.0;
    }
    
    double total_scale = 0.0;
    int valid_matches = 0;
    
    for (const auto& match : matches) {
        if (match.distance < 50) {
            double ref_size = reference_keypoints_[match.queryIdx].size;
            double curr_size = current_keypoints[match.trainIdx].size;
            
            if (ref_size > 0) {
                total_scale += curr_size / ref_size;
                valid_matches++;
            }
        }
    }
    
    if (valid_matches > 5) {
        double avg_scale = total_scale / valid_matches;
        return std::max(0.5, std::min(2.0, avg_scale));
    }
    
    return 1.0;
}

cv::Rect TrackerNode::apply_scale_estimation(const cv::Rect& current_bbox, const cv::Mat& frame) {
    double scale_factor = estimate_scale_change(frame, current_bbox);
    
    scale_history_.push_back(scale_factor);
    if (scale_history_.size() > MAX_SCALE_HISTORY) {
        scale_history_.pop_front();
    }
    
    double smoothed_scale = 0.0;
    for (double s : scale_history_) {
        smoothed_scale += s;
    }
    smoothed_scale /= scale_history_.size();
    
    if (std::abs(smoothed_scale - 1.0) < 0.05) {
        return current_bbox;
    }
    
    int new_width = static_cast<int>(current_bbox.width * smoothed_scale);
    int new_height = static_cast<int>(current_bbox.height * smoothed_scale);
    
    int new_x = current_bbox.x + (current_bbox.width - new_width) / 2;
    int new_y = current_bbox.y + (current_bbox.height - new_height) / 2;
    
    return cv::Rect(new_x, new_y, new_width, new_height);
}

void TrackerNode::attempt_tracker_reinitialization(cv::Mat& frame, const cv::Rect& last_known_bbox) {
    auto now = this->now();
    auto time_since_last_attempt = now - last_reinit_attempt_;
    
    if (time_since_last_attempt.seconds() < 2.0) {
        return;
    }
    
    last_reinit_attempt_ = now;
    
    RCLCPP_WARN(this->get_logger(), "Attempting tracker reinitialization...");
    
    cv::Rect search_region = last_known_bbox;
    int expand = 20;
    search_region.x = std::max(0, search_region.x - expand);
    search_region.y = std::max(0, search_region.y - expand);
    search_region.width = std::min(frame.cols - search_region.x, search_region.width + 2*expand);
    search_region.height = std::min(frame.rows - search_region.y, search_region.height + 2*expand);
    
    if (!validate_bbox(search_region, frame.cols, frame.rows)) {
        return;
    }
    
    cv::Ptr<cv::Tracker> new_tracker;
    try {
        new_tracker = cv::TrackerCSRT::create();
        new_tracker->init(frame, search_region);
        
        tracker_->manual_init(new_tracker, search_region);
        tracker_initialized_ = true;
        consecutive_failures_ = 0;
        tracking_failures_ = 0;
        
        cv::Mat roi = frame(search_region);
        reference_keypoints_.clear();
        feature_detector_->detectAndCompute(roi, cv::noArray(), reference_keypoints_, reference_descriptors_);
        
        RCLCPP_INFO(this->get_logger(), "Tracker successfully reinitialized!");
        
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reinitialize tracker");
    }
}

void TrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat frame = cv_ptr->image;
    current_frame_ = frame.clone();
    
    int frame_width = frame.cols;
    int frame_height = frame.rows;
    int frame_center_x = frame_width / 2;
    int frame_center_y = frame_height / 2;
    
    if (roi_selection_active_ && roi_ready_) {
        cv::Ptr<cv::Tracker> tracker;
        try {
            tracker = cv::TrackerCSRT::create();
        } catch (...) {
            try {
                tracker = cv::TrackerKCF::create();
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create tracker");
                roi_selection_active_ = false;
                roi_ready_ = false;
                return;
            }
        }
        
        if (tracker) {
            tracker->init(frame, roi_rect_);
            tracker_->manual_init(tracker, roi_rect_);
            tracker_initialized_ = true;
            tracking_failures_ = 0;
            consecutive_failures_ = 0;
            roi_selection_active_ = false;
            roi_ready_ = false;
            
            cv::Mat roi = frame(roi_rect_);
            reference_keypoints_.clear();
            reference_descriptors_ = cv::Mat();
            feature_detector_->detectAndCompute(roi, cv::noArray(), reference_keypoints_, reference_descriptors_);
            scale_history_.clear();
            last_valid_bbox_ = roi_rect_;
            
            stabilization_active_ = true;
            stabilization_start_ = this->now();
            last_time_ = this->now();
            
            RCLCPP_INFO(this->get_logger(), "Tracker initialized! Stabilizing...");
        }
    }
    
    if (stabilization_active_) {
        auto stabilization_elapsed = this->now() - stabilization_start_;
        if (stabilization_elapsed.seconds() >= 3.0) {
            stabilization_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Stabilization complete!");
        }
    }
    
    cv::Point obj_center;
    bool tracking_ok = false;
    cv::Rect current_bbox;
    
    if (tracker_initialized_) {
        tracking_ok = tracker_->update(frame, obj_center);
        
        if (tracking_ok) {
            current_bbox = tracker_->get_bbox();
            
            if (!validate_bbox(current_bbox, frame_width, frame_height)) {
                tracking_ok = false;
            } else {
                cv::Rect scaled_bbox = apply_scale_estimation(current_bbox, frame);
                if (validate_bbox(scaled_bbox, frame_width, frame_height)) {
                    current_bbox = scaled_bbox;
                    last_valid_bbox_ = current_bbox;
                }
            }
        }
        
        if (!tracking_ok) {
            tracking_failures_++;
            consecutive_failures_++;
            
            if (consecutive_failures_ > 10 && consecutive_failures_ < 30) {
                attempt_tracker_reinitialization(frame, last_valid_bbox_);
            }
            
            if (tracking_failures_ > 50) {
                RCLCPP_WARN(this->get_logger(), "Tracking lost - resetting");
                tracker_initialized_ = false;
                tracking_failures_ = 0;
                consecutive_failures_ = 0;
                stabilization_active_ = false;
            }
        } else {
            consecutive_failures_ = 0;
            tracking_failures_ = std::max(0, tracking_failures_ - 1);
        }
    }
    
    if (roi_selection_active_) {
        if (roi_drawing_) {
            cv::rectangle(frame, roi_start_, roi_end_, cv::Scalar(0, 255, 255), 2);
        } else if (roi_ready_) {
            cv::rectangle(frame, roi_rect_, cv::Scalar(0, 255, 0), 2);
        }
    }
    
    cv::line(frame, cv::Point(frame_center_x-20, frame_center_y), 
             cv::Point(frame_center_x+20, frame_center_y), cv::Scalar(255,255,255), 2);
    cv::line(frame, cv::Point(frame_center_x, frame_center_y-20), 
             cv::Point(frame_center_x, frame_center_y+20), cv::Scalar(255,255,255), 2);
    
    if (tracking_ok && tracker_initialized_ && !stabilization_active_) {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        dt = std::max(std::min(dt, 0.1), 0.01);
        last_time_ = current_time;
        
        double norm_x = (obj_center.x - frame_center_x) / double(frame_center_x);
        double norm_y = (obj_center.y - frame_center_y) / double(frame_center_y);
        
        bool roi_safe = is_roi_fully_visible(current_bbox, frame_width, frame_height, 50);
        double boundary_error = 0.0;
        
        if (!roi_safe) {
            int margin = 30;
            if (current_bbox.x < margin) boundary_error += (margin - current_bbox.x) / double(frame_width);
            if (current_bbox.y < margin) boundary_error += (margin - current_bbox.y) / double(frame_height);
            if (current_bbox.x + current_bbox.width > frame_width - margin) 
                boundary_error += (current_bbox.x + current_bbox.width - (frame_width - margin)) / double(frame_width);
            if (current_bbox.y + current_bbox.height > frame_height - margin) 
                boundary_error += (current_bbox.y + current_bbox.height - (frame_height - margin)) / double(frame_height);
        }
        
        if (!backing_away_mode_ && (!roi_safe || boundary_error > 0.02)) {
            backing_away_mode_ = true;
            backup_start_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "STARTING BACKUP MODE");
        } else if (backing_away_mode_) {
            auto backup_elapsed = current_time - backup_start_time_;
            if (backup_elapsed.seconds() >= 2.0 && roi_safe && boundary_error < 0.01) {
                backing_away_mode_ = false;
                RCLCPP_INFO(this->get_logger(), "BACKUP COMPLETE");
            }
        }
        
        double estimated_distance = distance_estimator_->estimate_distance(current_bbox);
        double current_size = calculate_bbox_size(current_bbox);
        double distance_error = 0.0;
        
        if (backing_away_mode_) {
            auto backup_elapsed = current_time - backup_start_time_;
            double backup_intensity = std::max(1.0, 2.0 * std::exp(-backup_elapsed.seconds() / 2.0));
            distance_error = backup_intensity;
        } else {
            const double TARGET_DISTANCE_PIXELS = 90.0;
            if (estimated_distance > 0 && estimated_distance < 10.0) {
                distance_error = (estimated_distance - target_distance_) / target_distance_;
            } else {
                distance_error = (TARGET_DISTANCE_PIXELS - current_size) / TARGET_DISTANCE_PIXELS;
            }
        }
        
        distance_error = std::max(std::min(distance_error, 1.2), -0.4);
        
        const double POSITION_DEADZONE = 0.08;
        const double ALTITUDE_DEADZONE = 0.12;
        const double DISTANCE_DEADZONE = backing_away_mode_ ? 0.05 : 0.10;
        
        double vx = 0.0, vy = 0.0, vz = 0.0;
        
        if (std::abs(norm_x) > POSITION_DEADZONE) {
            vy = pid_y_->compute(norm_x, dt);
        } else {
            pid_y_->reset();
        }
        
        if (std::abs(norm_y) > POSITION_DEADZONE) {
            if (std::abs(norm_y) < ALTITUDE_DEADZONE) {
                vx = pid_x_->compute(norm_y, dt) * 0.5;
            } else {
                vx = pid_x_->compute(norm_y, dt);
            }
        } else {
            pid_x_->reset();
        }
        
        if (std::abs(distance_error) > DISTANCE_DEADZONE) {
            double distance_ctrl = pid_distance_->compute(distance_error, dt);
            vx += -distance_ctrl;
        } else if (!backing_away_mode_) {
            pid_distance_->reset();
        }
        
        if (std::abs(norm_y) > ALTITUDE_DEADZONE) {
            vz = -pid_z_->compute(norm_y, dt) * 0.3;
        } else {
            pid_z_->reset();
        }
        
        vx = std::max(std::min(vx, 1.5), -1.5);
        vy = std::max(std::min(vy, 1.5), -1.5);
        vz = std::max(std::min(vz, 0.8), -0.8);
        
        if (std::abs(vx) < 0.08) vx = 0.0;
        if (std::abs(vy) < 0.08) vy = 0.0;
        if (std::abs(vz) < 0.05) vz = 0.0;
        
        auto target_msg = geometry_msgs::msg::PointStamped();
        target_msg.header.stamp = this->now();
        target_msg.header.frame_id = "camera_link";
        target_msg.point.x = norm_x * 1.5;
        target_msg.point.y = norm_y * 1.5;
        target_msg.point.z = 0.0;
        target_pub_->publish(target_msg);
        
        auto bbox_msg = geometry_msgs::msg::Vector3();
        bbox_msg.x = current_bbox.width * current_bbox.height;
        bbox_msg.y = 1.0;
        bbox_msg.z = roi_safe ? 0.0 : 1.0;
        bbox_pub_->publish(bbox_msg);
        
        draw_enhanced_feedback(frame, obj_center, current_bbox, vx, vy, vz, estimated_distance, roi_safe);
        
        if (!scale_history_.empty()) {
            double avg_scale = 0.0;
            for (double s : scale_history_) avg_scale += s;
            avg_scale /= scale_history_.size();
            
            char scale_text[100];
            snprintf(scale_text, sizeof(scale_text), "Scale: %.2fx", avg_scale);
            cv::putText(frame, scale_text, cv::Point(10, 110), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,100,200), 1);
        }
    }
    
    draw_status(frame);
    
    cv::imshow("Drone Tracker", frame);
    cv::setMouseCallback("Drone Tracker", TrackerNode::mouse_callback, nullptr);
    int key = cv::waitKey(1) & 0xFF;
    handle_key_input(key);
}

void TrackerNode::mouse_callback(int event, int x, int y, int, void*) {
    TrackerNode* node = g_tracker_node_instance;
    if (!node) return;
    
    if (event == cv::EVENT_LBUTTONDOWN && !node->roi_selection_active_) {
        node->roi_start_ = cv::Point(x, y);
        node->roi_drawing_ = true;
        node->roi_ready_ = false;
        node->roi_selection_active_ = true;
    }
    else if (event == cv::EVENT_MOUSEMOVE && node->roi_drawing_) {
        node->roi_end_ = cv::Point(x, y);
    }
    else if (event == cv::EVENT_LBUTTONUP && node->roi_drawing_) {
        node->roi_end_ = cv::Point(x, y);
        node->roi_drawing_ = false;
        
        int x1 = std::min(node->roi_start_.x, node->roi_end_.x);
        int y1 = std::min(node->roi_start_.y, node->roi_end_.y);
        int x2 = std::max(node->roi_start_.x, node->roi_end_.x);
        int y2 = std::max(node->roi_start_.y, node->roi_end_.y);
        
        if (x2 - x1 > 20 && y2 - y1 > 20) {
            node->roi_rect_ = cv::Rect(x1, y1, x2 - x1, y2 - y1);
            node->roi_ready_ = true;
        } else {
            node->roi_selection_active_ = false;
        }
    }
}

bool TrackerNode::is_roi_fully_visible(const cv::Rect& bbox, int frame_width, int frame_height, int margin) {
    int effective_margin = backing_away_mode_ ? margin + 40 : margin;
    return (bbox.x >= effective_margin && 
            bbox.y >= effective_margin && 
            bbox.x + bbox.width <= frame_width - effective_margin && 
            bbox.y + bbox.height <= frame_height - effective_margin);
}

double TrackerNode::calculate_bbox_size(const cv::Rect& bbox) {
    return std::sqrt(bbox.width * bbox.width + bbox.height * bbox.height);
}

void TrackerNode::draw_enhanced_feedback(cv::Mat& frame, const cv::Point& obj_center, 
                                        const cv::Rect& /*bbox*/, double vx, double vy, double vz,
                                        double distance, bool roi_safe) {
    int cx = frame.cols / 2;
    int cy = frame.rows / 2;
    
    cv::circle(frame, obj_center, 8, cv::Scalar(0,255,0), -1);
    cv::line(frame, cv::Point(cx, cy), obj_center, cv::Scalar(0,255,0), 2);
    
    int margin = 30;
    cv::Scalar boundary_color = roi_safe ? cv::Scalar(0,200,0) : cv::Scalar(0,0,200);
    cv::rectangle(frame, cv::Point(margin, margin), 
                 cv::Point(frame.cols - margin, frame.rows - margin), boundary_color, 2);
    
    char cmd_text[100];
    snprintf(cmd_text, sizeof(cmd_text), "Vel: vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz);
    cv::putText(frame, cmd_text, cv::Point(10, 70), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,255), 1);
    
    if (distance > 0) {
        char dist_text[100];
        snprintf(dist_text, sizeof(dist_text), "Distance: %.2fm", distance);
        cv::putText(frame, dist_text, cv::Point(10, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,255), 1);
    }
}

void TrackerNode::draw_status(cv::Mat& frame) {
    std::string status;
    cv::Scalar color;
    
    if (roi_selection_active_) {
        status = roi_ready_ ? "ROI READY - Processing" : "SELECT OBJECT - Drag mouse";
        color = roi_ready_ ? cv::Scalar(0,255,0) : cv::Scalar(0,255,255);
    } else if (stabilization_active_) {
        status = "STABILIZING...";
        color = cv::Scalar(255,128,0);
    } else if (!tracker_initialized_) {
        status = "READY - Click and drag to select object";
        color = cv::Scalar(0,150,255);
    } else {
        status = "TRACKING ACTIVE (Scale Adaptive)";
        color = cv::Scalar(0,255,0);
        
        if (tracking_failures_ > 0) {
            status += " [recovering: " + std::to_string(tracking_failures_) + "/50]";
            color = cv::Scalar(255,255,0);
        }
    }
    
    cv::putText(frame, status, cv::Point(10, frame.rows - 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}

void TrackerNode::handle_key_input(int key) {
    if (key == 'r' || key == 'R') {
        RCLCPP_INFO(this->get_logger(), "Resetting tracker");
        tracker_initialized_ = false;
        tracking_failures_ = 0;
        consecutive_failures_ = 0;
        roi_selection_active_ = false;
        roi_ready_ = false;
        stabilization_active_ = false;
        backing_away_mode_ = false;
        
        reference_keypoints_.clear();
        reference_descriptors_ = cv::Mat();
        scale_history_.clear();
        
        pid_x_->reset();
        pid_y_->reset();
        pid_z_->reset();
        pid_distance_->reset();
    }
}