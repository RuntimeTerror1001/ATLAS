/**
 * @brief TF Health Check Node for Atlas State Estimation
 * 
 * This node monitors critical TF transforms and publishes diagnostic information
 * to help identify state estimation issues. It checks:
 * - odom -> base_frame (required, published by EKF)
 * - map -> odom (optional, will be published by SLAM later)
 * and logs actionable warnings.
 */

 #include <chrono>
 #include <memory>
 #include <string>
 #include <vector>

 #include "rclcpp/rclcpp.hpp"
 #include "tf2_ros/transform_listener.h"
 #include "tf2_ros/buffer.h"
 #include "tf2/exceptions.h"
 #include "geometry_msgs/msg/transform_stamped.hpp"
 #include "diagnostic_msgs/msg/diagnostic_array.hpp"
 #include "diagnostic_msgs/msg/diagnostic_status.hpp"
 #include "diagnostic_msgs/msg/key_value.hpp"

 class TFHealthCheckNode : public rclcpp::Node
 {
    public:
        TFHealthCheckNode() : Node("tf_health_check")
        {
            // Parameters
            this->declare_parameter<std::string>("base_frame", "robot_base_footprint");
            this->declare_parameter<double>("check_frequency", 1.0);
            this->declare_parameter<double>("timeout", 1.0);

            base_frame_ = this->get_parameter("base_frame").as_string();
            check_frequency_ = this->get_parameter("check_frequency").as_double();
            timeout_ = this->get_parameter("timeout").as_double();

            RCLCPP_INFO(this->get_logger(),
                        "TF Health Check Starting - monitoring transform for base_frame: %s",
                    base_frame_.c_str());

            // TF2 Setup
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Publisher for diagnostics
            diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

            // Timer for periodic health checks
            auto timer_period = std::chrono::duration<double>(1.0 / check_frequency_);
            timer_ = this->create_wall_timer(
                timer_period, std::bind(&TFHealthCheckNode::checkTFHealth, this)
            );

            // Initialize diagnostic message
            diagnostics_msg_.header.frame_id = "";
            diagnostics_msg_.status.resize(2);  // Two transforms to check
            
            // Initialize status messages
            diagnostics_msg_.status[0].name = "EKF Transform (odom -> " + base_frame_ + ")";
            diagnostics_msg_.status[0].hardware_id = "atlas_state_estimation";
            
            diagnostics_msg_.status[1].name = "SLAM Transform (map -> odom)";
            diagnostics_msg_.status[1].hardware_id = "atlas_state_estimation";
            
            RCLCPP_INFO(this->get_logger(), "TF Health Check node initialized successfully");
        }

    private:
    void checkTFHealth()
    {
        // Update timestamp
        diagnostics_msg_.header.stamp = this->get_clock()->now();
        
        // Check odom -> base_frame transform (required for navigation)
        checkTransform("robot_odom", base_frame_, 0, true);
        
        // Check map -> odom transform (optional, provided by SLAM)
        checkTransform("map", "robot_odom", 1, false);
        
        // Publish diagnostics
        diagnostics_pub_->publish(diagnostics_msg_);
    }
    
    void checkTransform(const std::string& from_frame, 
                       const std::string& to_frame, 
                       size_t status_index,
                       bool required)
    {
        auto& status = diagnostics_msg_.status[status_index];
        status.values.clear();  // Clear previous values
        
        try
        {
            // Try to lookup the transform
            auto transform = tf_buffer_->lookupTransform(
                from_frame, to_frame, tf2::TimePointZero, 
                tf2::durationFromSec(timeout_));
            
            // Transform exists - success!
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Transform available";
            
            // Add transform details
            addKeyValue(status, "from_frame", from_frame);
            addKeyValue(status, "to_frame", to_frame);
            addKeyValue(status, "translation_x", std::to_string(transform.transform.translation.x));
            addKeyValue(status, "translation_y", std::to_string(transform.transform.translation.y));
            addKeyValue(status, "translation_z", std::to_string(transform.transform.translation.z));
            
            // Log success only on state changes to avoid spam
            std::string transform_name = from_frame + " -> " + to_frame;
            if (failed_transforms_.count(transform_name))
            {
                RCLCPP_INFO(this->get_logger(), 
                           "✓ Transform %s is now available", transform_name.c_str());
                failed_transforms_.erase(transform_name);
            }
        }
        catch (const tf2::TransformException& ex)
        {
            // Transform lookup failed
            std::string transform_name = from_frame + " -> " + to_frame;
            
            if (required)
            {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = "Required transform missing: " + std::string(ex.what());
                
                // Log error with actionable advice (only once per failure)
                if (!failed_transforms_.count(transform_name))
                {
                    RCLCPP_ERROR(this->get_logger(), 
                                "✗ Required transform %s missing: %s", 
                                transform_name.c_str(), ex.what());
                    RCLCPP_ERROR(this->get_logger(), 
                                "  → Check if robot_localization EKF node is running");
                    RCLCPP_ERROR(this->get_logger(), 
                                "  → Verify EKF configuration and sensor topics");
                    failed_transforms_.insert(transform_name);
                }
            }
            else
            {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "Optional transform missing: " + std::string(ex.what());
                
                // Log warning with advice (only once per failure)
                if (!failed_transforms_.count(transform_name))
                {
                    RCLCPP_WARN(this->get_logger(), 
                               "⚠ Optional transform %s missing: %s", 
                               transform_name.c_str(), ex.what());
                    RCLCPP_WARN(this->get_logger(), 
                               "  → This is expected if SLAM is not yet running");
                    RCLCPP_WARN(this->get_logger(), 
                               "  → SLAM will provide map -> odom transform later");
                    failed_transforms_.insert(transform_name);
                }
            }
            
            // Add error details
            addKeyValue(status, "from_frame", from_frame);
            addKeyValue(status, "to_frame", to_frame);
            addKeyValue(status, "error", ex.what());
        }
    }
    
    void addKeyValue(diagnostic_msgs::msg::DiagnosticStatus& status, 
                    const std::string& key, 
                    const std::string& value)
    {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    }
    
    // Node parameters
    std::string base_frame_;
    double check_frequency_;
    double timeout_;
    
    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ROS components
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State tracking
    diagnostic_msgs::msg::DiagnosticArray diagnostics_msg_;
    std::set<std::string> failed_transforms_;  // Track failed transforms to avoid spam
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFHealthCheckNode>();
    
    RCLCPP_INFO(node->get_logger(), "TF Health Check node spinning...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}