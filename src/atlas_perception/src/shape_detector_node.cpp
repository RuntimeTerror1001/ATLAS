#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include "atlas_perception/action/find_shape.hpp"

using namespace std::chrono_literals;
using FindShape = atlas_perception::action::FindShape;
using GoalHandleFindShape = rclcpp_action::ServerGoalHandle<FindShape>;

// Updated Detection struct to include color
struct Detection {
  std::string shape;
  std::string color;
  geometry_msgs::msg::PoseStamped pose;
  float confidence;
  int marker_id;
};

class ShapeDetectorNode : public rclcpp::Node {
public:
  ShapeDetectorNode() : Node("shape_detector"), marker_id_counter_(0) {
    // Declare general parameters
    declare_parameter("rgb_topic", "/atlas/front_camera/color/image");
    declare_parameter("depth_topic", "/atlas/front_camera/depth/image");
    declare_parameter("camera_info_topic", "/atlas/front_camera/color/camera_info");
    declare_parameter("depth_scale", 1.0);
    declare_parameter("morph_open_iters", 1);
    declare_parameter("min_contour_area_px", 400.0);
    declare_parameter("depth_min_m", 0.05);
    declare_parameter("depth_max_m", 10.0);
    declare_parameter("debug_markers", true);
    declare_parameter("publish_debug_images", false);
    declare_parameter("detection_timeout_s", 2.0);
    
    // Declare the color names you want to load from params.yaml
    declare_parameter("color_names", std::vector<std::string>{"blue", "purple"});

    // Get general parameters
    rgb_topic_ = get_parameter("rgb_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    depth_scale_ = get_parameter("depth_scale").as_double();
    morph_open_iters_ = get_parameter("morph_open_iters").as_int();
    min_contour_area_ = get_parameter("min_contour_area_px").as_double();
    depth_min_ = get_parameter("depth_min_m").as_double();
    depth_max_ = get_parameter("depth_max_m").as_double();
    debug_markers_ = get_parameter("debug_markers").as_bool();
    publish_debug_images_ = get_parameter("publish_debug_images").as_bool();
    detection_timeout_ = get_parameter("detection_timeout_s").as_double();

    // Dynamically load all declared color ranges
    auto color_names = get_parameter("color_names").as_string_array();
    for (const auto& color : color_names) {
        declare_parameter("color_ranges." + color + ".lower", std::vector<int64_t>{0,0,0});
        declare_parameter("color_ranges." + color + ".upper", std::vector<int64_t>{180,255,255});
        
        auto lower_vals = get_parameter("color_ranges." + color + ".lower").as_integer_array();
        auto upper_vals = get_parameter("color_ranges." + color + ".upper").as_integer_array();

        cv::Scalar lower_bound(lower_vals[0], lower_vals[1], lower_vals[2]);
        cv::Scalar upper_bound(upper_vals[0], upper_vals[1], upper_vals[2]);

        color_ranges_[color] = {lower_bound, upper_bound};
        RCLCPP_INFO(get_logger(), "Loaded color '%s'", color.c_str());
    }

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    auto qos = rclcpp::SensorDataQoS();
    rgb_sub_.subscribe(this, rgb_topic_, qos.get_rmw_qos_profile());
    depth_sub_.subscribe(this, depth_topic_, qos.get_rmw_qos_profile());
    camera_info_sub_.subscribe(this, camera_info_topic_, qos.get_rmw_qos_profile());

    // Synchronizer
    using ApproxPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(
      ApproxPolicy(10), rgb_sub_, depth_sub_, camera_info_sub_);
    sync_->registerCallback(
      std::bind(&ShapeDetectorNode::imageCallback, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

    // Publishers
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/atlas_perception/detections", 10);
    if (publish_debug_images_) {
      debug_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/atlas_perception/debug/mask", 10);
      debug_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/atlas_perception/debug/overlay", 10);
    }

    // Action server
    action_server_ = rclcpp_action::create_server<FindShape>(
      this, "/atlas_perception/find_shape",
      std::bind(&ShapeDetectorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ShapeDetectorNode::handleCancel, this, std::placeholders::_1),
      std::bind(&ShapeDetectorNode::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Shape detector node initialized");
  }

private:
  // imageCallback, handleGoal, handleCancel, handleAccepted, execute...
  // ...These functions do not need to be changed...

  // --- OMITTING UNCHANGED FUNCTIONS FOR BREVITY ---
  // (imageCallback, handleGoal, handleCancel, handleAccepted, and execute are the same as your original file)
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_rgb_ = rgb_msg;
    last_depth_ = depth_msg;
    last_camera_info_ = info_msg;
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FindShape::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal: shape_type='%s', frame_id_out='%s'",
                goal->shape_type.c_str(), goal->frame_id_out.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleFindShape> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleFindShape> goal_handle) {
    std::thread{std::bind(&ShapeDetectorNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFindShape> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FindShape::Feedback>();
    auto result = std::make_shared<FindShape::Result>();

    // Wait for data with timeout
    auto start_time = now();
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (last_rgb_ && last_depth_ && last_camera_info_) {
          rgb = last_rgb_;
          depth = last_depth_;
          info = last_camera_info_;
          break;
        }
      }

      if ((now() - start_time).seconds() > detection_timeout_) {
        result->detected_shape = "none";
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "Timeout waiting for camera data");
        return;
      }
      
      feedback->status_text = "Waiting for camera data...";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(100ms);
    }

    feedback->status_text = "Processing images...";
    goal_handle->publish_feedback(feedback);

    // Process images
    auto detection = detectShape(rgb, depth, info, goal->shape_type);

    if (!detection.has_value()) {
      result->detected_shape = "none";
      goal_handle->abort(result);
      RCLCPP_INFO(get_logger(), "No matching shape detected");
      return;
    }

    // Transform if needed
    auto det = detection.value();
    if (!goal->frame_id_out.empty() && 
        goal->frame_id_out != det.pose.header.frame_id) {
      try {
        auto transform = tf_buffer_->lookupTransform(
          goal->frame_id_out, det.pose.header.frame_id, 
          tf2::TimePointZero, tf2::durationFromSec(1.0));
        tf2::doTransform(det.pose, det.pose, transform);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Transform failed: %s", ex.what());
        goal_handle->abort(result);
        return;
      }
    }

    // Publish marker
    if (debug_markers_) {
      publishMarker(det);
    }

    // Return result
    result->object_pose = det.pose;
    result->detected_shape = det.shape;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded: shape='%s', color='%s'",
                det.shape.c_str(), det.color.c_str());
  }


  // --- UPDATED detectShape FUNCTION ---
  std::optional<Detection> detectShape(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
    const std::string& requested_shape) {
    
    cv_bridge::CvImageConstPtr rgb_ptr, depth_ptr;
    try {
      rgb_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
      if (depth_msg->encoding == "16UC1") {
        depth_ptr = cv_bridge::toCvShare(depth_msg, "16UC1");
      } else {
        depth_ptr = cv_bridge::toCvShare(depth_msg, "32FC1");
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return std::nullopt;
    }

    cv::Mat rgb = rgb_ptr->image;
    cv::Mat depth = depth_ptr->image;
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

    double fx = info_msg->k[0];
    double fy = info_msg->k[4];
    double cx = info_msg->k[2];
    double cy = info_msg->k[5];

    // Iterate through all configured colors
    for (const auto& [color_name, ranges] : color_ranges_) {
      cv::Mat mask;
      cv::inRange(hsv, ranges.first, ranges.second, mask);

      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), morph_open_iters_);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);
      
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // Process contours for the current color
      for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_contour_area_) continue;

        std::vector<cv::Point> approx;
        double epsilon = 0.04 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);

        std::string shape = (approx.size() == 4) ? "square" : "cylinder";

        if (requested_shape != "any" && requested_shape != shape) {
          continue;
        }

        cv::Moments m = cv::moments(contour);
        if (m.m00 == 0) continue;
        int u = static_cast<int>(m.m10 / m.m00);
        int v = static_cast<int>(m.m01 / m.m00);

        if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) continue;

        double depth_val = (depth.type() == CV_16UC1) ? 
                           (depth.at<uint16_t>(v, u) * depth_scale_) : 
                           (depth.at<float>(v, u) * depth_scale_);

        if (depth_val < depth_min_ || depth_val > depth_max_ || std::isnan(depth_val) || std::isinf(depth_val)) {
          continue;
        }

        double X = (u - cx) * depth_val / fx;
        double Y = (v - cy) * depth_val / fy;
        double Z = depth_val;

        // Create and return the first valid detection
        Detection det;
        det.shape = shape;
        det.color = color_name;
        det.pose.header = rgb_msg->header;
        det.pose.pose.position.x = X;
        det.pose.pose.position.y = Y;
        det.pose.pose.position.z = Z;
        det.pose.pose.orientation.w = 1.0;
        det.confidence = std::min(1.0, area / 10000.0);
        det.marker_id = marker_id_counter_++;

        return det;
      }
    }
    
    // If we loop through all colors and find nothing, return null
    return std::nullopt;
  }
  
  // --- UPDATED publishMarker FUNCTION ---
  void publishMarker(const Detection& det) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    
    marker.header = det.pose.header;
    marker.ns = "detections";
    marker.id = det.marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = det.pose.pose;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    // Dynamically set marker color based on detection
    if (det.color == "blue") {
      marker.color.r = 0.1f; marker.color.g = 0.1f; marker.color.b = 1.0f;
    } else if (det.color == "purple") {
      marker.color.r = 0.5f; marker.color.g = 0.0f; marker.color.b = 0.5f;
    } else { // Default green for other colors
      marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
    }
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(5, 0);

    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }

  // Parameters
  std::string rgb_topic_, depth_topic_, camera_info_topic_;
  double depth_scale_;
  int morph_open_iters_;
  double min_contour_area_;
  double depth_min_, depth_max_;
  bool debug_markers_, publish_debug_images_;
  double detection_timeout_;
  
  // A map to hold all color ranges, keyed by color name
  std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_ranges_;

  // ROS interfaces
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>> sync_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_overlay_pub_;

  rclcpp_action::Server<FindShape>::SharedPtr action_server_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Data
  std::mutex data_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr last_rgb_;
  sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
  
  int marker_id_counter_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShapeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}