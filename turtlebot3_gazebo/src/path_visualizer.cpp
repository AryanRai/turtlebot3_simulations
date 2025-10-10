#include "turtlebot3_gazebo/path_visualizer.hpp"
#include <cmath>

PathVisualizer::PathVisualizer()
: Node("path_visualizer"),
  last_x_(0.0),
  last_y_(0.0),
  min_distance_(0.02),  // 2cm minimum distance between points
  recording_enabled_(false)  // Start with recording DISABLED
{
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "robot_path",
    10);
  
  // Create services for control
  toggle_service_ = this->create_service<std_srvs::srv::SetBool>(
    "path_visualizer/toggle_recording",
    std::bind(&PathVisualizer::toggle_recording_callback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  clear_service_ = this->create_service<std_srvs::srv::Empty>(
    "path_visualizer/clear_path",
    std::bind(&PathVisualizer::clear_path_callback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // Use map frame for SLAM-corrected positions
  path_.header.frame_id = "map";
  
  // Timer to check for new positions at 10Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PathVisualizer::timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "Path visualizer started - recording DISABLED");
  RCLCPP_INFO(this->get_logger(), "Enable with: ros2 service call /path_visualizer/toggle_recording std_srvs/srv/SetBool \"{data: true}\"");
  RCLCPP_INFO(this->get_logger(), "Clear path with: ros2 service call /path_visualizer/clear_path std_srvs/srv/Empty");
}

void PathVisualizer::timer_callback()
{
  // Only record if enabled
  if (!recording_enabled_) {
    return;
  }
  
  geometry_msgs::msg::TransformStamped transform;
  
  try {
    // Look up the transform from map to base_footprint
    transform = tf_buffer_->lookupTransform(
      "map", "base_footprint",
      tf2::TimePointZero);
    
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    
    if (should_add_point(x, y)) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      
      path_.poses.push_back(pose);
      
      last_x_ = x;
      last_y_ = y;
      
      publish_path();
      
      if (path_.poses.size() % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Path has %zu points", path_.poses.size());
      }
    }
  } catch (tf2::TransformException &ex) {
    // Silently ignore - SLAM might not be ready yet
    return;
  }
}

void PathVisualizer::toggle_recording_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  recording_enabled_ = request->data;
  response->success = true;
  
  if (recording_enabled_) {
    RCLCPP_INFO(this->get_logger(), "✅ Path recording ENABLED - now tracking robot path");
    response->message = "Path recording enabled";
  } else {
    RCLCPP_INFO(this->get_logger(), "⏸️  Path recording DISABLED");
    response->message = "Path recording disabled";
  }
}

void PathVisualizer::clear_path_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  path_.poses.clear();
  last_x_ = 0.0;
  last_y_ = 0.0;
  publish_path();  // Publish empty path to clear visualization
  RCLCPP_INFO(this->get_logger(), "🗑️  Path cleared");
}

bool PathVisualizer::should_add_point(double x, double y)
{
  double dx = x - last_x_;
  double dy = y - last_y_;
  double distance = std::sqrt(dx*dx + dy*dy);
  
  return distance > min_distance_ || path_.poses.empty();
}

void PathVisualizer::publish_path()
{
  path_.header.stamp = this->now();
  path_pub_->publish(path_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathVisualizer>());
  rclcpp::shutdown();
  return 0;
}
