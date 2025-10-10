#include "turtlebot3_gazebo/path_visualizer.hpp"
#include <cmath>

PathVisualizer::PathVisualizer()
: Node("path_visualizer"),
  last_x_(0.0),
  last_y_(0.0),
  min_distance_(0.02)  // 2cm minimum distance between points
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    10,
    std::bind(&PathVisualizer::odom_callback, this, std::placeholders::_1));
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "robot_path",
    10);
  
  path_.header.frame_id = "odom";
  
  RCLCPP_INFO(this->get_logger(), "Path visualizer started - publishing to /robot_path");
}

void PathVisualizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  if (should_add_point(x, y)) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    
    path_.poses.push_back(pose);
    
    last_x_ = x;
    last_y_ = y;
    
    publish_path();
    
    if (path_.poses.size() % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "Path has %zu points", path_.poses.size());
    }
  }
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
