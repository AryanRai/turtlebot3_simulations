#ifndef TURTLEBOT3_GAZEBO__PATH_VISUALIZER_HPP_
#define TURTLEBOT3_GAZEBO__PATH_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathVisualizer : public rclcpp::Node
{
public:
    PathVisualizer();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool should_add_point(double x, double y);
    void publish_path();
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::Path path_;
    
    double last_x_;
    double last_y_;
    double min_distance_;
};

#endif  // TURTLEBOT3_GAZEBO__PATH_VISUALIZER_HPP_
