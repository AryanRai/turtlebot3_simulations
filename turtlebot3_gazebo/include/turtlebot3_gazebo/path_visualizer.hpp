#ifndef TURTLEBOT3_GAZEBO__PATH_VISUALIZER_HPP_
#define TURTLEBOT3_GAZEBO__PATH_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PathVisualizer : public rclcpp::Node
{
public:
    PathVisualizer();

private:
    void timer_callback();
    bool should_add_point(double x, double y);
    void publish_path();
    void toggle_recording_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void clear_path_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_service_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path path_;
    
    double last_x_;
    double last_y_;
    double min_distance_;
    bool recording_enabled_;
};

#endif  // TURTLEBOT3_GAZEBO__PATH_VISUALIZER_HPP_
