// Copyright 2019 ROBOTIS CO., LTD.
// Copyright 2025 MTRX3760 Project Team (Refactored)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
// Refactored by: MTRX3760 Project Team

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "turtlebot3_gazebo/common_types.hpp"
#include "turtlebot3_gazebo/navigation_controller.hpp"
#include "turtlebot3_gazebo/sensor_processor.hpp"
#include "turtlebot3_gazebo/motion_controller.hpp"

namespace turtlebot3_gazebo {

/**
 * @brief Main ROS2 node for Turtlebot3 autonomous navigation
 * 
 * Orchestrates ROS2 communication and coordinates navigation components.
 * Implements right wall following algorithm through object-oriented design.
 */
class Turtlebot3Drive : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes node and components
     */
    Turtlebot3Drive();
    
    /**
     * @brief Destructor - cleanup
     */
    ~Turtlebot3Drive();

private:
    // Navigation components
    NavigationController* nav_controller_;
    SensorProcessor* sensor_processor_;
    MotionController* motion_controller_;
    
    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    // ROS2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Robot state
    RobotPose robot_pose_;

    // ROS2 timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Callback functions
    void update_callback();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
