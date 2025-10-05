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

#include "turtlebot3_gazebo/turtlebot3_drive.hpp"
#include <memory>

using namespace std::chrono_literals;

namespace turtlebot3_gazebo {

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
    /************************************************************
    ** Initialize navigation components
    ************************************************************/
    nav_controller_ = new NavigationController();
    sensor_processor_ = new SensorProcessor();
    motion_controller_ = new MotionController();
    
    /************************************************************
    ** Initialize robot pose
    ************************************************************/
    robot_pose_.x = 0.0;
    robot_pose_.y = 0.0;
    robot_pose_.yaw = 0.0;
    robot_pose_.roll = 0.0;
    robot_pose_.pitch = 0.0;
    
    /************************************************************
    ** Declare and get ROS2 parameters
    ************************************************************/
    this->declare_parameter("use_centering", true);  // true = center, false = right wall only
    bool use_centering = this->get_parameter("use_centering").as_bool();
    nav_controller_->setUseCentering(use_centering);
    
    if (use_centering) {
        RCLCPP_INFO(this->get_logger(), "Driving Mode: CENTERING (balances between walls)");
    } else {
        RCLCPP_INFO(this->get_logger(), "Driving Mode: RIGHT WALL FOLLOW (original behavior)");
    }
    
    /************************************************************
    ** Initialize ROS2 publishers and subscribers
    ************************************************************/
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", qos);
    
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&Turtlebot3Drive::scan_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        qos,
        std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));
    
    /************************************************************
    ** Initialize ROS2 timer (100Hz control loop)
    ************************************************************/
    update_timer_ = this->create_wall_timer(
        10ms,
        std::bind(&Turtlebot3Drive::update_callback, this));
    
    RCLCPP_INFO(this->get_logger(), 
        "Turtlebot3 Drive Node initialized (Refactored OOP Architecture)");
}

Turtlebot3Drive::~Turtlebot3Drive()
{
    // Cleanup components
    delete nav_controller_;
    delete sensor_processor_;
    delete motion_controller_;
    
    RCLCPP_INFO(this->get_logger(), 
        "Turtlebot3 Drive Node terminated");
}

/********************************************************************************
** Callback functions for ROS2 subscribers
********************************************************************************/

void Turtlebot3Drive::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Delegate sensor processing to SensorProcessor
    sensor_processor_->processScan(msg);
}

void Turtlebot3Drive::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract roll, pitch, yaw angles from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Update robot pose with all orientation data
    robot_pose_.x = msg->pose.pose.position.x;
    robot_pose_.y = msg->pose.pose.position.y;
    robot_pose_.yaw = yaw;
    robot_pose_.roll = roll;
    robot_pose_.pitch = pitch;
}

/********************************************************************************
** Update function (main control loop)
********************************************************************************/

void Turtlebot3Drive::update_callback()
{
    // Get processed sensor data
    SensorData sensor_data = sensor_processor_->getSensorData();
    
    // Update navigation controller with latest data
    nav_controller_->update(sensor_data, robot_pose_);
    
    // Get motion command from navigation controller
    MotionCommand cmd = nav_controller_->getMotionCommand();
    
    // Create TwistStamped message and publish
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = this->now();
    twist_stamped.header.frame_id = "";
    twist_stamped.twist = motion_controller_->createTwistCommand(cmd);
    cmd_vel_pub_->publish(twist_stamped);
}

}  // namespace turtlebot3_gazebo

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<turtlebot3_gazebo::Turtlebot3Drive>());
    rclcpp::shutdown();
    return 0;
}
