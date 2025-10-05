// Copyright 2025 MTRX3760 Project Team
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

#ifndef TURTLEBOT3_GAZEBO__MOTION_CONTROLLER_HPP_
#define TURTLEBOT3_GAZEBO__MOTION_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include "turtlebot3_gazebo/common_types.hpp"

namespace turtlebot3_gazebo {

/**
 * @brief Generates velocity commands for robot motion
 * 
 * Converts high-level motion commands into ROS Twist messages
 * and manages velocity parameters.
 */
class MotionController {
public:
    /**
     * @brief Constructor with default velocities
     */
    MotionController();
    
    /**
     * @brief Create Twist message from motion command
     * @param cmd Motion command with linear and angular velocities
     * @return Twist message ready for publishing
     */
    geometry_msgs::msg::Twist createTwistCommand(const MotionCommand& cmd);
    
    /**
     * @brief Set velocity parameters
     * @param linear Linear velocity in m/s
     * @param angular Angular velocity in rad/s
     */
    void setVelocities(double linear, double angular);
    
private:
    double linear_velocity_;   // Default linear velocity (m/s)
    double angular_velocity_;  // Default angular velocity (rad/s)
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__MOTION_CONTROLLER_HPP_
