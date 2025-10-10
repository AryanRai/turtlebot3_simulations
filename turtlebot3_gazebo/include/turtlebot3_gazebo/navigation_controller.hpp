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

#ifndef TURTLEBOT3_GAZEBO__NAVIGATION_CONTROLLER_HPP_
#define TURTLEBOT3_GAZEBO__NAVIGATION_CONTROLLER_HPP_

#include "turtlebot3_gazebo/common_types.hpp"
#include "turtlebot3_gazebo/state_machine.hpp"

namespace turtlebot3_gazebo {

/**
 * @brief High-level navigation coordinator
 * 
 * Coordinates between sensor data, robot pose, and state machine
 * to generate appropriate motion commands for navigation.
 */
class NavigationController {
public:
    /**
     * @brief Constructor
     */
    NavigationController();
    
    /**
     * @brief Destructor
     */
    ~NavigationController();
    
    /**
     * @brief Update with latest sensor data and pose
     * @param data Current sensor data
     * @param pose Current robot pose
     */
    void update(const SensorData& data, const RobotPose& pose);
    
    /**
     * @brief Get current motion command
     * @return Motion command for robot control
     */
    MotionCommand getMotionCommand() const;
    
    /**
     * @brief Enable or disable centering mode
     * @param enable True for centering mode, False for right-wall-only mode
     */
    void setUseCentering(const bool enable);
    
    /**
     * @brief Get current centering mode status
     * @return True if centering enabled, False if right-wall-only
     */
    bool getUseCentering() const;
    
private:
    StateMachine* state_machine_;       // State management
    SensorData current_sensor_data_;    // Latest sensor readings
    RobotPose current_pose_;            // Latest robot pose
    MotionCommand current_command_;     // Current motion command
    
    /**
     * @brief Process navigation logic
     */
    void processNavigation();
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__NAVIGATION_CONTROLLER_HPP_