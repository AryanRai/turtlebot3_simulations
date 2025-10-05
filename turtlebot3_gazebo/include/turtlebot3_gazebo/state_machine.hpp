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

#ifndef TURTLEBOT3_GAZEBO__STATE_MACHINE_HPP_
#define TURTLEBOT3_GAZEBO__STATE_MACHINE_HPP_

#include "turtlebot3_gazebo/common_types.hpp"

namespace turtlebot3_gazebo {

/**
 * @brief Manages robot navigation states and transitions
 * 
 * Implements right wall following logic through state machine.
 * Handles state transitions based on sensor data and robot pose.
 */
class StateMachine {
public:
    /**
     * @brief Constructor with default parameters
     */
    StateMachine();
    
    /**
     * @brief Set current state
     * @param state New robot state
     */
    void setState(RobotState state);
    
    /**
     * @brief Get current state
     * @return Current robot state
     */
    RobotState getState() const;
    
    /**
     * @brief Check if state transition should occur and update state
     * @param data Current sensor data
     * @param pose Current robot pose
     * @return true if state changed
     */
    bool shouldTransition(const SensorData& data, const RobotPose& pose);
    
    /**
     * @brief Get motion command for current state
     * @param data Current sensor data for wall-following control
     * @return Motion command with velocities
     */
    MotionCommand getStateCommand(const SensorData& data) const;
    
    /**
     * @brief Enable or disable centering mode
     * @param enable True for centering mode, False for right-wall-only mode
     */
    void setUseCentering(bool enable);
    
    /**
     * @brief Get current centering mode status
     * @return True if centering enabled, False if right-wall-only
     */
    bool getUseCentering() const;
    
private:
    RobotState current_state_;   // Current navigation state
    double prev_pose_;           // Previous yaw for turn tracking
    double escape_range_;        // Turn angle in radians (30°)
    double sharp_turn_angle_;    // Sharp turn angle in radians (90°)
    
    // Thresholds for decision making
    double forward_threshold_;   // Minimum forward clearance (m)
    double side_threshold_;      // Target wall distance (m)
    
    // Collision detection
    double prev_x_;              // Previous X position
    double prev_y_;              // Previous Y position
    int stuck_counter_;          // Counter for detecting stuck state
    int recovery_counter_;       // Counter for recovery duration
    
    // Right turn opportunity tracking
    bool opening_detected_;      // Flag when opening is first detected
    double opening_start_x_;     // X position when opening detected
    double opening_start_y_;     // Y position when opening detected
    double prev_right_distance_; // Previous right distance for change detection
    double corridor_width_;      // Measured corridor width for adaptive turning
    
    // Driving mode configuration
    bool use_centering_;         // True = center between walls, False = right wall only
    
    // Helper functions
    bool isCollisionDetected(const SensorData& data, const RobotPose& pose);
    bool isStable(const RobotPose& pose) const;
    bool isCornerDetected(const SensorData& data) const;
    bool isRightTurnOpportunity(const SensorData& data) const;
    double calculateWallFollowingCorrection(const SensorData& data) const;
    double calculateAdaptiveTurnDistance(double corridor_width) const;
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__STATE_MACHINE_HPP_
