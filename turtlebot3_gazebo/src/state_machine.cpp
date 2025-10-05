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

#include "turtlebot3_gazebo/state_machine.hpp"
#include <cmath>

namespace turtlebot3_gazebo {

StateMachine::StateMachine()
: current_state_(RobotState::GET_DIRECTION),
  prev_pose_(0.0),
  escape_range_(30.0 * DEG2RAD),
  forward_threshold_(0.7),
  side_threshold_(0.6)
{
}

void StateMachine::setState(RobotState state) {
    current_state_ = state;
}

RobotState StateMachine::getState() const {
    return current_state_;
}

bool StateMachine::shouldTransition(
    const SensorData& data,
    const RobotPose& pose)
{
    bool state_changed = false;
    
    switch (current_state_) {
        case RobotState::GET_DIRECTION:
            // Analyze sensor data and decide next action
            if (data.forward_distance > forward_threshold_) {
                // Forward is clear
                if (data.left_distance < side_threshold_) {
                    // Left wall close, turn right to maintain right wall
                    prev_pose_ = pose.yaw;
                    current_state_ = RobotState::TURN_RIGHT;
                    state_changed = true;
                } else if (data.right_distance < side_threshold_) {
                    // Right wall close, turn left to avoid collision
                    prev_pose_ = pose.yaw;
                    current_state_ = RobotState::TURN_LEFT;
                    state_changed = true;
                } else {
                    // Path clear, drive forward
                    current_state_ = RobotState::DRIVE_FORWARD;
                    state_changed = true;
                }
            } else {
                // Forward blocked, turn right
                prev_pose_ = pose.yaw;
                current_state_ = RobotState::TURN_RIGHT;
                state_changed = true;
            }
            break;
            
        case RobotState::DRIVE_FORWARD:
            // Always return to GET_DIRECTION after moving
            current_state_ = RobotState::GET_DIRECTION;
            state_changed = true;
            break;
            
        case RobotState::TURN_RIGHT:
        case RobotState::TURN_LEFT:
            // Check if turn is complete (turned 30°)
            if (std::fabs(prev_pose_ - pose.yaw) >= escape_range_) {
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
            }
            break;
    }
    
    return state_changed;
}

MotionCommand StateMachine::getStateCommand() const {
    MotionCommand cmd;
    
    switch (current_state_) {
        case RobotState::GET_DIRECTION:
            // Stop and analyze
            cmd.linear = 0.0;
            cmd.angular = 0.0;
            break;
            
        case RobotState::DRIVE_FORWARD:
            // Move forward at constant velocity
            cmd.linear = 0.3;
            cmd.angular = 0.0;
            break;
            
        case RobotState::TURN_RIGHT:
            // Rotate clockwise
            cmd.linear = 0.0;
            cmd.angular = -1.5;
            break;
            
        case RobotState::TURN_LEFT:
            // Rotate counter-clockwise
            cmd.linear = 0.0;
            cmd.angular = 1.5;
            break;
    }
    
    return cmd;
}

}  // namespace turtlebot3_gazebo
