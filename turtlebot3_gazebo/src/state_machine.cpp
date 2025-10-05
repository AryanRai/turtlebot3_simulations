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
            if (data.forward_distance < forward_threshold_) {
                // Forward blocked, turn left (away from right wall)
                prev_pose_ = pose.yaw;
                current_state_ = RobotState::TURN_LEFT;
                state_changed = true;
            } else {
                // Forward is clear, just drive and adjust based on wall distance
                current_state_ = RobotState::DRIVE_FORWARD;
                state_changed = true;
            }
            break;
            
        case RobotState::DRIVE_FORWARD:
            // Check if we need to stop and turn sharply
            if (data.forward_distance < forward_threshold_) {
                // Obstacle ahead, need to turn
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
            }
            // Otherwise keep driving forward (with angular adjustment in getStateCommand)
            break;
            
        case RobotState::TURN_RIGHT:
        case RobotState::TURN_LEFT:
            // Turn until forward is clear OR we've turned enough
            double angle_turned = std::fabs(prev_pose_ - pose.yaw);
            // Handle angle wrapping around ±π
            if (angle_turned > M_PI) {
                angle_turned = 2 * M_PI - angle_turned;
            }
            
            if (data.forward_distance > forward_threshold_ && angle_turned >= escape_range_ * 0.5) {
                // Forward is clear and we've turned at least 15°, go back to driving
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
            } else if (angle_turned >= escape_range_) {
                // Turned full 30°, check situation again
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
            // Rotate clockwise (slower to avoid overshooting)
            cmd.linear = 0.0;
            cmd.angular = -0.8;
            break;
            
        case RobotState::TURN_LEFT:
            // Rotate counter-clockwise (slower to avoid overshooting)
            cmd.linear = 0.0;
            cmd.angular = 0.8;
            break;
    }
    
    return cmd;
}

}  // namespace turtlebot3_gazebo
