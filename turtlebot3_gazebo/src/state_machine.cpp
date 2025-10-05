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
  sharp_turn_angle_(90.0 * DEG2RAD),
  forward_threshold_(0.7),
  side_threshold_(0.6),
  prev_x_(0.0),
  prev_y_(0.0),
  stuck_counter_(0),
  recovery_counter_(0)
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
    
    // Check for collision/wobble in any state (except recovery)
    if (current_state_ != RobotState::RECOVERY && isCollisionDetected(data, pose)) {
        current_state_ = RobotState::RECOVERY;
        recovery_counter_ = 0;
        stuck_counter_ = 0;
        return true;
    }
    
    switch (current_state_) {
        case RobotState::GET_DIRECTION:
            // Analyze sensor data and decide next action
            if (data.forward_distance < forward_threshold_) {
                // Forward blocked - check if it's a corner or just an obstacle
                if (isCornerDetected(data)) {
                    // Corner detected: right wall ends, front blocked
                    // Execute sharp 90° left turn
                    prev_pose_ = pose.yaw;
                    current_state_ = RobotState::SHARP_TURN_LEFT;
                    state_changed = true;
                } else {
                    // Just an obstacle, gentle turn
                    prev_pose_ = pose.yaw;
                    current_state_ = RobotState::TURN_LEFT;
                    state_changed = true;
                }
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
            // Otherwise keep driving forward
            break;
            
        case RobotState::TURN_RIGHT:
        case RobotState::TURN_LEFT: {
            // Gentle turn until forward is clear OR we've turned enough
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
            
        case RobotState::SHARP_TURN_LEFT:
        case RobotState::SHARP_TURN_RIGHT: {
            // Sharp 90° turn for corners
            double angle_turned = std::fabs(prev_pose_ - pose.yaw);
            // Handle angle wrapping around ±π
            if (angle_turned > M_PI) {
                angle_turned = 2 * M_PI - angle_turned;
            }
            
            if (angle_turned >= sharp_turn_angle_ * 0.95) {
                // Turned ~90°, check situation
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
            } else if (data.forward_distance > forward_threshold_ + 0.3 && angle_turned >= sharp_turn_angle_ * 0.7) {
                // Forward is very clear and we've turned at least 63°, good enough
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
            }
            break;
        }
            
        case RobotState::RECOVERY:
            recovery_counter_++;
            // Stay in recovery for at least 50 cycles (0.5 seconds) and until stable
            if (recovery_counter_ > 50 && isStable(pose)) {
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
                stuck_counter_ = 0;
            }
            break;
    }
    
    // Update position tracking
    prev_x_ = pose.x;
    prev_y_ = pose.y;
    
    return state_changed;
}

bool StateMachine::isCollisionDetected(const SensorData& data, const RobotPose& pose) {
    // Check if robot is tilted (collision with wall)
    const double tilt_threshold = 5.0 * DEG2RAD;  // 5 degrees
    if (std::fabs(pose.roll) > tilt_threshold || std::fabs(pose.pitch) > tilt_threshold) {
        return true;
    }
    
    // Check if robot is stuck (not moving despite commands)
    double distance_moved = std::sqrt(
        std::pow(pose.x - prev_x_, 2) + std::pow(pose.y - prev_y_, 2)
    );
    
    if (current_state_ == RobotState::DRIVE_FORWARD && distance_moved < 0.001) {
        stuck_counter_++;
        if (stuck_counter_ > 20) {  // Stuck for 0.2 seconds
            return true;
        }
    } else {
        stuck_counter_ = 0;
    }
    
    // Check if any sensor reads extremely close (< 0.15m = collision)
    if (data.forward_distance < 0.15 || data.left_distance < 0.15 || data.right_distance < 0.15) {
        return true;
    }
    
    return false;
}

bool StateMachine::isStable(const RobotPose& pose) const {
    // Robot is stable if tilt is minimal
    const double stable_threshold = 2.0 * DEG2RAD;  // 2 degrees
    return (std::fabs(pose.roll) < stable_threshold && 
            std::fabs(pose.pitch) < stable_threshold);
}

bool StateMachine::isCornerDetected(const SensorData& data) const {
    // Corner detection: front blocked AND right wall suddenly disappears
    // This indicates we've reached the end of a wall (corner)
    const double corner_threshold = 1.5;  // Right wall far away
    
    return (data.forward_distance < forward_threshold_ && 
            data.right_distance > corner_threshold);
}

double StateMachine::calculateWallFollowingCorrection(const SensorData& data) const {
    // Proportional control to maintain constant distance from right wall
    // Target distance is side_threshold_ (0.6m)
    
    const double kp = 1.5;  // Proportional gain
    const double max_correction = 0.5;  // Maximum angular correction (rad/s)
    
    // Calculate error: positive = too far, negative = too close
    double error = data.right_distance - side_threshold_;
    
    // Apply proportional control
    double correction = kp * error;
    
    // Clamp correction to avoid excessive turning
    if (correction > max_correction) {
        correction = max_correction;
    } else if (correction < -max_correction) {
        correction = -max_correction;
    }
    
    // Positive correction = turn right (toward wall)
    // Negative correction = turn left (away from wall)
    return -correction;  // Negate because positive angular = left turn
}

MotionCommand StateMachine::getStateCommand(const SensorData& data) const {
    MotionCommand cmd;
    
    switch (current_state_) {
        case RobotState::GET_DIRECTION:
            // Stop and analyze
            cmd.linear = 0.0;
            cmd.angular = 0.0;
            break;
            
        case RobotState::DRIVE_FORWARD: {
            // Move forward with wall-following correction
            cmd.linear = 0.3;
            
            // Apply proportional control to maintain parallel distance
            if (data.right_distance < 2.0) {  // Only correct if wall is detected
                cmd.angular = calculateWallFollowingCorrection(data);
            } else {
                // No wall detected, drive straight
                cmd.angular = 0.0;
            }
            break;
        }
            
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
            
        case RobotState::SHARP_TURN_LEFT:
            // Sharp 90° left turn for corners (faster rotation)
            cmd.linear = 0.0;
            cmd.angular = 1.2;
            break;
            
        case RobotState::SHARP_TURN_RIGHT:
            // Sharp 90° right turn for corners (faster rotation)
            cmd.linear = 0.0;
            cmd.angular = -1.2;
            break;
            
        case RobotState::RECOVERY:
            // Back up slowly to disengage from wall
            if (recovery_counter_ < 30) {
                // First 0.3 seconds: back up
                cmd.linear = -0.1;
                cmd.angular = 0.0;
            } else {
                // Then stop and wait for stabilization
                cmd.linear = 0.0;
                cmd.angular = 0.0;
            }
            break;
    }
    
    return cmd;
}

}  // namespace turtlebot3_gazebo
