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
  recovery_counter_(0),
  opening_detected_(false),
  opening_start_x_(0.0),
  opening_start_y_(0.0),
  prev_right_distance_(0.0),
  use_centering_(true)  // Default to centering mode
{
}

void StateMachine::setUseCentering(bool enable) {
    use_centering_ = enable;
}

bool StateMachine::getUseCentering() const {
    return use_centering_;
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
            
        case RobotState::DRIVE_FORWARD: {
            // Check for 90° right turn opportunity (right wall opens up)
            if (isRightTurnOpportunity(data)) {
                if (!opening_detected_) {
                    // First time detecting opening - mark position
                    opening_detected_ = true;
                    opening_start_x_ = pose.x;
                    opening_start_y_ = pose.y;
                } else {
                    // Opening already detected - check if we've moved far enough into it
                    double distance_into_opening = std::sqrt(
                        std::pow(pose.x - opening_start_x_, 2) + 
                        std::pow(pose.y - opening_start_y_, 2)
                    );
                    
                    // Turn only after moving ~0.3-0.4m into the opening (midway through)
                    if (distance_into_opening > 0.35) {
                        // Now execute sharp 90° right turn
                        prev_pose_ = pose.yaw;
                        current_state_ = RobotState::SHARP_TURN_RIGHT;
                        state_changed = true;
                        opening_detected_ = false;  // Reset for next opening
                    }
                }
            } else {
                // No opening detected, reset flag
                opening_detected_ = false;
                
                if (data.forward_distance < forward_threshold_) {
                    // Obstacle ahead, need to turn
                    current_state_ = RobotState::GET_DIRECTION;
                    state_changed = true;
                }
            }
            // Otherwise keep driving forward
            break;
        }
            
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
            // Sharp 90° turn for corners - complete the full turn precisely
            double angle_turned = std::fabs(prev_pose_ - pose.yaw);
            // Handle angle wrapping around ±π
            if (angle_turned > M_PI) {
                angle_turned = 2 * M_PI - angle_turned;
            }
            
            if (angle_turned >= sharp_turn_angle_ * 0.98) {
                // Turned ~88°+, very close to 90°
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
    
    // Update position and sensor tracking
    prev_x_ = pose.x;
    prev_y_ = pose.y;
    prev_right_distance_ = data.right_distance;
    
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

bool StateMachine::isRightTurnOpportunity(const SensorData& data) const {
    // Detect 90° right turn opportunity by looking for a significant increase in right distance
    // This indicates the right wall has ended (opening appeared)
    
    const double min_increase = 1.0;  // Right distance must increase by at least 1.0m
    const double min_absolute_distance = 2.5;  // And be at least 2.5m away
    
    // Check if right distance increased significantly from previous reading
    double distance_increase = data.right_distance - prev_right_distance_;
    
    // Only trigger if:
    // 1. Forward is clear
    // 2. Right distance increased significantly (wall ended)
    // 3. Right distance is now large (confirming opening)
    return (data.forward_distance > forward_threshold_ + 0.3 && 
            distance_increase > min_increase &&
            data.right_distance > min_absolute_distance);
}

double StateMachine::calculateWallFollowingCorrection(const SensorData& data) const {
    const double kp = 1.5;  // Proportional gain for single wall following
    const double kp_center = 0.3;  // Much lower gain for centering to avoid oscillation
    const double max_correction = 0.5;  // Maximum angular correction (rad/s)
    const double wall_detect_threshold = 3.5;  // Distance to consider a wall present
    
    double correction = 0.0;
    
    if (use_centering_) {
        // CENTERING MODE: Balance between walls when both present
        bool left_wall_present = data.left_distance < wall_detect_threshold;
        bool right_wall_present = data.right_distance < wall_detect_threshold;
        
        if (left_wall_present && right_wall_present) {
            // Both walls detected - center between them
            double balance_error = data.left_distance - data.right_distance;
            correction = kp_center * balance_error;
        } else if (right_wall_present) {
            // Only right wall - maintain distance from it
            double error = data.right_distance - side_threshold_;
            correction = -kp * error;
        } else if (left_wall_present) {
            // Only left wall - maintain distance from it
            double error = data.left_distance - side_threshold_;
            correction = kp * error;
        }
    } else {
        // RIGHT WALL FOLLOW MODE: Only follow right wall (original behavior)
        if (data.right_distance < wall_detect_threshold) {
            // Right wall present - maintain distance from it
            double error = data.right_distance - side_threshold_;
            correction = -kp * error;
        }
        // Ignore left wall, drive straight if no right wall
    }
    
    // Clamp correction to avoid excessive turning
    if (correction > max_correction) {
        correction = max_correction;
    } else if (correction < -max_correction) {
        correction = -max_correction;
    }
    
    return correction;
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
            // Move forward with wall-following/centering correction
            cmd.linear = 0.3;
            
            // Apply wall following or centering correction
            // The function handles all cases (both walls, one wall, or no walls)
            cmd.angular = calculateWallFollowingCorrection(data);
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
            // Sharp 90° left turn for corners (aggressive rotation)
            cmd.linear = 0.0;
            cmd.angular = 1.5;
            break;
            
        case RobotState::SHARP_TURN_RIGHT:
            // Sharp 90° right turn for corners (aggressive rotation)
            cmd.linear = 0.0;
            cmd.angular = -1.5;
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
