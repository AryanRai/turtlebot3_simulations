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
#include <iostream>

namespace turtlebot3_gazebo {

// Navigation constants
const double ESCAPE_ANGLE_DEG = 30.0;          // Gentle turn angle (degrees)
const double SHARP_TURN_ANGLE_DEG = 90.0;      // Sharp turn angle (degrees)
const double FORWARD_THRESHOLD = 0.7;          // Minimum forward clearance (m)
const double SIDE_THRESHOLD = 0.6;             // Target wall distance (m)
const double DEFAULT_CORRIDOR_WIDTH = 2.0;     // Default corridor width (m)

// Collision detection constants
const double TILT_THRESHOLD_DEG = 5.0;         // Tilt threshold for collision (degrees)
const double STABLE_THRESHOLD_DEG = 2.0;       // Tilt threshold for stability (degrees)
const double MIN_MOVEMENT_THRESHOLD = 0.001;   // Minimum movement to detect stuck (m)
const int STUCK_COUNTER_THRESHOLD = 20;        // Cycles before declaring stuck
const double COLLISION_DISTANCE = 0.15;        // Distance threshold for collision (m)

// Turn detection constants
const double CORNER_THRESHOLD = 1.5;           // Distance for corner detection (m)
const double TURN_FORWARD_CLEARANCE = 0.5;     // Extra forward clearance for turns (m)
const double TURN_RIGHT_OPEN_THRESHOLD = 2.5;  // Right side opening threshold (m)
const double TURN_LEFT_WALL_THRESHOLD = 2.0;   // Left wall presence threshold (m)

// Wall following constants
const double WALL_DETECT_THRESHOLD = 3.5;      // Distance to consider wall present (m)
const double KP_SINGLE_WALL = 1.5;             // Proportional gain for single wall
const double KP_CENTERING = 0.3;               // Proportional gain for centering
const double MAX_ANGULAR_CORRECTION = 0.5;     // Maximum angular correction (rad/s)

// Velocity constants
const double LINEAR_VELOCITY_FORWARD = 0.3;    // Forward linear velocity (m/s)
const double ANGULAR_VELOCITY_GENTLE = 0.8;    // Gentle turn angular velocity (rad/s)
const double ANGULAR_VELOCITY_SHARP = 2.0;     // Sharp turn angular velocity (rad/s)
const double LINEAR_VELOCITY_RECOVERY = -0.1;  // Recovery backup velocity (m/s)

// Recovery constants
const int RECOVERY_BACKUP_CYCLES = 30;         // Cycles to back up during recovery
const int RECOVERY_MIN_CYCLES = 50;            // Minimum recovery cycles

// Adaptive turn constants
const double MIN_TURN_DISTANCE = 0.15;         // Minimum turn distance (m)
const double MAX_TURN_DISTANCE = 0.4;          // Maximum turn distance (m)
const double TURN_DISTANCE_FACTOR = 0.15;      // Turn distance as fraction of corridor width

StateMachine::StateMachine()
: current_state_(RobotState::GET_DIRECTION),
  prev_pose_(0.0),
  escape_range_(ESCAPE_ANGLE_DEG * DEG2RAD),
  sharp_turn_angle_(SHARP_TURN_ANGLE_DEG * DEG2RAD),
  forward_threshold_(FORWARD_THRESHOLD),
  side_threshold_(SIDE_THRESHOLD),
  prev_x_(0.0),
  prev_y_(0.0),
  stuck_counter_(0),
  recovery_counter_(0),
  opening_detected_(false),
  opening_start_x_(0.0),
  opening_start_y_(0.0),
  prev_right_distance_(0.0),
  corridor_width_(DEFAULT_CORRIDOR_WIDTH),
  use_centering_(false)  // Default to right-wall-follow (more reliable for maze solving)
{
}

void StateMachine::setUseCentering(const bool enable)
{
    use_centering_ = enable;
}

bool StateMachine::getUseCentering() const {
    return use_centering_;
}

double StateMachine::calculateAdaptiveTurnDistance(const double corridor_width) const
{
    // Calculate how far to drive into opening before turning
    // Based on corridor width for adaptive behavior
    
    // Calculate turn distance as a fraction of corridor width
    double turn_distance = corridor_width * TURN_DISTANCE_FACTOR;
    
    // Clamp to reasonable bounds
    if (turn_distance < MIN_TURN_DISTANCE) {
        turn_distance = MIN_TURN_DISTANCE;
    } else if (turn_distance > MAX_TURN_DISTANCE) {
        turn_distance = MAX_TURN_DISTANCE;
    }
    
    return turn_distance;
}

void StateMachine::setState(const RobotState state)
{
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
                    // First time detecting opening
                    opening_detected_ = true;
                    opening_start_x_ = pose.x;
                    opening_start_y_ = pose.y;
                    
                    // Measure current corridor width
                    corridor_width_ = data.left_distance + data.right_distance;
                    
                    // DEBUG OUTPUT
                    std::cout << "\n=== TURN OPPORTUNITY DETECTED ===" << std::endl;
                    std::cout << "Forward: " << data.forward_distance << "m" << std::endl;
                    std::cout << "Left: " << data.left_distance << "m" << std::endl;
                    std::cout << "Right: " << data.right_distance << "m" << std::endl;
                    std::cout << "Corridor width: " << corridor_width_ << "m" << std::endl;
                    std::cout << ">>> WILL DRIVE FORWARD BEFORE TURNING <<<" << std::endl;
                    std::cout << "================================\n" << std::endl;
                    
                    // Always drive forward a bit to position properly
                    // Don't turn immediately - this causes the robot to turn too early
                } else {
                    // Opening already detected - check if we've moved far enough
                    double distance_into_opening = std::sqrt(
                        std::pow(pose.x - opening_start_x_, 2) + 
                        std::pow(pose.y - opening_start_y_, 2)
                    );
                    
                    // Calculate adaptive turn distance - REDUCED for sharper turns
                    // Narrower corridors = turn sooner for tighter radius
                    // Wider corridors = can turn a bit later
                    double turn_distance = std::min(0.2, corridor_width_ * 0.12);  // REDUCED from 0.3 and 0.2
                    
                    std::cout << "[Driving into opening: " << distance_into_opening 
                             << "m / " << turn_distance << "m]" << std::endl;
                    
                    if (distance_into_opening > turn_distance) {
                        std::cout << "\n>>> DRIVEN " << distance_into_opening << "m, NOW TURNING RIGHT <<<\n" << std::endl;
                        // Execute sharp 90° right turn
                        prev_pose_ = pose.yaw;
                        current_state_ = RobotState::SHARP_TURN_RIGHT;
                        state_changed = true;
                        opening_detected_ = false;
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
            // Sharp 90° turn for corners - complete the FULL turn
            double angle_turned = std::fabs(prev_pose_ - pose.yaw);
            // Handle angle wrapping around ±π
            if (angle_turned > M_PI) {
                angle_turned = 2 * M_PI - angle_turned;
            }
            
            double angle_degrees = angle_turned * RAD2DEG;
            double target_degrees = sharp_turn_angle_ * RAD2DEG;
            
            // Log progress every 10 degrees
            static double last_logged = 0;
            if (angle_degrees - last_logged > 10.0) {
                std::cout << "[Turning: " << angle_degrees << "° / " << target_degrees << "°]" << std::endl;
                last_logged = angle_degrees;
            }
            
            // Complete the FULL 90° turn - increased threshold to ensure completion
            if (angle_turned >= sharp_turn_angle_ * 1.0) {
                // Turned full 90°
                std::cout << "\n>>> TURN COMPLETE: " << angle_degrees << "° <<<\n" << std::endl;
                current_state_ = RobotState::GET_DIRECTION;
                state_changed = true;
                last_logged = 0;
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
    // SIMPLIFIED: Detect right turn when forward is clear AND right is very open
    // This works better than trying to detect "wall ending" when already in open space
    
    bool forward_clear = data.forward_distance > forward_threshold_ + 0.5;  // Forward very clear
    bool right_very_open = data.right_distance > 2.5;  // Right side very open
    bool left_has_wall = data.left_distance < 2.0;  // Left wall present (confirms we're in corridor)
    
    // DEBUG: Log detection attempts
    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) {  // Every 50 cycles (0.5 seconds)
        std::cout << "[Right Turn Check] F:" << data.forward_distance 
                 << " L:" << data.left_distance
                 << " R:" << data.right_distance 
                 << " | FwdOK:" << (forward_clear ? "Y" : "N")
                 << " RightOpen:" << (right_very_open ? "Y" : "N")
                 << " LeftWall:" << (left_has_wall ? "Y" : "N") << std::endl;
    }
    
    // Trigger right turn if:
    // 1. Forward is very clear (can continue straight or turn)
    // 2. Right side is very open (turn opportunity)
    // 3. Left wall present (confirms we're following a corridor, not in open space)
    return forward_clear && right_very_open && left_has_wall;
}

double StateMachine::calculateWallFollowingCorrection(const SensorData& data) const {
    const double kp = 1.5;  // Proportional gain for single wall following
    const double kp_center = 0.3;  // Much lower gain for centering to avoid oscillation
    const double max_correction = 0.5;  // Maximum angular correction (rad/s)
    const double wall_detect_threshold = 3.5;  // Distance to consider a wall present
    
    double correction = 0.0;
    
    // CRITICAL: If we're in the middle of detecting/executing a turn, don't apply centering
    // This prevents the robot from turning the wrong way when right wall disappears
    if (opening_detected_) {
        // Turn in progress - drive straight, no correction
        return 0.0;
    }
    
    if (use_centering_) {
        // CENTERING MODE: Balance between walls when both present
        bool left_wall_present = data.left_distance < wall_detect_threshold;
        bool right_wall_present = data.right_distance < wall_detect_threshold;
        
        if (left_wall_present && right_wall_present) {
            // Both walls detected - center between them
            double balance_error = data.left_distance - data.right_distance;
            correction = kp_center * balance_error;
        } else if (right_wall_present && !left_wall_present) {
            // Only right wall - maintain distance from it
            double error = data.right_distance - side_threshold_;
            correction = -kp * error;
        } else if (left_wall_present && !right_wall_present) {
            // Only left wall present - this is tricky
            // In centering mode, we should NOT try to turn right to "balance"
            // Instead, just maintain distance from left wall (turn left if too close)
            // But ONLY if we're too close - otherwise drive straight
            if (data.left_distance < side_threshold_ * 0.8) {
                // Too close to left wall - turn away (right)
                double error = data.left_distance - side_threshold_;
                correction = kp * error * 0.5;  // Reduced gain
            } else {
                // Left wall at safe distance - drive straight
                correction = 0.0;
            }
        }
        // If no walls detected, correction stays 0 (drive straight)
    } else {
        // RIGHT WALL FOLLOW MODE: Only follow right wall (original behavior)
        if (data.right_distance < wall_detect_threshold) {
            // Right wall present - maintain distance from it
            double error = data.right_distance - side_threshold_;
            correction = -kp * error;
        } else {
            // No right wall detected - turn right to search for wall
            // This helps the robot find and follow walls in open environments
            correction = -0.3;  // Gentle right turn to search for wall
        }
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
            // Sharp 90° left turn for corners (very aggressive rotation for tight turns)
            cmd.linear = 0.0;
            cmd.angular = 2.0;  // INCREASED from 1.5 for faster, tighter turns
            break;
            
        case RobotState::SHARP_TURN_RIGHT:
            // Sharp 90° right turn for corners (very aggressive rotation for tight turns)
            cmd.linear = 0.0;
            cmd.angular = -2.0;  // INCREASED from -1.5 for faster, tighter turns
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
