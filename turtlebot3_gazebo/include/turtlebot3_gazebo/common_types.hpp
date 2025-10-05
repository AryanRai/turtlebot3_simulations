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

#ifndef TURTLEBOT3_GAZEBO__COMMON_TYPES_HPP_
#define TURTLEBOT3_GAZEBO__COMMON_TYPES_HPP_

#include <cmath>

namespace turtlebot3_gazebo {

// Conversion constants
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// Sensor array indices
#define CENTER 0
#define LEFT   1
#define RIGHT  2

/**
 * @brief Sensor data extracted from LiDAR scan
 */
struct SensorData {
    double forward_distance;  // Distance at 0° (front)
    double left_distance;     // Distance at 30° (left-front)
    double right_distance;    // Distance at 330° (right-front)
};

/**
 * @brief Robot pose information
 */
struct RobotPose {
    double x;      // X position in meters
    double y;      // Y position in meters
    double yaw;    // Orientation in radians
    double roll;   // Roll angle in radians (tilt side-to-side)
    double pitch;  // Pitch angle in radians (tilt front-back)
};

/**
 * @brief Motion command for robot control
 */
struct MotionCommand {
    double linear;   // Linear velocity in m/s
    double angular;  // Angular velocity in rad/s
};

/**
 * @brief Robot navigation states
 */
enum class RobotState {
    GET_DIRECTION,     // Analyze sensors and decide next action
    DRIVE_FORWARD,     // Move forward at constant velocity
    TURN_RIGHT,        // Rotate clockwise (gentle)
    TURN_LEFT,         // Rotate counter-clockwise (gentle)
    SHARP_TURN_LEFT,   // Sharp 90° left turn for corners
    SHARP_TURN_RIGHT,  // Sharp 90° right turn for corners
    RECOVERY           // Recover from collision/wobble
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__COMMON_TYPES_HPP_