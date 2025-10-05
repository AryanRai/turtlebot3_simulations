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

#ifndef TURTLEBOT3_GAZEBO__SENSOR_PROCESSOR_HPP_
#define TURTLEBOT3_GAZEBO__SENSOR_PROCESSOR_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include "turtlebot3_gazebo/common_types.hpp"

namespace turtlebot3_gazebo {

/**
 * @brief Processes LiDAR sensor data for navigation
 * 
 * Extracts key angles from 360° LiDAR scan and provides
 * convenient methods for obstacle detection.
 */
class SensorProcessor {
public:
    /**
     * @brief Constructor with default thresholds
     */
    SensorProcessor();
    
    /**
     * @brief Process incoming LiDAR scan message
     * @param msg Shared pointer to LaserScan message
     */
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Get processed sensor data
     * @return SensorData struct with distances at key angles
     */
    SensorData getSensorData() const;
    
    /**
     * @brief Check if obstacle is ahead
     * @return true if forward distance < threshold
     */
    bool isObstacleAhead() const;
    
    /**
     * @brief Check if wall is on right side
     * @return true if right distance < threshold
     */
    bool isWallOnRight() const;
    
    /**
     * @brief Check if wall is on left side
     * @return true if left distance < threshold
     */
    bool isWallOnLeft() const;
    
private:
    double scan_data_[3];        // CENTER, LEFT, RIGHT distances
    double forward_threshold_;   // Minimum forward clearance (m)
    double side_threshold_;      // Target wall distance (m)
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__SENSOR_PROCESSOR_HPP_
