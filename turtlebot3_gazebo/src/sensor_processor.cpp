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

#include "turtlebot3_gazebo/sensor_processor.hpp"
#include <cmath>

namespace turtlebot3_gazebo {

SensorProcessor::SensorProcessor()
: forward_threshold_(0.7),
  side_threshold_(0.6)
{
    scan_data_[CENTER] = 0.0;
    scan_data_[LEFT] = 0.0;
    scan_data_[RIGHT] = 0.0;
}

void SensorProcessor::processScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Extract distances at key angles: 0°, 30°, 330°
    uint16_t scan_angle[3] = {0, 30, 330};
    
    for (int num = 0; num < 3; num++) {
        // Handle infinite values by capping at max range
        if (std::isinf(msg->ranges.at(scan_angle[num]))) {
            scan_data_[num] = msg->range_max;
        } else {
            scan_data_[num] = msg->ranges.at(scan_angle[num]);
        }
    }
}

SensorData SensorProcessor::getSensorData() const {
    SensorData data;
    data.forward_distance = scan_data_[CENTER];
    data.left_distance = scan_data_[LEFT];
    data.right_distance = scan_data_[RIGHT];
    return data;
}

bool SensorProcessor::isObstacleAhead() const {
    return scan_data_[CENTER] < forward_threshold_;
}

bool SensorProcessor::isWallOnRight() const {
    return scan_data_[RIGHT] < side_threshold_;
}

bool SensorProcessor::isWallOnLeft() const {
    return scan_data_[LEFT] < side_threshold_;
}

}  // namespace turtlebot3_gazebo
