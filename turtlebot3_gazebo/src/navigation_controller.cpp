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

#include "turtlebot3_gazebo/navigation_controller.hpp"

namespace turtlebot3_gazebo {

NavigationController::NavigationController() {
    state_machine_ = new StateMachine();
    
    // Initialize with zero values
    current_sensor_data_.forward_distance = 0.0;
    current_sensor_data_.left_distance = 0.0;
    current_sensor_data_.right_distance = 0.0;
    
    current_pose_.x = 0.0;
    current_pose_.y = 0.0;
    current_pose_.yaw = 0.0;
    
    current_command_.linear = 0.0;
    current_command_.angular = 0.0;
}

NavigationController::~NavigationController() {
    delete state_machine_;
}

void NavigationController::update(
    const SensorData& data,
    const RobotPose& pose)
{
    // Store latest data
    current_sensor_data_ = data;
    current_pose_ = pose;
    
    // Process navigation logic
    processNavigation();
}

void NavigationController::processNavigation() {
    // Check for state transitions
    state_machine_->shouldTransition(
        current_sensor_data_,
        current_pose_);
    
    // Get command for current state (with sensor data for wall-following)
    current_command_ = state_machine_->getStateCommand(current_sensor_data_);
}

MotionCommand NavigationController::getMotionCommand() const {
    return current_command_;
}

void NavigationController::setUseCentering(bool enable) {
    state_machine_->setUseCentering(enable);
}

bool NavigationController::getUseCentering() const {
    return state_machine_->getUseCentering();
}

}  // namespace turtlebot3_gazebo
