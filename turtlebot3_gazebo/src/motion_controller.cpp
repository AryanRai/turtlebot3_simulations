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

#include "turtlebot3_gazebo/motion_controller.hpp"

namespace turtlebot3_gazebo {

MotionController::MotionController()
: linear_velocity_(0.3),
  angular_velocity_(1.5)
{
}

geometry_msgs::msg::Twist MotionController::createTwistCommand(
    const MotionCommand& cmd)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.linear;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = cmd.angular;
    return twist;
}

void MotionController::setVelocities(double linear, double angular) {
    linear_velocity_ = linear;
    angular_velocity_ = angular;
}

}  // namespace turtlebot3_gazebo
