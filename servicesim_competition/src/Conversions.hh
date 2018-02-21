/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SERVICESIM_CONVERSIONS_HH_
#define SERVICESIM_CONVERSIONS_HH_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace servicesim
{
  /// \brief Return the equivalent ROS message
  /// \param[in] _vec Ignition pose 3d to convert.
  /// \return ROS geometry pose message
  geometry_msgs::Pose convert(const ignition::math::Pose3d &_pose);

  /// \brief Return the equivalent ROS message
  /// \param[in] _vec Ignition vector 3d to convert.
  /// \return ROS geometry pose message
  geometry_msgs::Point convert(const ignition::math::Vector3d &_vec);
}
#endif
