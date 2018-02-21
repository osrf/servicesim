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

#include "Conversions.hh"

//////////////////////////////////////////////////
geometry_msgs::Pose servicesim::convert(
    const ignition::math::Pose3d &_pose)
{
  geometry_msgs::Pose msg;
  msg.position.x = _pose.Pos().X();
  msg.position.y = _pose.Pos().Y();
  msg.position.z = _pose.Pos().Z();
  msg.orientation.x = _pose.Rot().X();
  msg.orientation.y = _pose.Rot().Y();
  msg.orientation.z = _pose.Rot().Z();
  msg.orientation.w = _pose.Rot().W();
  return msg;
}

//////////////////////////////////////////////////
geometry_msgs::Point servicesim::convert(
    const ignition::math::Vector3d &_vec)
{
  geometry_msgs::Point msg;
  msg.x = _vec.X();
  msg.y = _vec.Y();
  msg.z = _vec.Z();
  return msg;
}

