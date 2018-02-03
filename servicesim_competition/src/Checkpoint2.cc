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

#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <gazebo/common/Console.hh>

#include "Checkpoint2.hh"

using namespace servicesim;

/////////////////////////////////////////////////
Checkpoint2::Checkpoint2(const sdf::ElementPtr &_sdf,
    const unsigned int _number) : Checkpoint(_sdf, _number)
{
  // ROS transport
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode.reset(new ros::NodeHandle());

  this->pickUpRosService = this->rosNode->advertiseService(
      "/servicesim/pickup_guest", &Checkpoint2::OnPickUpRosRequest, this);
}

/////////////////////////////////////////////////
bool Checkpoint2::Check()
{
  return this->done;
}

/////////////////////////////////////////////////
bool Checkpoint2::OnPickUpRosRequest(
    servicesim_competition::PickUpGuest::Request &_req,
    servicesim_competition::PickUpGuest::Response &_res)
{
  // Get info from ROS request
  auto guestName = _req.guest_name;
  auto robotName = _req.robot_name;

  // Send Ignition request to actor
  auto ignService = "/servicesim/" + guestName + "/follow";

  ignition::msgs::StringMsg ignReq;
  ignReq.set_data(robotName);
  ignition::msgs::Boolean rep;
  bool result{false};

  // Send synchronous request so we can tell the robot if it succeeded
  bool executed = this->ignNode.Request(ignService, ignReq, 500, rep, result);
  if (!executed)
    gzerr << "Follow request timed out" << std::endl;

  this->done = result && rep.data();

  if (!this->done)
  {
    // TODO: apply penalty for bad pickup request
  }

  _res.success = this->done;

  return true;
}

