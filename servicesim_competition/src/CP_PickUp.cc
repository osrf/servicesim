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
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>

#include "CP_PickUp.hh"

using namespace servicesim;

/////////////////////////////////////////////////
CP_PickUp::CP_PickUp(const sdf::ElementPtr &_sdf)
    : Checkpoint(_sdf)
{
  this->canPause = true;

  // Get SDF params
  if (!_sdf)
  {
    gzerr << "Missing checkpoint's SDF element" << std::endl;
    return;
  }

  if (!_sdf->HasElement("weight"))
  {
    gzerr << "Missing checkpoint's <weight> element" << std::endl;
    return;
  }

  auto weightElem = _sdf->GetElement("weight");
  if (!weightElem->HasElement("failed_attempt"))
  {
    gzerr << "Missing checkpoint's <weight><failed_attempt> element"
          << std::endl;
    return;
  }
  this->weightFailedAttempt = weightElem->Get<double>("failed_attempt");

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
      "/servicesim/pickup_guest", &CP_PickUp::OnPickUpRosRequest, this);
}

/////////////////////////////////////////////////
bool CP_PickUp::Check()
{
  return this->Done();
}

/////////////////////////////////////////////////
bool CP_PickUp::OnPickUpRosRequest(
    servicesim_competition::PickUpGuest::Request &_req,
    servicesim_competition::PickUpGuest::Response &_res)
{
  if (!this->Started())
  {
    gzmsg  << "[ServiceSim] " << this->weightFailedAttempt
           << " penalty: pick-up attempt before reaching checkpoint"
           << std::endl;

    this->penalty += this->weightFailedAttempt;

    _res.success = false;
    return true;
  }

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

  this->SetDone(result && rep.data());

  if (!this->Done())
  {
    gzmsg  << "[ServiceSim] " << this->weightFailedAttempt
           << " penalty: failed pick-up" << std::endl;

    this->penalty += this->weightFailedAttempt;
  }

  _res.success = this->Done();

  return true;
}

