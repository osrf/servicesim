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

#include "CP_DropOff.hh"

using namespace servicesim;

/////////////////////////////////////////////////
CP_DropOff::CP_DropOff(const sdf::ElementPtr &_sdf)
    : Checkpoint(_sdf)
{
  this->canPause = true;

  // From ContainCheckpoint
  if (!_sdf || !_sdf->HasElement("namespace"))
    gzwarn << "Missing <namespace> for contain plugin" << std::endl;
  else
    this->ns = _sdf->Get<std::string>("namespace");

  // Ignition transport
  std::function<void(const ignition::msgs::UInt32 &)> driftCb =
      [this](const ignition::msgs::UInt32 &_msg)
  {
    // Drift reason
    auto reason = _msg.data();

    // 1: Robot moved too fast and actor couldn't follow
    if (reason == 1u)
    {
      // TODO: apply penalty
    }
    // 2: Scheduled drift time
    else if (reason == 2u)
    {
      // Do nothing
    }
    // 3: User requested unfollow - we assume it came from this checkpoint
    else if (reason == 3u)
    {
      // No pausing, we're finishing the checkpoint
      return;
    }
    else
    {
      gzerr << "Drift reason not supported [" << reason << "], not pausing."
            << std::endl;
      return;
    }

    // End current interval
    this->Pause();
  };

  this->ignNode.Subscribe("/servicesim/guest/drift", driftCb);

  // ROS transport
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode.reset(new ros::NodeHandle());

  this->dropOffRosService = this->rosNode->advertiseService(
      "/servicesim/dropoff_guest", &CP_DropOff::OnDropOffRosRequest, this);
}

/////////////////////////////////////////////////
void CP_DropOff::EnableCallback(const ignition::msgs::Boolean &/*_rep*/,
    const bool _result)
{
  if (_result)
    this->enabled = !this->enabled;
}

/////////////////////////////////////////////////
bool CP_DropOff::Check()
{
  // Enable contain checkpoint once
  if (!this->enabled && !this->Done())
  {
    // Setup contain subscriber
    this->ignNode.Subscribe(this->ns + "/contain",
        &CP_DropOff::OnContain, this);

    // Enable contain plugin
    ignition::msgs::Boolean req;
    req.set_data(true);
    this->ignNode.Request(this->ns + "/enable", req,
        &CP_DropOff::EnableCallback, this);
  }

  // Done, now clean up
  if (this->enabled && this->Done() &&
      !this->ignNode.SubscribedTopics().empty())
  {
    // Unsubscribe
    for (auto const &sub : this->ignNode.SubscribedTopics())
      this->ignNode.Unsubscribe(sub);

    // Disable contain plugin
    ignition::msgs::Boolean req;
    req.set_data(false);
    this->ignNode.Request(this->ns + "/enable", req,
        &CP_DropOff::EnableCallback, this);
  }

  return this->Done();
}

/////////////////////////////////////////////////
bool CP_DropOff::OnDropOffRosRequest(
    servicesim_competition::DropOffGuest::Request &_req,
    servicesim_competition::DropOffGuest::Response &_res)
{
  // Get info from ROS request
  auto guestName = _req.guest_name;

  // Check if guest is in drop-off location
  if (!this->containGuest)
  {
    // TODO: apply penalty for bad dropoff request
    gzwarn << "Failed to drop-off, guest not in drop-off area" << std::endl;
    _res.success = false;
    return true;
  }

  // Send Ignition request to actor
  auto ignService = "/servicesim/" + guestName + "/unfollow";
  ignition::msgs::Boolean rep;
  bool result{false};

  // Send synchronous request so we can tell the robot if it succeeded
  bool executed = this->ignNode.Request(ignService, 500, rep, result);
  if (!executed)
    gzerr << "Unfollow request timed out" << std::endl;

  this->SetDone(result && rep.data());

  if (!this->Done())
  {
    // TODO: apply penalty for bad dropoff request
  }

  _res.success = this->Done();

  return true;
}

//////////////////////////////////////////////////
void CP_DropOff::OnContain(const ignition::msgs::Boolean &_msg)
{
  this->containGuest = _msg.data();
}

