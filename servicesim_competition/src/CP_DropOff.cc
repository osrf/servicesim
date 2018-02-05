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
  // Ignition transport
  std::function<void(const ignition::msgs::Time &)> driftCb =
      [this](const ignition::msgs::Time &_msg)
  {
    // Drift time
    gazebo::common::Time time;
    time.Set(_msg.sec(), _msg.nsec());

    // End current interval
    auto interval = this->intervals.back();
    interval.second = time;

    // Set paused
    this->paused = true;
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
void CP_DropOff::Start()
{
  Checkpoint::Start();

  // Start new interval
  std::pair<gazebo::common::Time, gazebo::common::Time> interval(
      this->startTime, gazebo::common::Time::Zero);
  this->intervals.push_back(interval);
}

/////////////////////////////////////////////////
bool CP_DropOff::Check()
{
  return this->done;
}

/////////////////////////////////////////////////
bool CP_DropOff::Paused()
{
  if (this->done)
    return false;

  return false;
}

/////////////////////////////////////////////////
double CP_DropOff::Score() const
{
  double elapsedSeconds{0.0};

  // Iterate over all intervals
  for (auto i : this->intervals)
  {
    auto start = i.first;
    auto end = i.second;

    // If not finished yet
    if (end == gazebo::common::Time::Zero)
      end = gazebo::physics::get_world()->SimTime();

    elapsedSeconds += (end - start).Double();
  }

  return elapsedSeconds * this->weight;
}

/////////////////////////////////////////////////
bool CP_DropOff::OnDropOffRosRequest(
    servicesim_competition::DropOffGuest::Request &_req,
    servicesim_competition::DropOffGuest::Response &_res)
{
  // Get info from ROS request
  auto guestName = _req.guest_name;

  // TODO: Check if robot and guest are in drop-off location

  // Send Ignition request to actor
  auto ignService = "/servicesim/" + guestName + "/unfollow";
  ignition::msgs::Boolean rep;
  bool result{false};

  // Send synchronous request so we can tell the robot if it succeeded
  bool executed = this->ignNode.Request(ignService, 500, rep, result);
  if (!executed)
    gzerr << "Unfollow request timed out" << std::endl;

  this->done = result && rep.data();

  if (!this->done)
  {
    // TODO: apply penalty for bad dropoff request
  }

  // Set end time
  if (this->done && this->endTime == gazebo::common::Time::Zero)
  {
    this->endTime = gazebo::physics::get_world()->SimTime();

    auto interval = this->intervals.back();
    interval.second = this->endTime;
  }

  _res.success = this->done;

  return true;
}

