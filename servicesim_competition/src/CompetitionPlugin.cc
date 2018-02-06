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

#include <gazebo/common/Console.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>
#include <servicesim_competition/Score.h>

#include "CompetitionPlugin.hh"
#include "Conversions.hh"
#include "CP_DropOff.hh"
#include "CP_GoToPickUp.hh"
#include "CP_PickUp.hh"

/////////////////////////////////////////////////
class servicesim::CompetitionPluginPrivate
{
  /// \brief Pick-up location
  public: ignition::math::Pose3d pickUpLocation;

  /// \brief Drop-off location
  public: ignition::math::Pose3d dropOffLocation;

  /// \brief Guest name
  public: std::string guestName;

  /// \brief Connection to world update
  public: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Vector of checkpoints
  /// checkpoints[0]: Checkpoint 1
  /// checkpoints[1]: Checkpoint 2
  /// checkpoints[2]: Checkpoint 3
  /// checkpoints[3]: Checkpoint 4
  public: std::vector<std::unique_ptr<Checkpoint>> checkpoints;

  /// \brief Current checkpoint number, starting from 1.
  /// Zero means no checkpoint.
  public: uint8_t current = 0;

  /// \brief ROS node handle
  public: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief ROS new task service server
  public: ros::ServiceServer newTaskRosService;

  /// \brief ROS publisher which publishes the score.
  public: ros::Publisher scoreRosPub;
};

using namespace servicesim;

GZ_REGISTER_WORLD_PLUGIN(CompetitionPlugin)

/////////////////////////////////////////////////
CompetitionPlugin::CompetitionPlugin() : WorldPlugin(),
    dataPtr(new CompetitionPluginPrivate)
{
}

/////////////////////////////////////////////////
void CompetitionPlugin::Load(gazebo::physics::WorldPtr /*_world*/,
    sdf::ElementPtr _sdf)
{
  // Load parameters
  this->dataPtr->pickUpLocation =
      _sdf->Get<ignition::math::Pose3d>("pick_up_location");
  this->dataPtr->dropOffLocation =
      _sdf->Get<ignition::math::Pose3d>("drop_off_location");
  this->dataPtr->guestName = _sdf->Get<std::string>("guest_name");

  // Checkpoint 1
  {
    std::unique_ptr<CP_GoToPickUp> cp(new CP_GoToPickUp(
        _sdf->GetElement("go_to_pick_up")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  // Checkpoint 2
  {
    std::unique_ptr<CP_PickUp> cp(new CP_PickUp(
        _sdf->GetElement("pick_up")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  // Checkpoint 3
  {
    std::unique_ptr<CP_DropOff> cp(new CP_DropOff(
        _sdf->GetElement("drop_off")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  // ROS transport
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->dataPtr->rosNode.reset(new ros::NodeHandle());

  this->dataPtr->newTaskRosService = this->dataPtr->rosNode->advertiseService(
      "/servicesim/new_task", &CompetitionPlugin::OnNewTaskRosService, this);

  this->dataPtr->scoreRosPub =
      this->dataPtr->rosNode->advertise<servicesim_competition::Score>(
      "/servicesim/score", 1000);

  // Trigger update at every world iteration
  this->dataPtr->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&CompetitionPlugin::OnUpdate, this, std::placeholders::_1));

  gzmsg << "[ServiceSim] Competition plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
bool CompetitionPlugin::OnNewTaskRosService(
    servicesim_competition::NewTask::Request &_req,
    servicesim_competition::NewTask::Response &_res)
{
  if (this->dataPtr->current != 0)
  {
    gzerr << "Competition is already running." << std::endl;
    return false;
  }

  // Start checkpoint
  this->dataPtr->current = 1;
  this->dataPtr->checkpoints[this->dataPtr->current - 1]->Start();

  // Respond
  _res.pick_up_location = servicesim::convert(this->dataPtr->pickUpLocation);
  _res.drop_off_location = servicesim::convert(this->dataPtr->dropOffLocation);
  _res.guest_name = this->dataPtr->guestName;

  return true;
}

/////////////////////////////////////////////////
void CompetitionPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
  if (this->dataPtr->current == 0)
    return;

  // If current checkpoint is complete
  if (this->dataPtr->checkpoints[this->dataPtr->current - 1]->Check())
  {
    // Next checkpoint
    this->dataPtr->current++;

    // Check if competition complete
    if (this->dataPtr->current > this->dataPtr->checkpoints.size())
    {
      gzmsg << "[ServiceSim] Competition complete!" << std::endl;
      this->dataPtr->current = 0;
    }
    else
    {
      this->dataPtr->checkpoints[this->dataPtr->current - 1]->Start();
    }
  }

  // If current checkpoint is paused, go back to previous one
  if (this->dataPtr->current > 0 &&
      this->dataPtr->checkpoints[this->dataPtr->current - 1]->Paused())
  {
    // Previous checkpoint
    this->dataPtr->current--;

    if (this->dataPtr->current > 0)
      this->dataPtr->checkpoints[this->dataPtr->current - 1]->Start();
  }

  // TODO: Check penalties

  // Publish ROS score message
  static gazebo::common::Time lastScorePubTime = _info.simTime;

  // TODO
  const double freq = 10;

  if (_info.simTime - lastScorePubTime < 1/freq)
    return;

  servicesim_competition::Score msg;

  for (int i = 0; i < this->dataPtr->checkpoints.size(); ++i)
  {
    msg.checkpoints.push_back(this->dataPtr->checkpoints[i]->Score());
  }

  // TODO add penalties

  this->dataPtr->scoreRosPub.publish(msg);
  lastScorePubTime = _info.simTime;
}

