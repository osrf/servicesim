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
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>
#include <servicesim_competition/Score.h>

#include "CompetitionPlugin.hh"
#include "Conversions.hh"
#include "CP_DropOff.hh"
#include "CP_GoToPickUp.hh"
#include "CP_PickUp.hh"
#include "CP_ReturnToStart.hh"
#include "PenaltyChecker.hh"

/////////////////////////////////////////////////
class servicesim::CompetitionPluginPrivate
{
  /// \brief Pick-up location name
  public: std::string pickUpLocation;

  /// \brief Drop-off location name
  public: std::string dropOffLocation;

  /// \brief Guest name
  public: std::string guestName;

  /// \brief Keep robot start pose
  public: ignition::math::Pose3d robotStartPose;

  /// \brief Connection to world update
  public: gazebo::event::ConnectionPtr updateConnection{nullptr};

  /// \brief Vector of checkpoints
  public: std::vector<std::unique_ptr<Checkpoint>> checkpoints;

  /// \brief Current checkpoint number, starting from 1.
  /// Zero means no checkpoint.
  public: uint8_t current{0};

  /// \brief ROS node handle
  public: std::unique_ptr<ros::NodeHandle> rosNode{nullptr};

  /// \brief ROS new task service server
  public: ros::ServiceServer newTaskRosService;

  /// \brief ROS task info service server
  public: ros::ServiceServer taskInfoRosService;

  /// \brief ROS room info service server
  public: ros::ServiceServer roomInfoRosService;

  /// \brief ROS publisher for the score.
  public: ros::Publisher scoreRosPub;

  /// \brief Frequency in Hz to publish score message
  public: double scoreFreq{50};

  /// \brief Map with coordinates for every room's drop-off region
  /// Room name - pair<min, max>
  public: std::map<std::string, std::pair<ignition::math::Vector3d,
                                          ignition::math::Vector3d>> roomInfo;

  /// \brief Penalty checker
  public: std::unique_ptr<PenaltyChecker> penaltyChecker{nullptr};
};

using namespace servicesim;

GZ_REGISTER_WORLD_PLUGIN(CompetitionPlugin)

/////////////////////////////////////////////////
CompetitionPlugin::CompetitionPlugin() : WorldPlugin(),
    dataPtr(new CompetitionPluginPrivate)
{
}

/////////////////////////////////////////////////
void CompetitionPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  // Set solver tolerance
  if (_sdf->HasElement("sor_lcp_tolerance"))
  {
    double sorLcpTolerance = _sdf->Get<double>("sor_lcp_tolerance");
    auto physics = _world->Physics();
    physics->SetParam("sor_lcp_tolerance", sorLcpTolerance);
  }

  // Load general competition parameters
  if (_sdf->HasElement("score_frequency"))
    this->dataPtr->scoreFreq = _sdf->Get<double>("score_frequency");

  if (!_sdf->HasElement("pick_up_location"))
  {
    gzerr << "Missing <pick_up_location>, competition not initialized"
          << std::endl;
    return;
  }
  this->dataPtr->pickUpLocation = _sdf->Get<std::string>("pick_up_location");

  if (!_sdf->HasElement("drop_off_location"))
  {
    gzerr << "Missing <drop_off_location>, competition not initialized"
          << std::endl;
    return;
  }
  this->dataPtr->dropOffLocation = _sdf->Get<std::string>("drop_off_location");

  if (!_sdf->HasElement("robot_start_pose"))
  {
    gzerr << "Missing <robot_start_pose>, competition not initialized"
          << std::endl;
    return;
  }
  this->dataPtr->robotStartPose =
      _sdf->Get<ignition::math::Pose3d>("robot_start_pose");

  this->dataPtr->guestName = _sdf->Get<std::string>("guest_name");

  if (!_sdf->HasElement("room_info"))
  {
    gzerr << "Missing <room_info>, competition not initialized"
          << std::endl;
    return;
  }

  auto roomInfoElem = _sdf->GetElement("room_info");
  while (roomInfoElem)
  {
    auto name = roomInfoElem->Get<std::string>("name");
    auto min = roomInfoElem->Get<ignition::math::Vector3d>("min");
    auto max = roomInfoElem->Get<ignition::math::Vector3d>("max");

    this->dataPtr->roomInfo[name] = std::make_pair(min, max);
    roomInfoElem = roomInfoElem->GetNextElement("room_info");
  }

  // Create checkpoints
  {
    std::unique_ptr<CP_GoToPickUp> cp(new CP_GoToPickUp(
        _sdf->GetElement("go_to_pick_up")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  {
    std::unique_ptr<CP_PickUp> cp(new CP_PickUp(
        _sdf->GetElement("pick_up")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  {
    std::unique_ptr<CP_DropOff> cp(new CP_DropOff(
        _sdf->GetElement("drop_off")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  {
    std::unique_ptr<CP_ReturnToStart> cp(new CP_ReturnToStart(
        _sdf->GetElement("return_to_start")));
    this->dataPtr->checkpoints.push_back(std::move(cp));
  }

  // Penalty checker
  this->dataPtr->penaltyChecker.reset(new PenaltyChecker(_sdf));

  // ROS transport
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->dataPtr->rosNode.reset(new ros::NodeHandle());

  // Advertise new task service
  this->dataPtr->newTaskRosService = this->dataPtr->rosNode->advertiseService(
      "/servicesim/new_task", &CompetitionPlugin::OnNewTaskRosService, this);

  // Advertise task info service
  this->dataPtr->taskInfoRosService = this->dataPtr->rosNode->advertiseService(
      "/servicesim/task_info", &CompetitionPlugin::OnTaskInfoRosService, this);

  // Advertise room info service
  this->dataPtr->roomInfoRosService = this->dataPtr->rosNode->advertiseService(
      "/servicesim/room_info", &CompetitionPlugin::OnRoomInfoRosService, this);

  // Advertise score messages
  this->dataPtr->scoreRosPub =
      this->dataPtr->rosNode->advertise<servicesim_competition::Score>(
      "/servicesim/score", 1000);

  // Trigger update at every world iteration
  this->dataPtr->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
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
  _res.pick_up_location = this->dataPtr->pickUpLocation;
  _res.drop_off_location = this->dataPtr->dropOffLocation;
  _res.guest_name = this->dataPtr->guestName;
  _res.robot_start_pose = convert(this->dataPtr->robotStartPose);

  return true;
}

/////////////////////////////////////////////////
bool CompetitionPlugin::OnTaskInfoRosService(
    servicesim_competition::TaskInfo::Request &_req,
    servicesim_competition::TaskInfo::Response &_res)
{
  if (this->dataPtr->current == 0)
  {
    gzerr << "Competition has not been started yet."
          << " Please call `/servicesim/new_task`"
          <<   "service to start the competition." << std::endl;

    return false;
  }
  // Respond
  _res.pick_up_location = this->dataPtr->pickUpLocation;
  _res.drop_off_location = this->dataPtr->dropOffLocation;
  _res.guest_name = this->dataPtr->guestName;
  _res.robot_start_pose = convert(this->dataPtr->robotStartPose);

  return true;
}

/////////////////////////////////////////////////
bool CompetitionPlugin::OnRoomInfoRosService(
    servicesim_competition::RoomInfo::Request &_req,
    servicesim_competition::RoomInfo::Response &_res)
{
  if (this->dataPtr->roomInfo.find(_req.name) == this->dataPtr->roomInfo.end())
  {
    gzwarn << "Unknown room [" << _req.name << "]" << std::endl;
    return false;
  }

  auto pts = this->dataPtr->roomInfo[_req.name];

  _res.min = convert(pts.first);
  _res.max = convert(pts.second);

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

  // Publish ROS score message
  static gazebo::common::Time lastScorePubTime = _info.simTime;

  if (_info.simTime - lastScorePubTime < 1/this->dataPtr->scoreFreq)
    return;

  servicesim_competition::Score msg;
  double total{0.0};
  for (int i = 0; i < this->dataPtr->checkpoints.size(); ++i)
  {
    total += this->dataPtr->checkpoints[i]->Score();
  }

  total += this->dataPtr->penaltyChecker->Penalty();

  msg.score = total;

  this->dataPtr->scoreRosPub.publish(msg);
  lastScorePubTime = _info.simTime;
}
