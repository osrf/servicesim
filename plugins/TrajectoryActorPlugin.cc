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

#include <functional>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "TrajectoryActorPlugin.hh"

using namespace gazebo;
using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::TrajectoryActorPlugin)

class servicesim::TrajectoryActorPluginPrivate
{
  /// \brief Pointer to the parent actor.
  public: physics::ActorPtr actor{nullptr};

  /// \brief Pointer to the world, for convenience.
  public: physics::WorldPtr world{nullptr};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief List of connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Current target location
  public: std::vector<ignition::math::Pose3d> targets;
  public: unsigned int currentTarget{0};

  /// \brief Target location weight (used for vector field)
  public: double targetWeight{1.15};

  /// \brief Obstacle weight (used for vector field)
  public: double obstacleWeight{1.5};

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  public: double animationFactor{4.5};

  /// \brief Time of the last update.
  public: common::Time lastUpdate;

  /// \brief List of models to ignore. Used for vector field
  public: std::vector<std::string> ignoreModels;

  /// \brief Custom trajectory info.
  public: physics::TrajectoryInfoPtr trajectoryInfo;

  /// \brief Animation
  public: std::string animation{"animation"};
};

/////////////////////////////////////////////////
TrajectoryActorPlugin::TrajectoryActorPlugin()
    : dataPtr(new TrajectoryActorPluginPrivate)
{
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->dataPtr->world = this->dataPtr->actor->GetWorld();

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrajectoryActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Read in the velocity
  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");

  // Read in the target poses
  auto targetElem = _sdf->GetElement("target");
  while (targetElem)
  {
    this->dataPtr->targets.push_back(targetElem->Get<ignition::math::Pose3d>());
    targetElem = targetElem->GetNextElement("target");
  }

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->dataPtr->targetWeight = _sdf->Get<double>("target_weight");

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->dataPtr->obstacleWeight = _sdf->Get<double>("obstacle_weight");

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->dataPtr->animationFactor = _sdf->Get<double>("animation_factor");

  // Add our own name to models we should ignore when avoiding obstacles.
  this->dataPtr->ignoreModels.push_back(this->dataPtr->actor->GetName());

  // Read in the other obstacles to ignore
  auto ignoreElem = _sdf->GetElement("ignore");
  while (ignoreElem)
  {
    this->dataPtr->ignoreModels.push_back(ignoreElem->Get<std::string>());
    ignoreElem = ignoreElem->GetNextElement("ignore");
  }

  // Read in the animation
  if (_sdf->HasElement("animation"))
    this->dataPtr->animation = _sdf->Get<std::string>("animation");

  auto skelAnims = this->dataPtr->actor->SkeletonAnimations();
  if (skelAnims.find(this->dataPtr->animation) == skelAnims.end())
  {
    gzerr << "Skeleton animation [" << this->dataPtr->animation << "] not found."
          << std::endl;
  }
  else
  {
    // Create custom trajectory
    this->dataPtr->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->dataPtr->trajectoryInfo->type = this->dataPtr->animation;
    this->dataPtr->trajectoryInfo->duration = 1.0;

    this->dataPtr->actor->SetCustomTrajectory(this->dataPtr->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  // Iterate over all models in the world
  for (unsigned int i = 0; i < this->dataPtr->world->ModelCount(); ++i)
  {
    // Skip models we're ignoring
    physics::ModelPtr model = this->dataPtr->world->ModelByIndex(i);
    if (std::find(this->dataPtr->ignoreModels.begin(), this->dataPtr->ignoreModels.end(),
        model->GetName()) != this->dataPtr->ignoreModels.end())
    {
      continue;
    }

    // Check against point obstacles
    auto offset = model->WorldPose().Pos() - this->dataPtr->actor->WorldPose().Pos();
    double modelDist = offset.Length();
    if (modelDist < 1.5)
    {
      double invModelDist = this->dataPtr->obstacleWeight / modelDist;
      offset.Normalize();
      offset *= invModelDist;
      _pos -= offset;
    }

    // TODO: Improve obstacle avoidance. Some ideas: check contacts, ray-query
    // the path forward, check against bounding box of each collision shape...
  }
}

/////////////////////////////////////////////////
void TrajectoryActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->dataPtr->lastUpdate).Double();

  auto actorPose = this->dataPtr->actor->WorldPose();

  // 2D distance to target
  auto posDiff = this->dataPtr->targets[this->dataPtr->currentTarget].Pos() -
      actorPose.Pos();
  posDiff.Z(0);

  double distance = posDiff.Length();

  // Get next target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->dataPtr->currentTarget++;
    if (this->dataPtr->currentTarget > this->dataPtr->targets.size() - 1)
      this->dataPtr->currentTarget = 0;
    posDiff = this->dataPtr->targets[this->dataPtr->currentTarget].Pos() - actorPose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  posDiff = posDiff.Normalize() * this->dataPtr->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(posDiff);

  // Compute the yaw orientation
  auto rpy = actorPose.Rot().Euler();
  ignition::math::Angle yaw = atan2(posDiff.Y(), posDiff.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    actorPose.Pos() += posDiff * this->dataPtr->velocity * dt;
    actorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }
  actorPose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() -
      this->dataPtr->actor->WorldPose().Pos()).Length();

  this->dataPtr->actor->SetWorldPose(actorPose, false, false);
  this->dataPtr->actor->SetScriptTime(this->dataPtr->actor->ScriptTime() +
    (distanceTraveled * this->dataPtr->animationFactor));
  this->dataPtr->lastUpdate = _info.simTime;
}
