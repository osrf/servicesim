/*
 * Copyright (C) 2018 Open Source Robotics Foundation, Inc.
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

#include "VicinityPlugin.hh"

using namespace::servicesim;

GZ_REGISTER_MODEL_PLUGIN(servicesim::VicinityPlugin);

////////////////////////
// Constructor
VicinityPlugin::VicinityPlugin()
{
}

//Destructor
VicinityPlugin::~VicinityPlugin()
{

}

void VicinityPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->threshold_ = 5.0;

  this->model_ = _parent;
  this->world_ = _parent->GetWorld(); // Store the pointer to the world

  for (unsigned int i = 0; i < this->world_->ModelCount(); ++i)
  {
    auto model = this->world_->ModelByIndex(i);
    if (gazebo::physics::ActorPtr a = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model))
    {
      this->actorPtrs_.push_back(a);
    }
  }
  gzmsg << "Found " << this->actorPtrs_.size() << " actors in the world" << std::endl;

  // std::string model_name = this->model->GetName();
  // std::string topic_name = "vicinity";
  // pub = nh.advertise<std_msgs::Bool>(topic_name,1);
  // paramName = "guest_name";
  // if (_sdf->HasElement(paramName))
  // {
  //   this->entity = world->GetEntity(_sdf->Get<std::string>(paramName));
  // }
  paramName = "threshold";
  if (_sdf->HasElement(paramName))
  {
    this->threshold_ = _sdf->Get<double>(paramName);
  }
  else
  {
    this->threshold_ = 5.0;
  }

  if (!_sdf->HasElement("updateRate"))
  {
    this->update_rate_ = 1.0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();


  this->last_time_ = this->world_->SimTime();
  // Listen to the update event
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&VicinityPlugin::Update, this));
}

void VicinityPlugin::Update()
{
  gazebo::common::Time cur_time = this->world_->SimTime();
  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;
  gzmsg << "Updating Vicinity" << std::endl;
  this->last_time_ = cur_time;
  // auto robot_pos = this->model->WorldPose().Pos();
  // auto entity_pos = this->entity->WordlPose().Pos();
}
