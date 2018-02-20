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

#ifndef GAZEBO_PLUGINS_VICINITY_PLUGIN_HH
#define GAZEBO_PLUGINS_VICINITY_PLUGIN_HH

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include <servicesim_competition/ActorNames.h>

namespace servicesim
{
  class VicinityPlugin: public gazebo::ModelPlugin
  {

  public: VicinityPlugin();
  public: virtual ~VicinityPlugin();
  public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  public: void Update();

  //Store pointer to the model
  private: gazebo::physics::ModelPtr model_;
  //Store pointer to the world
  private: gazebo::physics::WorldPtr world_;
  private: gazebo::event::ConnectionPtr updateConnection;
  private: double threshold_;
  private: double update_rate_;
  private: std::string topicName_;
  private: gazebo::common::Time last_time_;

  private: std::vector<gazebo::physics::ActorPtr> actorPtrs_;
  /// ROS stuff
  private: ros::NodeHandle * rosnode_;
  private: ros::Publisher vicinity_pub_;
  };
}

#endif
