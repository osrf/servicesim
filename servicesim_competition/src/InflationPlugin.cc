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
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/SurfaceParams.hh>

#include "InflationPlugin.hh"

using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(servicesim::InflationPlugin)

/////////////////////////////////////////////////
InflationPlugin::InflationPlugin()
{
}

/////////////////////////////////////////////////
void InflationPlugin::Load(gazebo::physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{
  // Link
  if (!_sdf->HasElement("link"))
  {
    gzerr << "Missing <link>" << std::endl;
    return;
  }
  auto linkName = _sdf->Get<std::string>("link");

  auto link = _model->GetLink(linkName);
  if (!link)
  {
    gzerr << "Failed to find link [" << linkName << "]" << std::endl;
    return;
  }

  // Collisions
  if (!_sdf->HasElement("collision"))
  {
    gzerr << "Missing <collision>" << std::endl;
    return;
  }
  auto collisionSubStr = _sdf->Get<std::string>("collision");

  auto cols = link->GetCollisions();
  for (const auto &col : cols)
  {
    if (col->GetName().find(collisionSubStr) != std::string::npos)
    {
      auto surf = col->GetSurface();
      surf->collideWithoutContact = true;
    }
  }
}

