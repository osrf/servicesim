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

#include <ignition/math/Vector3.hh>

#include <gazebo/physics/Actor.hh>
#include <gazebo/physics/BoxShape.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include "CollisionActorPlugin.hh"

using namespace servicesim;
GZ_REGISTER_MODEL_PLUGIN(CollisionActorPlugin)

/////////////////////////////////////////////////
CollisionActorPlugin::CollisionActorPlugin()
{
}

/////////////////////////////////////////////////
void CollisionActorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get a pointer to the actor
  auto actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);

  // Map of collision scaling factors
  std::map<std::string, ignition::math::Vector3d> scaling;

  // Map of collision pose offsets
  std::map<std::string, ignition::math::Pose3d> offsets;

  // Vector of collisions to be deleted
  std::vector<std::string> deleting;

  // Read in the collision scaling factors, if present
  if (_sdf->HasElement("collision"))
  {
    auto elem = _sdf->GetElement("collision");
    while (elem)
    {
      if (!elem->HasAttribute("name"))
      {
        gzwarn << "Skipping element without name attribute" << std::endl;
        elem = elem->GetNextElement("collision");
        continue;
      }
      auto name = elem->Get<std::string>("name");

      if (elem->HasAttribute("scale"))
      {
        auto scale = elem->Get<ignition::math::Vector3d>("scale");
        scaling[name] = scale;
      }

      if (elem->HasAttribute("pose"))
      {
        auto pose = elem->Get<ignition::math::Pose3d>("pose");
        offsets[name] = pose;
      }

      if (elem->HasAttribute("delete") && elem->Get<bool>("delete"))
      {
        deleting.push_back(name);
      }
      elem = elem->GetNextElement("collision");
    }
  }

  for (const auto &link : actor->GetLinks())
  {
    // Init the links, which in turn enables collisions
    link->Init();

    if (scaling.empty())
      continue;

    // Process all the collisions in all the links
    for (const auto &collision : link->GetCollisions())
    {
      auto name = collision->GetName();

      if (std::find(deleting.begin(), deleting.end(), name) != deleting.end())
      {
        gzdbg << "Remove col " << name << std::endl;
        link->RemoveCollision(name);
        continue;
      }

      if (scaling.find(name) != scaling.end())
      {
        auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
            collision->GetShape());

        // Make sure we have a box shape.
        if (boxShape)
          boxShape->SetSize(boxShape->Size() * scaling[name]);
      }

      if (offsets.find(name) != offsets.end())
      {
        collision->SetInitialRelativePose(
            offsets[name] + collision->InitialRelativePose());
      }
    }
  }
}

