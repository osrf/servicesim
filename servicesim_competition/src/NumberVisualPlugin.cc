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

#include <gazebo/rendering/MovableText.hh>
#include <gazebo/rendering/Visual.hh>
#include "NumberVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the NumberVisualPlugin class.
  class NumberVisualPluginPrivate
  {
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(NumberVisualPlugin)

/////////////////////////////////////////////////
NumberVisualPlugin::NumberVisualPlugin() : dataPtr(new NumberVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
NumberVisualPlugin::~NumberVisualPlugin()
{
}

/////////////////////////////////////////////////
void NumberVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  // Visual name
  auto name = _visual->Name();

  // Skip if it't the preview
  if (name == "CreateActor::ghost::head::Visual_0")
    return;

  // Number from name
  name = name.substr(0, name.find("::"));

  auto id = name.find("_");
  // The first one has no number
  if (id == std::string::npos)
  {
    name = "1";
  }
  else
  {
    name = name.substr(id+1);
    name = std::to_string(std::stoi(name) + 2);
  }

  // Get head bounding box
  auto visBox = _visual->BoundingBox();

  // Create text
  auto text = new rendering::MovableText();
  text->Load(_visual->Name() + "_TEXT",
      name, "Arial", 0.8, common::Color::Green);
  text->SetBaseline(visBox.Max().Z() + 0.1);
  text->SetShowOnTop(true);
  text->SetTextAlignment(rendering::MovableText::H_CENTER,
                         rendering::MovableText::V_BELOW);

  // Attach text to the visual's node
  // FIXME: Using wall time to generate unique names, Gazebo is not deleting
  // the node when the visual is deleted.
  auto textNode = _visual->GetSceneNode()->createChildSceneNode(
      _visual->Name() + "_TEXT__NODE__" +
      common::Time::GetWallTimeAsISOString());
  textNode->attachObject(text);
  textNode->setInheritScale(false);
}

