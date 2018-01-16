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
#ifndef GAZEBO_PLUGINS_CREATEACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_CREATEACTORPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>

// See: https://bugreports.qt-project.org/browse/QTBUG-22829
#ifndef Q_MOC_RUN
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  // Forward declare private data class
  class CreateActorPluginPrivate;

  /// \brief
  class GAZEBO_VISIBLE CreateActorPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor.
    public: CreateActorPlugin();

    /// \brief Destructor.
    public: virtual ~CreateActorPlugin();

    /// \brief Spawn current actor
    private: void Spawn();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<CreateActorPluginPrivate> dataPtr;
  };
}

#endif
