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

#ifndef SERVICESIM_INFLATIONPLUGIN_HH_
#define SERVICESIM_INFLATIONPLUGIN_HH_

#include <gazebo/common/Plugin.hh>

namespace servicesim
{
  class InflationPluginPrivate;

  /// \brief Enables <collide_without_contact> for all collisions in a given
  /// link which match a given collision name. It would be much more
  /// convenient to be able to just set this on the URDF...
  class InflationPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: InflationPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
        sdf::ElementPtr _sdf) override;
  };
}
#endif
