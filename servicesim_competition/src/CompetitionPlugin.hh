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

#ifndef SERVICESIM_COMPETITIONPLUGIN_HH_
#define SERVICESIM_COMPETITIONPLUGIN_HH_

#include <memory>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>

#include <servicesim_competition/NewTask.h>
#include <servicesim_competition/RoomInfo.h>
#include <servicesim_competition/TaskInfo.h>

namespace servicesim
{
  class CompetitionPluginPrivate;

  class CompetitionPlugin : public gazebo::WorldPlugin
  {
    // Documentation inherited
    public: CompetitionPlugin();

    // Documentation inherited
    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
        override;

    /// \brief Service when competitor asks to start competition.
    /// \param[in] _req Competitor's request.
    /// \param[out] _res Response containing information about the task.
    /// \return False if failed.
    private: bool OnNewTaskRosService(
        servicesim_competition::NewTask::Request &_req,
        servicesim_competition::NewTask::Response &_res);

    /// \brief Service when competitor asks about task information.
    /// \param[in] _req Competitor's request.
    /// \param[out] _res Response containing information about the task.
    /// \return False if failed.
    private: bool OnTaskInfoRosService(
        servicesim_competition::TaskInfo::Request &_req,
        servicesim_competition::TaskInfo::Response &_res);

    /// \brief Service when competitor asks information about a room.
    /// \param[in] _req Competitor's request.
    /// \param[out] _res Response containing information about the room.
    /// \return False if failed.
    private: bool OnRoomInfoRosService(
        servicesim_competition::RoomInfo::Request &_req,
        servicesim_competition::RoomInfo::Response &_res);

    /// \brief Update on world update begin
    /// \param[in] _info Update info
    private: void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// \internal
    private: std::unique_ptr<CompetitionPluginPrivate> dataPtr;
  };
}
#endif
