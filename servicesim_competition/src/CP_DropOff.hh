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

#ifndef SERVICESIM_CP_DROPOFF_HH_
#define SERVICESIM_CP_DROPOFF_HH_

#include <vector>
#include <gazebo/common/Time.hh>
#include <servicesim_competition/DropOffGuest.h>

#include "Checkpoint.hh"

namespace servicesim
{
  class CP_DropOffPrivate;

  /// \brief Checkpoint: Drop-off guest
  class CP_DropOff : public Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    public: CP_DropOff(const sdf::ElementPtr &_sdf);

    // Documentation inherited
    protected: bool Check() override;

    /// \brief Service for dropping off the guest
    /// \param[in] _req Request
    /// \param[in] _res Response
    private: bool OnDropOffRosRequest(
        servicesim_competition::DropOffGuest::Request &_req,
        servicesim_competition::DropOffGuest::Response &_res);

    /// \brief Callback when messages are received from the ContainPlugin.
    /// \param[in] _msg True if contains.
    private: void OnContain(const ignition::msgs::Boolean &_msg);

    /// \brief Callback when drift messages are received from FollowActorPlugin.
    /// \param[in] _msg Message with a code for the drift reason
    private: void OnDrift(const ignition::msgs::UInt32 &_msg);

    /// \brief Callabck for enable service
    /// \param[in] _rep Response
    /// \param[in] _result Result
    private: void EnableCallback(const ignition::msgs::Boolean &_rep,
        const bool _result);

    /// \brief Namespace for transport
    private: std::string ns;

    /// \brief Ignition transport node for communication.
    private: ignition::transport::Node ignNode;

    /// \brief ROS node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief DropOff ROS service
    private: ros::ServiceServer dropOffRosService;

    /// \brief True if checkpoint is paused
    private: bool paused{true};

    /// \brief Guest name
    private: std::string guestName;

    /// \brief True if enabled
    private: bool enabled{false};

    /// \brief True if guest is currently in the drop-off area
    private: bool containGuest{false};

    /// \brief The weight for each failed pick-up attempt
    private: double weightFailedAttempt{0.0};

    /// \brief The weight for each time the robot moves too fast and the guest
    /// can't follow.
    private: double weightTooFast{0.0};
  };
}
#endif
