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

#ifndef SERVICESIM_CHECKPOINT2_HH_
#define SERVICESIM_CHECKPOINT2_HH_

#include <servicesim_competition/PickUpGuest.h>

#include "Checkpoint.hh"

namespace servicesim
{
  class Checkpoint2Private;

  /// \brief Checkpoint 2
  class Checkpoint2 : public Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    /// \param[in] _number Unique number for this checkpoint
    public: Checkpoint2(const sdf::ElementPtr &_sdf,
        const unsigned int _number);

    // Documentation inherited
    protected: bool Check() override;

    /// \brief Service for picking up the guest
    /// \param[in] _req Request
    /// \param[in] _res Response
    private: bool OnPickUpRosRequest(
        servicesim_competition::PickUpGuest::Request &_req,
        servicesim_competition::PickUpGuest::Response &_res);

    /// \brief Ignition transport node for communication.
    private: ignition::transport::Node ignNode;

    /// \brief ROS node handle
    public: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief PickUp ROS service
    private: ros::ServiceServer pickUpRosService;
  };
}
#endif
