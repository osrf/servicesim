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

#ifndef SERVICESIM_CP_PICKUP_HH_
#define SERVICESIM_CP_PICKUP_HH_

#include <servicesim_competition/PickUpGuest.h>

#include "Checkpoint.hh"

namespace servicesim
{
  class CP_PickUpPrivate;

  /// \brief Checkpoint: Pick-up guest
  class CP_PickUp : public Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    public: CP_PickUp(const sdf::ElementPtr &_sdf);

    // Documentation inherited
    protected: bool Check() override;

    /// \brief Service for picking up the guest
    /// \param[in] _req Request
    /// \param[in] _res Response
    private: bool OnPickUpRosRequest(
        servicesim_competition::PickUpGuest::Request &_req,
        servicesim_competition::PickUpGuest::Response &_res);

    /// \brief ROS node handle
    public: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief Ignition transport node for communication.
    private: ignition::transport::Node ignNode;

    /// \brief PickUp ROS service
    private: ros::ServiceServer pickUpRosService;

    /// \brief The weight for each failed pick-up attempt
    private: double weightFailedAttempt{0.0};
  };
}
#endif
