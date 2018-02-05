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
    /// \param[in] _number Unique number for this checkpoint
    public: CP_DropOff(const sdf::ElementPtr &_sdf,
        const unsigned int _number);

    // Documentation inherited
    public: virtual void Start();

    // Documentation inherited
    protected: bool Check() override;

    // Documentation inherited
    protected: bool Paused() override;

    // Documentation inherited
    public: virtual double Score() const;

    /// \brief Service for dropping off the guest
    /// \param[in] _req Request
    /// \param[in] _res Response
    private: bool OnDropOffRosRequest(
        servicesim_competition::DropOffGuest::Request &_req,
        servicesim_competition::DropOffGuest::Response &_res);

    /// \brief Ignition transport node for communication.
    private: ignition::transport::Node ignNode;

    /// \brief ROS node handle
    public: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief DropOff ROS service
    private: ros::ServiceServer dropOffRosService;

    /// \brief Vector of sim time intervals when the checkpoint was
    /// running. The first time is the beginning of the interval, the
    /// second is the end
    private: std::vector<std::pair<gazebo::common::Time,
                                   gazebo::common::Time>> intervals;

    /// \brief True if checkpoint is paused
    private: bool paused{true};
  };
}
#endif
