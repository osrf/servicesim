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

#ifndef SERVICESIM_PENALTYCHECKER_HH_
#define SERVICESIM_PENALTYCHECKER_HH_

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

namespace servicesim
{
  /// \brief Responsible for checking penalties which are not
  /// checkpoint-specific.
  class PenaltyChecker
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element with configuration.
    public: PenaltyChecker(const sdf::ElementPtr &_sdf);

    /// \brief Destructor
    public: virtual ~PenaltyChecker();

    /// \brief Returns the current total penalty.
    public: double Penalty() const;

    /// \brief Callback when contact message is received
    private: void OnContacts(ConstContactsPtr &_msg);

    /// \brief Total penalties
    private: double penalty{0.0};

    /// \brief Penalty weight when contact with human happens. Will be
    /// multiplied by contact depth.
    private: double weightHumanContact{0.0};

    /// \brief Penalty weight when contact with an object happens. Will be
    /// multiplied by contact depth.
    private: double weightObjContact{0.0};

    /// \brief Penalty weight when approximation with human happens. Will be
    /// multiplied by approximation depth.
    private: double weightHumanApproximation{0.0};

    /// \brief Penalty weight when approximation with an object happens. Will be
    /// multiplied by approximation depth.
    private: double weightObjApproximation{0.0};

    /// \brief Robot name
    private: std::string robotName;

    /// \brief Ground name
    private: std::string groundName;

    /// \brief Prefix common to all human names
    private: std::string humanName;

    private: std::mutex mutex;

    /// \brief Gazebo node for communication.
    private: gazebo::transport::NodePtr gzNode;

    /// \brief Subscription to contact data.
    private: gazebo::transport::SubscriberPtr contactsSub;
  };
}
#endif
