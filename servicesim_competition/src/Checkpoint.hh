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

#ifndef SERVICESIM_CHECKPOINT_HH_
#define SERVICESIM_CHECKPOINT_HH_

#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>
#include <gazebo/common/Time.hh>

namespace servicesim
{
  class Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    public: Checkpoint(const sdf::ElementPtr &_sdf, const unsigned int _number);

    /// \brief Default destructor
    public: virtual ~Checkpoint() = default;

    /// \brief Check whether checkpoint has been completed.
    /// Any publishers or subscribers should be created the first time this is
    /// called, and cleaned up once it returns true.
    /// \return True if completed.
    public: virtual bool Check() = 0;

    /// \brief Call this the first time the checkpoint is checked.
    public: virtual void Start();

    /// \brief Get the current score for this checkpoint.
    /// \return Score
    public: double Score() const;

    /// \brief Sim time when the checkpoint ended
    protected: gazebo::common::Time endTime;

    /// \brief Sim time when the checkpoint started
    private: gazebo::common::Time startTime;

    /// \brief The checkpoint number
    private: unsigned int number{0};

    /// \brief The weight for this checkpoint when scoring.
    private: double weight{0.0};
  };

  /// \brief A checkpoint tied to a gazebo::ContainPlugin.
  class ContainCheckpoint : public Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    public: ContainCheckpoint(const sdf::ElementPtr &_sdf,
        const unsigned int _number);

    /// \brief Check whether the contain checkpoint has been completed.
    /// \return True if completed.
    protected: bool Check() override;

    /// \brief Callback when messages are received from the ContainPlugin.
    /// \param[in] _msg True if contains.
    private: void OnContain(const ignition::msgs::Boolean &_msg);

    /// \brief Ignition transport node for communication.
    protected: ignition::transport::Node ignNode;

    /// \brief Namespace for transport
    protected: std::string ns;

    /// \brief True if enabled
    private: bool enabled{false};

    /// \brief Flag to indicate whether contain has been achieved.
    private: bool containDone{false};
  };
}
#endif
