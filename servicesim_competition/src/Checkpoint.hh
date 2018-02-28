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
    /// \param[in] _sdf SDF element with configuration for this checkpoint.
    public: Checkpoint(const sdf::ElementPtr &_sdf);

    /// \brief Default destructor
    public: virtual ~Checkpoint() = default;

    /// \brief Check whether checkpoint has been completed.
    /// Any publishers or subscribers should be created the first time this is
    /// called, and cleaned up once it returns true.
    /// \return True if completed.
    public: virtual bool Check() = 0;

    /// \brief Check whether to pause this checkpoint and return to the
    /// previous one.
    /// \return True if it should pause.
    public: bool Paused() const;

    /// \brief Check whether the checkpoint has started at least once
    /// \return True if it has started.
    public: bool Started() const;

    /// \brief Call this the first time the checkpoint is checked.
    public: void Start();

    /// \brief Get the current score for this checkpoint.
    /// \return Score
    public: virtual double Score() const;

    /// \brief Get the checkpoint's name
    /// \return Checkpoint's name
    public: std::string Name() const;

    /// \brief Pause the checkpoint, ending the current interval. Only works if
    /// canPause is true.
    protected: void Pause();

    /// \brief Set if this checkpoint is done. It also registers the time when
    /// it was done.
    /// \param[in] _done True if done
    protected: void SetDone(const bool _done);

    /// \brief Get whether this checkpoint is done
    /// \return True if done.
    protected: bool Done() const;

    /// \brief The total penalty for this checkpoint
    protected: double penalty{0.0};

    /// \brief The weight for this checkpoint's time when scoring.
    protected: double weightTime{0.0};

    /// \brief The checkpoint's name
    protected: std::string name;

    /// \brief True if it's possible to restart the checkpoint.
    protected: bool canPause{false};

    /// \brief Vector of sim time intervals when the checkpoint was
    /// running. The first time is the beginning of the interval, the
    /// second is the end
    private: std::vector<std::pair<gazebo::common::Time,
                                   gazebo::common::Time>> intervals;

    /// \brief True when checkpoint is complete.
    private: bool done{false};

    /// \brief True when checkpoint is paused.
    private: bool paused{false};
  };

  /// \brief A checkpoint tied to a gazebo::ContainPlugin.
  class ContainCheckpoint : public Checkpoint
  {
    /// \brief Constructor
    /// \param[in] _sdf SDF element for this checkpoint.
    public: ContainCheckpoint(const sdf::ElementPtr &_sdf);

    /// \brief Check whether the contain checkpoint has been completed.
    /// \return True if completed.
    protected: bool Check() override;

    /// \brief Callback when messages are received from the ContainPlugin.
    /// \param[in] _msg True if contains.
    private: void OnContain(const ignition::msgs::Boolean &_msg);

    /// \brief Callabck for enable service
    /// \param[in] _rep Response
    /// \param[in] _result Result
    private: void EnableCallback(const ignition::msgs::Boolean &_rep,
        const bool _result);

    /// \brief Ignition transport node for communication.
    protected: ignition::transport::Node ignNode;

    /// \brief Namespace for transport
    protected: std::string ns;

    /// \brief True if enabled
    private: bool enabled{false};
  };
}
#endif
