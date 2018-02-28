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

#include <gazebo/common/Console.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>

#include "Checkpoint.hh"

using namespace servicesim;

/////////////////////////////////////////////////
Checkpoint::Checkpoint(const sdf::ElementPtr &_sdf)
{
  if (!_sdf)
  {
    gzerr << "Missing checkpoint's SDF element" << std::endl;
    return;
  }

  if (!_sdf->HasElement("weight"))
  {
    gzerr << "Missing checkpoint's <weight> element" << std::endl;
    return;
  }

  auto weightElem = _sdf->GetElement("weight");
  if (!weightElem->HasElement("time"))
  {
    gzerr << "Missing checkpoint's <weight><time> element" << std::endl;
    return;
  }
  this->weightTime = weightElem->Get<double>("time");


  if (!_sdf->HasElement("name"))
  {
    gzerr << "Missing checkpoint's <name> element" << std::endl;
    return;
  }
  this->name = _sdf->Get<std::string>("name");
}

/////////////////////////////////////////////////
double Checkpoint::Score() const
{
  double elapsedSeconds{0.0};

  // Iterate over all intervals
  for (const auto &i : this->intervals)
  {
    auto start = i.first;
    auto end = i.second;

    // If not finished yet
    if (end == gazebo::common::Time::Zero)
    {
      end = gazebo::physics::get_world()->SimTime();
    }

    elapsedSeconds += (end - start).Double();
  }

  return elapsedSeconds * this->weightTime + this->penalty;
}

/////////////////////////////////////////////////
void Checkpoint::Start()
{
  // Check if restarting
  if (!this->canPause && this->intervals.size() > 0)
  {
    gzerr << "It's not possible to restart checkpoint \""
          << this->name << "\"" << std::endl;
    return;
  }

  // Current time
  auto time = gazebo::physics::get_world()->SimTime();
  auto timeStr = time.FormattedString(gazebo::common::Time::HOURS,
                                      gazebo::common::Time::MILLISECONDS);

  // Message
  if (this->intervals.empty())
  {
    gzmsg << "[ServiceSim] Started Checkpoint \"" << this->name << "\" at "
          << timeStr << std::endl;
  }
  else if (this->done || this->paused)
  {
    this->done = false;
    this->paused = false;
    gzmsg << "[ServiceSim] Restarted Checkpoint \"" << this->name << "\" at "
          << timeStr << std::endl;
  }
  else
  {
    gzerr << "Trying to restart checkpoint \"" << this->name
          << "\", which was never completed or paused." << std::endl;
    return;
  }

  // Start new interval
  std::pair<gazebo::common::Time, gazebo::common::Time> interval(
      time, gazebo::common::Time::Zero);
  this->intervals.push_back(interval);
}

/////////////////////////////////////////////////
void Checkpoint::Pause()
{
  if (this->intervals.empty())
  {
    gzerr << "Trying to pause checkpoint which hasn't been started."
          << std::endl;
    return;
  }

  // Get latest checkpoint
  auto &interval = this->intervals.back();
  if (interval.second != gazebo::common::Time::Zero)
  {
    gzerr << "Trying to pause checkpoint which is not running."
          << std::endl;
    return;
  }
  interval.second = gazebo::physics::get_world()->SimTime();

  // Set paused
  this->paused = true;
}

/////////////////////////////////////////////////
bool Checkpoint::Paused() const
{
  if (this->Done())
    return false;

  return this->paused;
}

/////////////////////////////////////////////////
bool Checkpoint::Started() const
{
  return !this->intervals.empty();
}

/////////////////////////////////////////////////
std::string Checkpoint::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
void Checkpoint::SetDone(const bool _done)
{
  // Sanity check
  if (this->done && !_done)
  {
    gzerr << "Can't undo a done checkpoint!" << std::endl;
    return;
  }

  if (!_done)
    return;

  if (this->intervals.empty())
  {
    gzerr << "Can't complete a checkpoint which hasn't started!" << std::endl;
    return;
  }

  // Set end time
  auto &interval = this->intervals.back();
  if (interval.second == gazebo::common::Time::Zero)
  {
    this->done = _done;
    interval.second = gazebo::physics::get_world()->SimTime();
    gzmsg << "[ServiceSim] Checkpoint \"" << this->Name() << "\" complete"
          << std::endl;
  }
}

/////////////////////////////////////////////////
bool Checkpoint::Done() const
{
  return this->done;
}

/////////////////////////////////////////////////
ContainCheckpoint::ContainCheckpoint(const sdf::ElementPtr &_sdf)
    : Checkpoint(_sdf)
{
  if (!_sdf || !_sdf->HasElement("namespace"))
    gzwarn << "Missing <namespace> for contain plugin" << std::endl;
  else
    this->ns = _sdf->Get<std::string>("namespace");
}

/////////////////////////////////////////////////
void ContainCheckpoint::EnableCallback(const ignition::msgs::Boolean &/*_rep*/,
    const bool _result)
{
  if (_result)
    this->enabled = !this->enabled;
}

/////////////////////////////////////////////////
bool ContainCheckpoint::Check()
{
  // First time checking
  if (!this->enabled && !this->Done())
  {
    // Setup contain subscriber
    this->ignNode.Subscribe(this->ns + "/contain",
        &ContainCheckpoint::OnContain, this);

    // Enable contain plugin
    ignition::msgs::Boolean req;
    req.set_data(true);
    this->ignNode.Request(this->ns + "/enable", req,
        &ContainCheckpoint::EnableCallback, this);
  }

  // Done, now clean up
  if (this->enabled && this->Done() && !this->ignNode.SubscribedTopics().empty())
  {
    // Unsubscribe
    for (auto const &sub : this->ignNode.SubscribedTopics())
      this->ignNode.Unsubscribe(sub);

    // Disable contain plugin
    ignition::msgs::Boolean req;
    req.set_data(false);
    this->ignNode.Request(this->ns + "/enable", req,
        &ContainCheckpoint::EnableCallback, this);
  }

  return this->Done();
}

//////////////////////////////////////////////////
void ContainCheckpoint::OnContain(const ignition::msgs::Boolean &_msg)
{
  this->SetDone(_msg.data());
}
