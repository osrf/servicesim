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

#include "PenaltyChecker.hh"

using namespace servicesim;

/////////////////////////////////////////////////
PenaltyChecker::PenaltyChecker(const sdf::ElementPtr &_sdf)
{
  if (!_sdf)
  {
    gzerr << "Missing SDF element" << std::endl;
    return;
  }

  if (!_sdf->HasElement("weight"))
  {
    gzerr << "Missing top-level <weight> element" << std::endl;
    return;
  }

  auto weightElem = _sdf->GetElement("weight");
  if (!weightElem->HasElement("human_contact"))
  {
    gzerr << "Missing top-level <weight><human_contact> element" << std::endl;
    return;
  }
  this->weightHumanContact = weightElem->Get<double>("human_contact");

  if (!weightElem->HasElement("obj_contact"))
  {
    gzerr << "Missing top-level <weight><obj_contact> element" << std::endl;
    return;
  }
  this->weightObjContact = weightElem->Get<double>("obj_contact");

  if (!weightElem->HasElement("human_approximation"))
  {
    gzerr << "Missing top-level <weight><human_approximation> element" << std::endl;
    return;
  }
  this->weightHumanApproximation = weightElem->Get<double>("human_approximation");

  if (!weightElem->HasElement("obj_approximation"))
  {
    gzerr << "Missing top-level <weight><obj_approximation> element" << std::endl;
    return;
  }
  this->weightObjApproximation = weightElem->Get<double>("obj_approximation");

  this->robotName = _sdf->Get<std::string>("robot_name");
  this->groundName = _sdf->Get<std::string>("ground_name");
  this->humanName = _sdf->Get<std::string>("human_name");

  // Gazebo transport
  this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gzNode->Init();

  this->contactsSub = this->gzNode->Subscribe("~/physics/contacts",
      &PenaltyChecker::OnContacts, this);
}

/////////////////////////////////////////////////
PenaltyChecker::~PenaltyChecker()
{
  this->contactsSub.reset();
  this->gzNode->Fini();
}

/////////////////////////////////////////////////
void PenaltyChecker::OnContacts(ConstContactsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    auto contact = _msg->contact(i);

    // Skip if robot is not in contact
    if (contact.collision1().find(this->robotName) == std::string::npos &&
        contact.collision2().find(this->robotName) == std::string::npos)
    {
      continue;
    }

    // Skip if it is contact with the ground
    if (contact.collision1().find(this->groundName) != std::string::npos ||
        contact.collision2().find(this->groundName) != std::string::npos)
    {
      continue;
    }

    // Human or object
    bool human{false};
    if (contact.collision1().find(this->humanName) != std::string::npos ||
        contact.collision2().find(this->humanName) != std::string::npos)
    {
      human = true;
    }

    // Approximation
    bool approximation{false};
    if (contact.collision1().find("inflation_people") != std::string::npos ||
        contact.collision2().find("inflation_people") != std::string::npos)
    {
      if (!human)
        continue;

      approximation = true;
    }

    if (contact.collision1().find("inflation_obj") != std::string::npos ||
        contact.collision2().find("inflation_obj") != std::string::npos)
    {
      if (human)
        continue;

      approximation = true;
    }

    // Choose weight
    double weight{0.0};

    if (human)
    {
      if (approximation)
        weight = this->weightHumanApproximation;
      else
        weight = this->weightHumanContact;
    }
    else
    {
      if (approximation)
        weight = this->weightObjApproximation;
      else
        weight = this->weightObjContact;
    }

    // In case of multiple contact points, take highest depth
    double depth{0.0};
    for (int d = 0; d < contact.depth_size(); ++d)
    {
      depth = std::max(depth, contact.depth(d));
    }

    // Penalty
    auto p = weight * depth;
    this->penalty += p;

    // Message
    // Commenting out because it's too spammy, consider adding a flag or a
    // topic to debug this
/*
    std::stringstream msg;
    msg << "[ServiceSim] " << p << " penalty: ";

    if (approximation)
      msg << "too close to ";
    else
      msg << "collided with ";

    if (human)
      msg << "human";
    else
      msg << "object";

    gzmsg << msg.str() << std::endl;
*/
  }
}

/////////////////////////////////////////////////
double PenaltyChecker::Penalty() const
{
  return this->penalty;
}
