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

#include <ros/ros.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gtest/gtest.h>

bool receivedPoseInfo = false;
gazebo::msgs::PosesStamped lastPoseInfo;

/////////////////////////////////////////////////
void onPoseInfo(ConstPosesStampedPtr &_msg)
{
  lastPoseInfo.CopyFrom(*_msg);
  receivedPoseInfo = true;
}

//////////////////////////////////////////////////
TEST (TrajectoryActorPluginTest, Load)
{
  // Gazebo transport node
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  ASSERT_NE(nullptr, node);
  node->Init();

/*
  // Listen pose/info topic
  auto sub = node->Subscribe("~/pose/info", onPoseInfo);
  ASSERT_NE(nullptr, sub);

  unsigned int sleep = 0;
  unsigned int maxSleep = 30;
  while (!receivedPoseInfo && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(receivedPoseInfo);
  EXPECT_EQ(lastPoseInfo.pose_size(), 34);

  // Check initial actor pose
  bool hasActor{false};
  for (auto pose : lastPoseInfo.pose())
  {
    if (pose.name() == "actor")
    {
      hasActor = true;
      EXPECT_EQ(pose.position().x(), 1);
      EXPECT_EQ(pose.position().y(), 2);
      EXPECT_EQ(pose.position().z(), 3);
      break;
    }
  }
  EXPECT_TRUE(hasActor);

  // World control publiher
  auto worldControlPub = node->Advertise<gazebo::msgs::WorldControl>(
      "/gazebo/default/world_control");
  ASSERT_NE(nullptr, worldControlPub);

  worldControlPub->WaitForConnection();
  EXPECT_TRUE(worldControlPub->HasConnections());

  // Step world
  gazebo::msgs::WorldControl msg;
  msg.set_multi_step(1000);

  worldControlPub->Publish(msg);

  // Check new actor pose
  hasActor = false;
  for (auto pose : lastPoseInfo.pose())
  {
    if (pose.name() == "actor")
    {
      hasActor = true;
      EXPECT_EQ(pose.position().x(), 1);
      EXPECT_EQ(pose.position().y(), 2);
      EXPECT_EQ(pose.position().z(), 3);
      break;
    }
  }
  EXPECT_TRUE(hasActor);
*/
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  testing::InitGoogleTest(&_argc, _argv);

  // Setup a Gazebo client
  gazebo::client::setup(_argc, _argv);

  ros::init(_argc, _argv, "trajectory_actor_plugin-test");
  ros::Time::init();

  return RUN_ALL_TESTS();
}
