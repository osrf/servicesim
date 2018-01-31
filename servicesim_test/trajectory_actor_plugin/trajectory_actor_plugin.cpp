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
gazebo::msgs::PoseAnimation lastPoseInfo;

/////////////////////////////////////////////////
void onSkeletonPose(ConstPoseAnimationPtr &_msg)
{
  lastPoseInfo.CopyFrom(*_msg);
  receivedPoseInfo = true;
}

//////////////////////////////////////////////////
TEST (TrajectoryActorPluginTest, Load)
{
  // Gazebo transport node
  gazebo::transport::NodePtr gzNode(new gazebo::transport::Node());
  ASSERT_NE(nullptr, gzNode);
  gzNode->Init();

  // Skeleton pose subscriber
  auto skeletonPoseSub = gzNode->Subscribe("~/skeleton_pose/info",
      onSkeletonPose);
  ASSERT_NE(nullptr, skeletonPoseSub);

  // World control publisher
  auto worldControlPub = gzNode->Advertise<gazebo::msgs::WorldControl>(
      "/gazebo/default/world_control");
  ASSERT_NE(nullptr, worldControlPub);
  worldControlPub->WaitForConnection();
  EXPECT_TRUE(worldControlPub->HasConnections());

  // Actor updates at 30 Hz, so we step forward
  unsigned int sleep = 0;
  unsigned int maxSleep = 300;
  while (!receivedPoseInfo && sleep < maxSleep)
  {
    gazebo::msgs::WorldControl msg;
    msg.set_multi_step(1);
    worldControlPub->Publish(msg);
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(receivedPoseInfo);

  // Check initial pose
  EXPECT_EQ(lastPoseInfo.model_name(), "actor::actor_pose::actor_visual");
  EXPECT_EQ(lastPoseInfo.pose().size(), 63);
  EXPECT_EQ(lastPoseInfo.time().size(), 1);
  EXPECT_EQ(lastPoseInfo.time(0).sec(), 0);
  EXPECT_GT(lastPoseInfo.time(0).nsec(), 0);

  ignition::math::Vector3d initialPos;
  for (auto pose : lastPoseInfo.pose())
  {
    if (pose.name() == "actor")
    {
      initialPos = gazebo::msgs::ConvertIgn(pose.position());
      break;
    }
  }
  EXPECT_NE(initialPos, ignition::math::Vector3d::Zero);
  EXPECT_NEAR(initialPos.X(), 1.0, 0.05);
  EXPECT_NEAR(initialPos.Y(), 2.0, 0.05);
  EXPECT_NEAR(initialPos.Z(), 0.0, 0.05);

  // Step until at least 3 seconds sim time
  sleep = 0;
  maxSleep = 300;
  while (lastPoseInfo.time(0).sec() < 3 && sleep < maxSleep)
  {
    gazebo::msgs::WorldControl msg;
    msg.set_multi_step(1000);
    worldControlPub->Publish(msg);
    gazebo::common::Time::MSleep(10);
    sleep++;
  }
  EXPECT_GE(lastPoseInfo.time(0).sec(), 3);

  // Check actor has moved
  ignition::math::Vector3d newPos;
  for (auto pose : lastPoseInfo.pose())
  {
    if (pose.name() == "actor")
    {
      newPos = gazebo::msgs::ConvertIgn(pose.position());
      break;
    }
  }
  EXPECT_NE(newPos, ignition::math::Vector3d::Zero);
  EXPECT_NEAR(newPos.Z(), 0.0, 0.05);

  // Expected new position based on default velocity
  const double velocity{0.8};
  auto duration = gazebo::msgs::Convert(lastPoseInfo.time(0));
  auto expectedDist = velocity * duration.Double();

  auto realDist = (newPos - initialPos).Length();

  EXPECT_NEAR(expectedDist, realDist, 0.006);

  // Cleanup
  skeletonPoseSub.reset();
  worldControlPub.reset();
  gzNode->Fini();
  gzNode.reset();
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
