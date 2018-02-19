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

bool receivedPoseInfo{false};
gazebo::msgs::PosesStamped lastPoseInfo;

/////////////////////////////////////////////////
void onPose(ConstPosesStampedPtr &_msg)
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

  // Pose subscriber
  auto poseSub = gzNode->Subscribe("~/pose/info", onPose);
  ASSERT_NE(nullptr, poseSub);

  // Actor updates at 30 Hz, so we step forward
  unsigned int sleep = 0;
  unsigned int maxSleep = 300;
  while (!receivedPoseInfo && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(receivedPoseInfo);

  // Check initial pose
  EXPECT_EQ(lastPoseInfo.pose().size(), 34);
  EXPECT_EQ(lastPoseInfo.pose(0).name(), "test-servicebot");
/*
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
*/
  // Cleanup
  poseSub.reset();
  gzNode->Fini();
  gzNode.reset();
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  testing::InitGoogleTest(&_argc, _argv);

  // Setup a Gazebo client
  gazebo::client::setup(_argc, _argv);

  ros::init(_argc, _argv, "spawn_robot-test");
  ros::Time::init();

  return RUN_ALL_TESTS();
}
