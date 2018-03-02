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
#include <gtest/gtest.h>
#include <servicesim_competition/NewTask.h>

TEST(NewTaskTest, Call)
{
  // Creating ROS node handle
  ros::NodeHandle n;
  // Client for the `servicesim/new_task` service
  ros::ServiceClient client= n.serviceClient<servicesim_competition::NewTask>("servicesim/new_task");
  servicesim_competition::NewTask srv;
  // Waiting for the service to be avaialable
  EXPECT_TRUE(ros::service::waitForService("servicesim/new_task",100000));
  // Calling the service, the service expects empty request
  EXPECT_TRUE(client.call(srv));
  // Checking for the desired response
  EXPECT_EQ(srv.response.pick_up_location, "FrontElevator");
  EXPECT_EQ(srv.response.drop_off_location, "PrivateCubicle_32_1");
  EXPECT_EQ(srv.response.guest_name, "human_86138");
  // Calling the same service again should return false.
  EXPECT_FALSE(client.call(srv));
}

int main(int _argc, char **_argv)
{

  testing::InitGoogleTest(&_argc, _argv);

  ros::init(_argc, _argv, "new_task-test");

  ros::Time::init();

  return RUN_ALL_TESTS();
}
