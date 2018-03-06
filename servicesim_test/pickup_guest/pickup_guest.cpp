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
#include <servicesim_competition/PickUpGuest.h>
#include <servicesim_competition/RoomInfo.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

TEST(PickUpGuestTest, Call)
{

  ros::NodeHandle n;

  ros::ServiceClient new_task_client = n.serviceClient<servicesim_competition::NewTask>("servicesim/new_task");
  servicesim_competition::NewTask new_task_srv;
  EXPECT_TRUE(ros::service::waitForService("servicesim/new_task",100000));
  EXPECT_TRUE(new_task_client.call(new_task_srv));
  std::string pick_up_location = "FrontElevator" ;
  std::string drop_off_location = "PrivateCubicle_32_1";
  std::string guest_name = "human_86138";

  EXPECT_EQ(new_task_srv.response.pick_up_location, pick_up_location);
  EXPECT_EQ(new_task_srv.response.drop_off_location, drop_off_location);
  EXPECT_EQ(new_task_srv.response.guest_name, guest_name);
  //
  // Get the room Info
  ros::ServiceClient room_info_client = n.serviceClient<servicesim_competition::RoomInfo>("servicesim/room_info");
  servicesim_competition::RoomInfo room_info_srv;
  EXPECT_TRUE(ros::service::waitForService("servicesim/room_info",10000));
  float x_min = room_info_srv.response.min.x;
  float y_min = room_info_srv.response.min.y;
  float x_max = room_info_srv.response.max.x;
  float y_max = room_info_srv.response.max.y;

  // Choosing a position in the pickup location for robot to Teleport
  float x = -0.8;
  float y = 1.3;
  float z = 0;

  // Teleport the robot to the guest location.
  ros::ServiceClient modelstate_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::ModelState modelstate;
  gazebo_msgs::SetModelState setmodelstate_srv;
  modelstate.model_name = "servicebot";
  modelstate.pose.position.x = x;
  modelstate.pose.position.y = y;
  modelstate.pose.position.z = z;

  // Updating the service with the request
  setmodelstate_srv.request.model_state = modelstate;
  EXPECT_TRUE(ros::service::waitForService("/gazebo/set_model_state", 10000));
  // Calling the service to set model state
  EXPECT_TRUE(modelstate_client.call(setmodelstate_srv));

  // Return true if the setting state is successfull
  EXPECT_TRUE(setmodelstate_srv.response.success);

  //verify the position of the robot
  ros::ServiceClient modelstate_verify_client = n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  gazebo_msgs::GetModelState getmodelstate_srv;
  getmodelstate_srv.request.model_name = "servicebot";

  bool spawn_flag = false;
  float delta = 0.3;
  int count = 0; // to keep track of the iterations

  while(spawn_flag == false && count <30)
  {
  // call to verify the position
  EXPECT_TRUE(modelstate_verify_client.call(getmodelstate_srv));
  float x_getmodelstate = getmodelstate_srv.response.pose.position.x;
  float y_getmodelstate = getmodelstate_srv.response.pose.position.y;
  float z_getmodelstate = getmodelstate_srv.response.pose.position.z;
  count ++;
  // verify for the robot to reach the specified location and make sure it is still there with count parameter
  if (((x_getmodelstate-x)<delta) && ((y_getmodelstate-y)<delta) && ((z_getmodelstate-z)<delta) && count >15)
  {
    spawn_flag = true;
    // Pickup service
    ros::ServiceClient pickup_client= n.serviceClient<servicesim_competition::PickUpGuest>("servicesim/pickup_guest");
    servicesim_competition::PickUpGuest pickup_srv;
    EXPECT_TRUE(ros::service::waitForService("servicesim/pickup_guest", 10000));
    // Updating the service with requests
    pickup_srv.request.guest_name = guest_name;
    pickup_srv.request.robot_name = "servicebot";
    // Call the pickup service
    EXPECT_TRUE(pickup_client.call(pickup_srv));
    // Expecting the response success to be true
    EXPECT_TRUE(pickup_srv.response.success);
  }

  ros::Duration(0.3).sleep();
}

}

int main(int _argc, char **_argv)
{

  testing::InitGoogleTest(&_argc, _argv);

  ros::init(_argc, _argv, "pickup_guest-test");

  ros::Time::init();

  return RUN_ALL_TESTS();
}
