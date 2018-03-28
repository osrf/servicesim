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
#include <servicesim_competition/TaskInfo.h>
#include <servicesim_competition/PickUpGuest.h>
#include <servicesim_competition/DropOffGuest.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <cstdlib>

namespace servicesim_test
{
  class ServicesimTasksTest
  {
    public:

      /// \brief Constructor
      ServicesimTasksTest() {};

      /// \brief Destructor
      ~ServicesimTasksTest(){};

      /// \brief Method to move the model to a deisred location and return the status
      bool move_model(ServicesimTasksTest& obj, std::string model_name ,
                      float pos_x, float pos_y, float pos_z,
                      float rot_x, float rot_y, float rot_z);

      /// \brief Method to pickup the guest and return the status
      bool pickup_guest(ServicesimTasksTest& obj);

      /// \brief Method to drop_off the guest and return the status
      bool dropoff_guest(ServicesimTasksTest& obj);

      /// \brief Creating a ROS NodeHandle
      ros::NodeHandle n;

      /// \brief Parameters for the new task
      std::string pick_up_location = "FrontElevator" ;
      std::string drop_off_location = "PrivateCubicle_32_1";
      std::string guest_name = "human_20843";
      std::string robot_name = "servicebot";

      /// \brief Robot Start pose
      float start_pos_x = 11.67;
      float start_pos_y = 3.75;
      float start_pos_z = 0.;
      float start_rot_x = 0;
      float start_rot_y = 0;
      float start_rot_z = 3.14;

      /// \brief Robot pose in the pickup location
      float pickup_pos_x = -0.8;
      float pickup_pos_y = 1.3;
      float pickup_pos_z = 0;
      float pickup_rot_x = 0;
      float pickup_rot_y = 0;
      float pickup_rot_z = 3.14;

      /// \brief Robot pose in the drop-off location
      float drop_off_pos_x = -20.5;
      float drop_off_pos_y = 10.2;
      float drop_off_pos_z = 0;
      float drop_off_rot_x = 0;
      float drop_off_rot_y = 0;
      float drop_off_rot_z = 3.14;

      /// \brief Guest pose in the drop-off location
      float drop_off_guest_pos_x = -19.6;
      float drop_off_guest_pos_y = 9.7;
      float drop_off_guest_pos_z = 1;
      float drop_off_guest_rot_x = 1.54;
      float drop_off_guest_rot_y = 0;
      float drop_off_guest_rot_z = -1.805;
  };
}

using namespace servicesim_test;

//////////////////////////////////////////////////
// Test for new_task
TEST(ServicesimTasksTest, new_task)
{
  // Creating the test object
  servicesim_test::ServicesimTasksTest test_obj;

  // Start new_task service
  ros::ServiceClient new_task_client =
      test_obj.n.serviceClient<servicesim_competition::NewTask>("servicesim/new_task");
  servicesim_competition::NewTask new_task_srv;

  // Waiting for the service to be avaialable
  EXPECT_TRUE(ros::service::waitForService("servicesim/new_task",100000));

  // Calling the service, the service expects empty request
  EXPECT_TRUE(new_task_client.call(new_task_srv));

  // Verifying with the known parameters
  EXPECT_EQ(new_task_srv.response.pick_up_location, test_obj.pick_up_location);
  EXPECT_EQ(new_task_srv.response.drop_off_location, test_obj.drop_off_location);
  EXPECT_EQ(new_task_srv.response.guest_name, test_obj.guest_name);

  // TODO: Could check that score started increasing.

  // Calling the same service again should return false.
  EXPECT_FALSE(new_task_client.call(new_task_srv));
}

//////////////////////////////////////////////////
// Test task_info service
TEST(ServicesimTasksTest, task_info)
{
  // Creating the test object
  servicesim_test::ServicesimTasksTest test_obj;

  // Start task_info service
  ros::ServiceClient task_info_client =
      test_obj.n.serviceClient<servicesim_competition::NewTask>("servicesim/task_info");
  servicesim_competition::NewTask task_info_srv;

  // Waiting for the service to be avaialable
  EXPECT_TRUE(ros::service::waitForService("servicesim/task_info",100000));

  // Calling the service, the service expects empty request
  EXPECT_TRUE(task_info_client.call(task_info_srv));

  // Verifying with the known parameters
  EXPECT_EQ(task_info_srv.response.pick_up_location, test_obj.pick_up_location);
  EXPECT_EQ(task_info_srv.response.drop_off_location, test_obj.drop_off_location);
  EXPECT_EQ(task_info_srv.response.guest_name, test_obj.guest_name);
}

//////////////////////////////////////////////////
// Test for pickup_guest task
TEST(ServicesimTasksTest, pickup_guest)
{
  // Creating the test object
  servicesim_test::ServicesimTasksTest test_obj;

  // Start Pickup task at the start location
  bool pickup_start_flag = false;
  pickup_start_flag = test_obj.pickup_guest(test_obj);

  // Pickup should be unsuccessful
  EXPECT_FALSE(pickup_start_flag);

  // Teleport the robot to the guest pickup location.
  bool spawn_flag = false;
  spawn_flag = test_obj.move_model(test_obj, test_obj.robot_name, test_obj.pickup_pos_x,
                                   test_obj.pickup_pos_y, test_obj.pickup_pos_z,
                                   test_obj.pickup_rot_x, test_obj.pickup_rot_y,
                                   test_obj.pickup_rot_z);

  // Verify that the robot has spawned correctly at the pickup location
  EXPECT_TRUE(spawn_flag);

  // Start Pickup task
  bool pickup_flag = false;
  pickup_flag = test_obj.pickup_guest(test_obj);

  // Verify that the pickup has been successful
  EXPECT_TRUE(pickup_flag);
}

//////////////////////////////////////////////////
// Test for drop_off_guest task (and re-pick_up)
TEST(ServicesimTasksTest, drop_off_guest)
{
  // Creating the test object
  servicesim_test::ServicesimTasksTest test_obj;

  // Start drop_off service at the first pickup location
  bool dropoff_start_flag = false;
  dropoff_start_flag = test_obj.dropoff_guest(test_obj);

  // Verify that the drop_off has been Unsuccessful
  EXPECT_FALSE(dropoff_start_flag);

  // Teleport the robot to the guest drop_off location.
  bool spawn_robot_flag = false;
  spawn_robot_flag = test_obj.move_model(test_obj, test_obj.robot_name, test_obj.drop_off_pos_x,
                                         test_obj.drop_off_pos_y, test_obj.drop_off_pos_z,
                                         test_obj.drop_off_rot_x, test_obj.drop_off_rot_y,
                                         test_obj.drop_off_rot_z);

  // Verify that the robot has spawned correctly at the drop_off location
  EXPECT_TRUE(spawn_robot_flag);

  // Verify that pickup fails if robot is far from guest
  EXPECT_FALSE(test_obj.pickup_guest(test_obj));

  // Teleport the guest to the drop_off_location
  bool spawn_guest_flag = false;
  spawn_guest_flag = test_obj.move_model(test_obj, test_obj.guest_name, test_obj.drop_off_guest_pos_x,
                                         test_obj.drop_off_guest_pos_y, test_obj.drop_off_guest_pos_z,
                                         test_obj.drop_off_guest_rot_x, test_obj.drop_off_guest_rot_y,
                                         test_obj.drop_off_guest_rot_z);

  // Verify that the guest has spawned correctly at the drop_off location
  EXPECT_TRUE(spawn_guest_flag);

  // Restart the pickup task
  bool pickup_flag = false;
  pickup_flag = test_obj.pickup_guest(test_obj);

  // Verify that the pickup has been successful
  EXPECT_TRUE(pickup_flag);

  ros::Duration(0.7).sleep();

  // Start drop_off service
  bool dropoff_flag = false;
  dropoff_flag = test_obj.dropoff_guest(test_obj);

  // Verify that the drop_off has been successful
  EXPECT_TRUE(dropoff_flag);
}

//////////////////////////////////////////////////
// Test for return_to_start task
TEST(ServicesimTasksTest, return_to_start)
{
  // Creating the test object
  servicesim_test::ServicesimTasksTest test_obj;

  // Return to the start location
  // Teleport the robot to the start location.
  bool spawn_robot_start_flag = false;
  spawn_robot_start_flag = test_obj.move_model(test_obj, test_obj.robot_name, test_obj.start_pos_x,
                                               test_obj.start_pos_y, test_obj.start_pos_z,
                                               test_obj.start_rot_x, test_obj.start_rot_y,
                                               test_obj.start_rot_z);

  // Verify that the robot has spawned correctly at the start location
  EXPECT_TRUE(spawn_robot_start_flag);

  // TODO: Could check that score stopped increasing.
}

//////////////////////////////////////////////////
bool ServicesimTasksTest::move_model(ServicesimTasksTest& obj, std::string model_name,
    float pos_x, float pos_y, float pos_z, float rot_x, float rot_y, float rot_z)
{
  // Teleport the model to the given location.
  ros::ServiceClient modelstate_client =
      obj.n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = model_name;
  modelstate.pose.position.x = pos_x;
  modelstate.pose.position.y = pos_y;
  modelstate.pose.position.z = pos_z;
  modelstate.pose.orientation.x = rot_x;
  modelstate.pose.orientation.y = rot_y;
  modelstate.pose.orientation.z = rot_z;

  // Updating the service with the request
  gazebo_msgs::SetModelState setmodelstate_srv;
  setmodelstate_srv.request.model_state = modelstate;
  EXPECT_TRUE(ros::service::waitForService("/gazebo/set_model_state", 10000));

  // Calling the service to set model state
  EXPECT_TRUE(modelstate_client.call(setmodelstate_srv));

  // Return true if the setting state is successfull
  EXPECT_TRUE(setmodelstate_srv.response.success);

  // Verify the position of the model
  ros::ServiceClient modelstate_verify_client =
      obj.n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  gazebo_msgs::GetModelState getmodelstate_srv;
  getmodelstate_srv.request.model_name = model_name;

  bool spawn = false;
  float delta = 0.3;
  int sleep{0};
  int maxSleep{30};

  // Keep checking until the robot is not spawned correctly
  while(!spawn && sleep < maxSleep)
  {
    // Call to get the model state
    EXPECT_TRUE(modelstate_verify_client.call(getmodelstate_srv));
    float x_getmodelstate = getmodelstate_srv.response.pose.position.x;
    float y_getmodelstate = getmodelstate_srv.response.pose.position.y;
    float z_getmodelstate = getmodelstate_srv.response.pose.position.z;
    ros::Duration(0.3).sleep();
    sleep++;

    // Verifying if the model has spawned correctly
    if ((std::abs(x_getmodelstate-pos_x)<delta) &&
        (std::abs(y_getmodelstate-pos_y)<delta) &&
        (std::abs(z_getmodelstate-pos_z)<delta))
    {
      spawn = true;
      break;
    }
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_TRUE(spawn);
  return spawn;
}

//////////////////////////////////////////////////
bool ServicesimTasksTest::pickup_guest(ServicesimTasksTest& obj)
{
  // Start pickup service
  ros::ServiceClient pickup_client =
      obj.n.serviceClient<servicesim_competition::PickUpGuest>("servicesim/pickup_guest");
  servicesim_competition::PickUpGuest pickup_srv;

  // Waiting for the service to be avaialable
  EXPECT_TRUE(ros::service::waitForService("servicesim/pickup_guest", 10000));

  // Updating the service with requests
  pickup_srv.request.guest_name = obj.guest_name;
  pickup_srv.request.robot_name = obj.robot_name;

  // Call the pickup service
  EXPECT_TRUE(pickup_client.call(pickup_srv));

  // Expecting the response success to be true
  return pickup_srv.response.success;
}

//////////////////////////////////////////////////
bool ServicesimTasksTest::dropoff_guest(ServicesimTasksTest& obj)
{
  // Start dropoff_guest service
  ros::ServiceClient dropoff_client =
      obj.n.serviceClient<servicesim_competition::DropOffGuest>("servicesim/dropoff_guest");
  servicesim_competition::DropOffGuest dropoff_srv;

  // Waiting for the service to be avaialable
  EXPECT_TRUE(ros::service::waitForService("servicesim/dropoff_guest", 10000));

  // Updating the service with requests
  dropoff_srv.request.guest_name = obj.guest_name;

  // Call the pickup service
  EXPECT_TRUE(dropoff_client.call(dropoff_srv));

  // Return the status of the drop off
  return dropoff_srv.response.success;
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  testing::InitGoogleTest(&_argc, _argv);

  ros::init(_argc, _argv, "servicesim_tasks-test");

  ros::Time::init();

  return RUN_ALL_TESTS();
}
