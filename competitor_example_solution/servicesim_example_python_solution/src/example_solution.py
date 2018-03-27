#! /usr/bin/env python

# Copyright (C) 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
import math
import random
from enum import Enum

import actionlib

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy

from servicesim_competition.msg import ActorNames
from servicesim_competition.srv import DropOffGuest, DropOffGuestRequest, NewTask
from servicesim_competition.srv import PickUpGuest, PickUpGuestRequest, RoomInfo, RoomInfoRequest

from std_srvs.srv import Empty, EmptyRequest

from servicesim_example_python_solution.msg import Contour

import tf
import tf2_ros


class CompetitionState(Enum):
    BeginTask = 0
    Pickup = 1
    DropOff = 2
    ReturnToStart = 3
    RePickup = 4


class ExampleNode(object):
    def __init__(self):
        self.robot_name = 'servicebot'
        self.guest_name = ''
        self.action_timeout = 6
        self.actors_in_range = []
        self.state = CompetitionState.BeginTask
        self.distance = 0
        self.center_bbox = 0
        self.new_pickup_goal_set = False
        self.last_rfid_update = None
        self.thresh_distance = 1.6    #  the distance from guest where the new pickup_goal is generated.
        self.transform_flag = False

        rospy.init_node('example_solution')
        rospy.loginfo('node created')
        # create service proxies
        rospy.loginfo('wait for new task srv')
        rospy.wait_for_service('/servicesim/new_task')
        self.new_task_srv = rospy.ServiceProxy('/servicesim/new_task', NewTask)
        rospy.loginfo('wait for pickup guest srv')
        rospy.wait_for_service('/servicesim/pickup_guest')
        self.pickup_guest_srv = rospy.ServiceProxy('/servicesim/pickup_guest', PickUpGuest)
        rospy.loginfo('wait for drop off srv')
        rospy.wait_for_service('/servicesim/dropoff_guest')
        self.dropoff_guest_srv = rospy.ServiceProxy('/servicesim/dropoff_guest', DropOffGuest)
        rospy.loginfo('wait for room info srv')
        rospy.wait_for_service('/servicesim/room_info')
        self.room_info_srv = rospy.ServiceProxy('/servicesim/room_info', RoomInfo)
        # create a service proxy to clear the navigation costmaps
        rospy.loginfo('wait for clear_costmaps')
        rospy.wait_for_service('/servicebot/move_base/clear_costmaps')
        self.clear_costmaps_srv = rospy.ServiceProxy(
            '/servicebot/move_base/clear_costmaps', Empty)

        # create subscribers
        rospy.loginfo('create subs')
        rospy.Subscriber('/servicebot/rfid', ActorNames, self.rfid_callback)
        rospy.Subscriber('/servicebot/bbox_distance', Contour, self.get_new_pickup_distance_callback)
        rospy.loginfo('done creating subs')

        # create publisher
        # create a publisher to initialize the localization with the starting pose
        # provided by the competition
        self.initial_pose_pub = rospy.Publisher(
            '/servicebot/initialpose', PoseWithCovarianceStamped, latch=True)
        # creating a publisher to publish cmd_vel to the robot
        self.cmd_vel_pub = rospy.Publisher('/servicebot/cmd_vel', Twist, queue_size=1)
        # create action client
        self.move_base = actionlib.SimpleActionClient('/servicebot/move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(self.action_timeout))
        rospy.loginfo('done waiting for move base action server')

    # This function will be called periodically with the list of RFID tags
    # detected around the robot
    def rfid_callback(self, msg):
        # Print when there is a change
        if msg.actor_names != self.actors_in_range:
            rospy.loginfo(msg.actor_names)

        self.actors_in_range = msg.actor_names
        self.last_rfid_update = rospy.get_time()
        # Cancel navigation in case of drift during drop-off
        if self.state == CompetitionState.DropOff and self.guest_name not in self.actors_in_range:
            self.move_base.cancel_all_goals()


        return True

    # Calls the service to start the competition and receive a new task
    def request_new_task(self):
        resp = self.new_task_srv()
        rospy.loginfo(resp)
        return [
            resp.pick_up_location, resp.drop_off_location, resp.guest_name, resp.robot_start_pose]

    # Calls the service to request a specific guest to follow the robot
    # And if successful sets the state to DropOff
    def request_follow(self):
        req = PickUpGuestRequest()
        req.robot_name = self.robot_name
        req.guest_name = self.guest_name
        resp = self.pickup_guest_srv(req)
        rospy.loginfo(resp)
        return [resp.success]

    # Calls the service to request a specific guest to stop followingthe robot
    def request_unfollow(self):
        req = DropOffGuestRequest()
        req.guest_name = self.guest_name
        resp = self.dropoff_guest_srv(req)
        rospy.loginfo(resp)
        return [resp.success]

    # Returns a target pose from the room name
    def pose_from_room_name(self, room_name):
        # Calls the service to get pose infromations based on a room name
        req = RoomInfoRequest()
        req.name = room_name
        resp = self.room_info_srv(req)

        # COMPETITOR: replace with custom logic,
        # here we just return the center of the provided area
        mid_pose = Pose(
            Point(
                (resp.min.x + resp.max.x) / 2., (resp.min.y + resp.max.y) / 2., 0),
            Quaternion(0, 0, 1.0, 0.0))
        return mid_pose

    # Takes a 6D pose and returns a navigation goal
    # COMPETITOR: replace with custom logic
    def construct_goal_from_pose(self, pose):
        posecopy = copy.deepcopy(pose)
        goal = MoveBaseGoal()
        goal.target_pose.pose = posecopy
        # As we need the guest to be at the target location and not the robot,
        # we offset every goal of 1 meter
        goal.target_pose.pose.position.x -= 1
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # add randomness (square of 10cm) around the goal
        dx = random.randrange(0, 20, 1) - 10
        dy = random.randrange(0, 20, 1) - 10
        goal.target_pose.pose.position.x += dx / 100.
        goal.target_pose.pose.position.y += dy / 100.
        return goal

    # Requested for new pickup goal
    # COMPETITOR: replace with custom logic
    def construct_new_pickup_goal(self):
        # Locate the guest by rotating until the guest is in the center of the
        # front camera
        rospy.loginfo('Rotating to find guest...')
        cmd_vel_msg = Twist()
        while not 310 < self.center_bbox < 330:
            # turn around until the X_coordinate of center of bbox  \
            # lies within a range of the X_coordinate of center of the image
            cmd_vel_msg.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd_vel_msg)
        rospy.loginfo('Guest located')
        cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel_msg)

        rospy.loginfo("Constructing new goal")
        tflistener = tf.TransformListener()
        r = rospy.Rate(10)
        for i in range(100):
            now = rospy.Time(0)
            try:
                tflistener.waitForTransform('map', 'base_footprint', now, rospy.Duration(5))
                (trans, rot) = tflistener.lookupTransform('map', 'base_footprint', now)
                self.transform_flag = True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                continue
        # if the transform if found
        if self.transform_flag:
            quaternion = (rot[0], rot[1], rot[2], rot[3])
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            if(self.distance):
                d = self.distance - self.thresh_distance   # thresh_distance away from the guest
                x_delta = d*math.cos(yaw)
                y_delta = d*math.sin(yaw)
            new_pickup_goal_pose = Pose()
            new_pickup_goal_pose.position = Point(trans[0] + x_delta, trans[1] + y_delta, 0)
            new_pickup_goal_pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])
            rospy.loginfo("Sending new goal:")
            rospy.loginfo(new_pickup_goal_pose)
            new_goal = MoveBaseGoal()
            new_goal.target_pose.pose = new_pickup_goal_pose
            new_goal.target_pose.header.frame_id = 'map'
            new_goal.target_pose.header.stamp = rospy.Time.now()
            self.new_pickup_goal_set = True
            self.transform_flag = False
            return new_goal
        else:
            rospy.loginfo(" Robot transform to map not found")
            return None

    def get_new_pickup_distance_callback(self, msg):
        self.center_bbox = msg.bbox.center.x
        self.distance = msg.distance

    # This handles the competition logic
    # A state maching that gives goals to the robot and calls the competition services
    # to complete the checkpoints
    def example_solution(self):
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            if self.state == CompetitionState.BeginTask:
                rospy.loginfo('-- State: BeginTask')
                # Request a new task
                [pick_up_room, drop_off_room, self.guest_name, self.start_pose] = \
                    self.request_new_task()
                self.start_pose.position.z = 0
                # Initialize the localization with the provided starting position
                init_pose_msg = PoseWithCovarianceStamped()
                init_pose_msg.header.frame_id = 'map'
                init_pose_msg.pose.pose = self.start_pose
                self.initial_pose_pub.publish(init_pose_msg)
                rospy.loginfo('published initial pose')
                rospy.sleep(0.5)

                # Switch to the Pickup state
                self.state = CompetitionState.Pickup

            elif self.state == CompetitionState.Pickup:
                # Go to pick up point
                rospy.loginfo('-- State: Pickup')
                # Cancel previous navigation goals
                self.move_base.cancel_all_goals()
                # Clears the obstacle maps
                self.clear_costmaps_srv(EmptyRequest())
                rospy.sleep(0.5)
                pickup_goal = self.construct_goal_from_pose(self.pose_from_room_name(pick_up_room))
                # Ask the robot to go to the pickup location by sending it a navigation goal
                self.move_base.send_goal(pickup_goal)
                rospy.loginfo('Sent pick-up goal')

                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))

                # If the robot succeeded to reach the pickup point
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    # check that the guest is in range
                    if self.guest_name in self.actors_in_range:
                        rospy.loginfo('Requesting follow')
                        # if the guest is in range, ask the guest to follow the robot
                        if self.request_follow():
                            # The guest is following the robot
                            # Switch to the DropOff state
                            self.state = CompetitionState.DropOff
                    else:
                        # pickup point reached but the guest is not in range
                        # COMPETITOR: add custom room exploration logic here
                        rospy.logwarn('robot reached pickup point but guest is not in range!')
                else:
                    rospy.loginfo('Action failed in state: %s' % self.move_base.get_state())

            # When the guest is lost or drifted away
            elif self.state == CompetitionState.RePickup:
                # Change the goals, pickup guest and then dropoff
                rospy.loginfo('-- State: RePickup')
                # Cancel the previous goals
                self.move_base.cancel_all_goals()
                # Clears the obstacle maps
                self.clear_costmaps_srv(EmptyRequest())
                rospy.sleep(0.5)
                new_pickup_goal = self.construct_new_pickup_goal()
                self.move_base.send_goal(new_pickup_goal)
                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))
                # If the robot succeeded to reach the new pickup point
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    # if the robot succeeded to reach the pickup point
                    if self.guest_name in self.actors_in_range:
                        rospy.loginfo('Requesting follow')
                        # if the guest is in range, ask the guest to follow the robot
                        if self.request_follow():
                            # The guest is following the robot
                            # Switch to the DropOff state
                            self.state = CompetitionState.DropOff
                    else:
                        rospy.logwarn('robot reached new pickup point but guest is not in range! ')
                else:
                    rospy.loginfo('Action failed in state: %s' % self.move_base.get_state())

            elif self.state == CompetitionState.DropOff:
                # Go to drop off point
                rospy.loginfo('-- State: DropOff')
                # If guest drifted, or when the rfid is not updated because there is no one in range for greater than 2secs
                if self.guest_name not in self.actors_in_range or (rospy.get_time()-self.last_rfid_update) > 2:
                    rospy.loginfo('Guest out of range')
                    self.state = CompetitionState.RePickup
                    continue
                # Cancel previous navigation goals
                self.move_base.cancel_all_goals()
                # Clears the obstacle maps
                self.clear_costmaps_srv(EmptyRequest())
                rospy.sleep(0.5)
                # Ask the robot to go to the drop off location by sending it a navigation goal
                dropoff_goal = self.construct_goal_from_pose(
                    self.pose_from_room_name(drop_off_room))
                self.move_base.send_goal(dropoff_goal)
                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))
                # If the robot succeeded to reach the dropoff point
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    # ask the guest to stop following the robot
                    self.request_unfollow()
                    self.state = CompetitionState.ReturnToStart
                else:
                    rospy.loginfo('Action failed in state: %s' % self.move_base.get_state())

            elif self.state == CompetitionState.ReturnToStart:
                # Go to drop off point
                rospy.loginfo('-- State: ReturnToStart')
                # Cancel previous navigation goals
                self.move_base.cancel_all_goals()
                # Clears the obstacle maps
                self.clear_costmaps_srv(EmptyRequest())
                rospy.sleep(0.5)
                # Ask the robot to return to the starting position
                startgoal = self.construct_goal_from_pose(self.start_pose)
                self.move_base.send_goal(startgoal)
                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    # If the robot succeeded to reach the starting point
                    # the competition is over
                    rospy.loginfo('Hooray!')
                    rospy.signal_shutdown('Competition completed!')
                else:
                    rospy.loginfo('Action failed in state: %s' % self.move_base.get_state())

            rate.sleep()


if __name__ == '__main__':
    node = ExampleNode()
    try:
        node.example_solution()
    except rospy.ROSInterruptException:
        pass
