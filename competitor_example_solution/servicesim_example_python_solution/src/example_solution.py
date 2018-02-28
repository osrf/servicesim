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
import random
from enum import Enum

import actionlib

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy

from sensor_msgs.msg import Image

from servicesim_competition.msg import ActorNames
from servicesim_competition.srv import DropOffGuest, DropOffGuestRequest, NewTask
from servicesim_competition.srv import PickUpGuest, PickUpGuestRequest, RoomInfo, RoomInfoRequest

from std_srvs.srv import Empty, EmptyRequest

class CompetitionState(Enum):
    BeginTask = 0
    Pickup = 1
    DropOff = 2
    ReturnToStart = 3


class ExampleNode(object):
    def __init__(self):
        self.robot_name = 'servicebot'
        self.guest_name = ''
        self.action_timeout = 10
        self.actors_in_range = []

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

        # create subscribers
        rospy.loginfo('create subs')
        rospy.Subscriber('/servicebot/rfid', ActorNames, self.rfid_callback)
        rospy.Subscriber('/servicebot/camera_front/image_raw', Image, self.image_callback)
        rospy.loginfo('done creating subs')

        self.initial_pose_pub = rospy.Publisher(
            '/servicebot/initialpose', PoseWithCovarianceStamped, latch=True)
        # create action client
        self.move_base = actionlib.SimpleActionClient('/servicebot/move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(self.action_timeout))
        rospy.loginfo('done waiting for move base action server')

    def rfid_callback(self, msg):
        if msg.actor_names != self.actors_in_range:
            rospy.loginfo(msg.actor_names)
        self.actors_in_range = msg.actor_names
        return True

    def image_callback(self, msg):
        #  placeholder
        return True

    def request_new_task(self):
        resp = self.new_task_srv()
        rospy.loginfo(resp)
        return [
            resp.pick_up_location, resp.drop_off_location, resp.guest_name, resp.robot_start_pose]

    def request_follow(self):
        req = PickUpGuestRequest()
        req.robot_name = self.robot_name
        req.guest_name = self.guest_name
        resp = self.pickup_guest_srv(req)
        rospy.loginfo(resp)
        return [resp.success]

    def request_unfollow(self):
        req = DropOffGuestRequest()
        req.guest_name = self.guest_name
        resp = self.dropoff_guest_srv(req)
        rospy.loginfo(resp)
        return [resp.success]

    # returns a target pose from the room name
    def pose_from_room_name(self, room_name):
        req = RoomInfoRequest()
        req.name = room_name
        resp = self.room_info_srv(req)

        # To be replaced by competitors
        # here we just return the center of the provided area
        mid_pose = Pose(
            Point(
                (resp.min.x + resp.max.x) / 2., (resp.min.y + resp.max.y) / 2., 0),
            Quaternion(0, 0, 1.0, 0.0))
        return mid_pose

    def construct_goal_from_pose(self, pose):
        posecopy = copy.deepcopy(pose)
        goal = MoveBaseGoal()
        goal.target_pose.pose = posecopy
        goal.target_pose.pose.position.x -= 1
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # add randomness (square of 10cm) around the goal
        dx = random.randrange(0, 20, 1) - 10
        dy = random.randrange(0, 20, 1) - 10
        goal.target_pose.pose.position.x += dx / 100.
        goal.target_pose.pose.position.y += dy / 100.
        return goal

    def example_solution(self):
        state = CompetitionState.BeginTask

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            if state == CompetitionState.BeginTask:
                rospy.loginfo('In BeginTask state')
                [pick_up_room, drop_off_room, self.guest_name, self.start_pose] = \
                    self.request_new_task()
                self.start_pose.position.z = 0
                init_pose_msg = PoseWithCovarianceStamped()
                init_pose_msg.header.frame_id = 'map'
                init_pose_msg.pose.pose = self.start_pose
                self.initial_pose_pub.publish(init_pose_msg)
                rospy.loginfo('published initial pose')
                rate.sleep()

                clear_costmaps_srv = rospy.ServiceProxy(
                    '/servicebot/move_base/clear_costmaps', Empty)
                clear_costmaps_srv(EmptyRequest())
                state = CompetitionState.Pickup

            elif state == CompetitionState.Pickup:
                rospy.loginfo('In Pickup state')
                pickup_goal = self.construct_goal_from_pose(self.pose_from_room_name(pick_up_room))
                self.move_base.send_goal(pickup_goal)
                rospy.loginfo('sent goal')

                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))
                rospy.loginfo('action result came in')
                rospy.loginfo(
                    'got result: Action state is: %s\n success is %d' %
                    (self.move_base.get_state(), GoalStatus.SUCCEEDED))
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    if self.guest_name in self.actors_in_range:
                        rospy.loginfo('requesting unfollow')
                        if self.request_follow():
                            state = CompetitionState.DropOff
                    else:
                        # pickup point reached but actor not in range
                        # add custom room exploration logic here
                        rospy.logwarn('robot reached pickup point but guest is not in range!')
                else:
                    rospy.logwarn('action failed!')

            elif state == CompetitionState.DropOff:
                rospy.loginfo('In DropOff state')
                dropoff_goal = self.construct_goal_from_pose(
                    self.pose_from_room_name(drop_off_room))
                self.move_base.send_goal(dropoff_goal)
                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    self.request_unfollow()
                    state = CompetitionState.ReturnToStart
                else:
                    rospy.logerr('action timed out in state: %s' % self.move_base.get_state())

            elif state == CompetitionState.ReturnToStart:
                rospy.loginfo('In ReturnToStart state')
                startgoal = self.construct_goal_from_pose(self.start_pose)
                self.move_base.send_goal(startgoal)
                self.move_base.wait_for_result(rospy.Duration(self.action_timeout))
                if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo('Hooray!')
                    rospy.signal_shutdown('Competition completed!')
                else:
                    rospy.logerr('action timed out in state: %s' % self.move_base.get_state())

            rate.sleep()


if __name__ == '__main__':
    node = ExampleNode()
    try:
        node.example_solution()
    except rospy.ROSInterruptException:
        pass
