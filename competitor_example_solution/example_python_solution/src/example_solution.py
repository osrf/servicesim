#! /usr/bin/env python
# from __future__ import print_function

from enum import Enum

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Point, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from servicesim_competition.srv import DropOffGuest, DropOffGuestRequest, NewTask
from servicesim_competition.srv import PickUpGuest, PickUpGuestRequest
import rospy
# TODO(mikaelarguedas): use response field values for robot name and guest name + refactor waypoints


class CompetitionState(Enum):
    BeginTask = 0
    Pickup = 1
    DropOff = 2
    ReturnToStart = 3


def request_new_task():
    rospy.loginfo('waiting for new task service')
    rospy.wait_for_service('/servicesim/new_task')
    rospy.loginfo('requesting new task')
    new_task = rospy.ServiceProxy('/servicesim/new_task', NewTask)
    resp = new_task()
    rospy.loginfo(resp)
    # print(resp)
    return True


def request_follow():
    rospy.loginfo('waiting for pickup_guest service')
    rospy.wait_for_service('/servicesim/pickup_guest')
    rospy.loginfo('requesting guest to follow the robot')
    pickup_guest = rospy.ServiceProxy('/servicesim/pickup_guest', PickUpGuest)
    req = PickUpGuestRequest()
    req.guest_name = 'guest'
    req.robot_name = 'servicebot'
    resp = pickup_guest(req)
    # print(resp)
    return True


def request_unfollow():
    rospy.loginfo('waiting for dropoff_guest service')
    rospy.wait_for_service('/servicesim/dropoff_guest')
    rospy.loginfo('requesting guest to stop following the robot')
    dropoff_guest = rospy.ServiceProxy('/servicesim/dropoff_guest', DropOffGuest)
    req = DropOffGuestRequest()
    req.guest_name = 'guest'
    resp = dropoff_guest(req)
    # print(resp)
    return True


def example_solution():
    move_base = actionlib.SimpleActionClient('/servicebot/move_base', MoveBaseAction)

    action_timeout = 60

    move_base.wait_for_server(rospy.Duration(action_timeout))

    # Set up the goal location
    start = MoveBaseGoal()
    start.target_pose.pose = Pose(Point(3.7, 1.23, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
    start.target_pose.header.frame_id = 'map'
    pickup = MoveBaseGoal()
    pickup.target_pose.pose = Pose(Point(-6.2, 1.9, 0.000), Quaternion(0.000, 0.000, 1.000, 0.000))
    pickup.target_pose.header.frame_id = 'map'
    dropoff_door = MoveBaseGoal()
    dropoff_door.target_pose.pose = Pose(
        Point(-3.764, 6.893, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
    dropoff_door.target_pose.header.frame_id = 'map'
    dropoff_in = MoveBaseGoal()
    # dropoff_in.target_pose.pose = Pose(
    #     Point(-2.468, 9.061, 0.000), Quaternion(0.000, 0.000, 0.896, -0.444))
    dropoff_in.target_pose.pose = Pose(
        Point(-2.468, 9.061, 0.000), Quaternion(0.000, 0.000, 0.000, 1.0))
    dropoff_in.target_pose.header.frame_id = 'map'
    dropoff_around_guest = MoveBaseGoal()
    dropoff_around_guest.target_pose.pose = Pose(
        Point(-2.832, 6.134, 0.000), Quaternion(0.000, 0.000, 0.987, 0.163))
    dropoff_around_guest.target_pose.header.frame_id = 'map'
    dropoff_goals = [dropoff_door, dropoff_in, dropoff_around_guest]

    state = CompetitionState.BeginTask
    goal_nb = 0

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if state == CompetitionState.BeginTask:
            rospy.loginfo('In BeginTask state')
            if request_new_task():
                state = CompetitionState.Pickup

        elif state == CompetitionState.Pickup:
            rospy.loginfo('In Pickup state')
            pickup.target_pose.header.stamp = rospy.Time.now()
            move_base.send_goal(pickup)
            # rospy.loginfo('sending goal: %s' % pickup)
            move_base.wait_for_result(rospy.Duration(action_timeout))
            rospy.loginfo(
                'got result: Action state is: %s\n success is %d' %
                (move_base.get_state(), GoalStatus.SUCCEEDED))
            if move_base.get_state() == GoalStatus.SUCCEEDED:
                if request_follow():
                    state = CompetitionState.DropOff

        elif state == CompetitionState.DropOff:
            rospy.loginfo('In DropOff state')
            rospy.loginfo('Sending goal #%d' % goal_nb)
            move_base.send_goal(dropoff_goals[goal_nb])
            move_base.wait_for_result(rospy.Duration(action_timeout))
            if move_base.get_state() == GoalStatus.SUCCEEDED:
                goal_nb += 1
                if goal_nb == len(dropoff_goals) - 1:
                    request_unfollow()
                elif goal_nb == len(dropoff_goals):
                    state = CompetitionState.ReturnToStart
            else:
                rospy.logerr('action timed out in state: %s' % move_base.get_state())

        elif state == CompetitionState.ReturnToStart:
            rospy.loginfo('In ReturnToStart state')
            move_base.send_goal(start)
            move_base.wait_for_result(rospy.Duration(action_timeout))
            if move_base.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo('Hooray!')
                rospy.signal_shutdown('Competition completed!')
            else:
                rospy.logerr('action timed out in state: %s' % move_base.get_state())

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('example_solution')
    try:
        example_solution()
    except rospy.ROSInterruptException:
        pass
