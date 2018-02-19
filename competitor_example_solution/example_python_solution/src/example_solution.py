#! /usr/bin/env python
# from __future__ import print_function

from enum import Enum

import actionlib

from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, Pose, Quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy

from servicesim_competition.srv import DropOffGuest, DropOffGuestRequest, NewTask
from servicesim_competition.srv import PickUpGuest, PickUpGuestRequest


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
    return [resp.pick_up_location, resp.drop_off_location, resp.guest_name]


def request_follow(robot_name, guest_name):
    rospy.loginfo('waiting for pickup_guest service')
    rospy.wait_for_service('/servicesim/pickup_guest')
    rospy.loginfo('requesting guest to follow the robot')
    pickup_guest = rospy.ServiceProxy('/servicesim/pickup_guest', PickUpGuest)
    req = PickUpGuestRequest()
    req.robot_name = robot_name
    req.guest_name = guest_name
    resp = pickup_guest(req)
    rospy.loginfo(resp)
    return [resp.success]


def request_unfollow(guest_name):
    rospy.loginfo('waiting for dropoff_guest service')
    rospy.wait_for_service('/servicesim/dropoff_guest')
    rospy.loginfo('requesting guest to stop following the robot')
    dropoff_guest = rospy.ServiceProxy('/servicesim/dropoff_guest', DropOffGuest)
    req = DropOffGuestRequest()
    req.guest_name = guest_name
    resp = dropoff_guest(req)
    rospy.loginfo(resp)
    return [resp.success]


def pose_from_room_name(room_name):
    # TODO(mikaelarguedas) Replace this with a service call to get room pose
    room_mapping = {
        'start': Pose(Point(3.7, 1.23, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)),
        'reception': Pose(Point(-6.2, 1.9, 0.000), Quaternion(0.000, 0.000, 1.000, 0.000)),
        'office_1': Pose(Point(-3.764, 6.893, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
    }
    if room_name in room_mapping.keys():
        return room_mapping[room_name]
    else:
        rospy.logerr('position of room "%s" is unknown' % room_name)
        return None


def example_solution():
    robot_name = 'servicebot'
    move_base = actionlib.SimpleActionClient('/servicebot/move_base', MoveBaseAction)

    action_timeout = 60

    move_base.wait_for_server(rospy.Duration(action_timeout))

    # Set up dropoff location waypoints
    dropoff_door = MoveBaseGoal()
    dropoff_in = MoveBaseGoal()
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
            [pick_up_location, drop_off_location, guest_name] = request_new_task()
            rospy.loginfo('requesting task twice')
            # test that service call fails when task requested while previous task not ended
            # try:
            #     request_new_task()
            # except rospy.service.ServiceException as e:
            #     rospy.logerr('service call failed with message: "%s"' % e.message)
            pickupgoal = MoveBaseGoal()
            pickupgoal.target_pose.pose = pose_from_room_name(pick_up_location)
            pickupgoal.target_pose.header.frame_id = 'map'
            state = CompetitionState.Pickup

        elif state == CompetitionState.Pickup:
            rospy.loginfo('In Pickup state')
            pickupgoal.target_pose.header.stamp = rospy.Time.now()
            move_base.send_goal(pickupgoal)
            move_base.wait_for_result(rospy.Duration(action_timeout))
            rospy.loginfo(
                'got result: Action state is: %s\n success is %d' %
                (move_base.get_state(), GoalStatus.SUCCEEDED))
            if move_base.get_state() == GoalStatus.SUCCEEDED:
                if request_follow(robot_name, guest_name):
                    state = CompetitionState.DropOff
                    dropoff_goals[0] = MoveBaseGoal()
                    dropoff_goals[0].target_pose.pose = pose_from_room_name(drop_off_location)
                    dropoff_goals[0].target_pose.header.frame_id = 'map'

        elif state == CompetitionState.DropOff:
            rospy.loginfo('In DropOff state')
            rospy.loginfo('Sending goal #%d' % goal_nb)
            dropoff_goals[goal_nb].target_pose.header.stamp = rospy.Time.now()
            move_base.send_goal(dropoff_goals[goal_nb])
            move_base.wait_for_result(rospy.Duration(action_timeout))
            if move_base.get_state() == GoalStatus.SUCCEEDED:
                goal_nb += 1
                if goal_nb == len(dropoff_goals) - 1:
                    request_unfollow(guest_name)
                elif goal_nb == len(dropoff_goals):
                    startgoal = MoveBaseGoal()
                    startgoal.target_pose.pose = pose_from_room_name('start')
                    startgoal.target_pose.header.frame_id = 'map'
                    state = CompetitionState.ReturnToStart
            else:
                rospy.logerr('action timed out in state: %s' % move_base.get_state())

        elif state == CompetitionState.ReturnToStart:
            rospy.loginfo('In ReturnToStart state')
            startgoal.target_pose.header.stamp = rospy.Time.now()
            move_base.send_goal(startgoal)
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
