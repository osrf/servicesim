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

import cv2

from cv_bridge import CvBridge, CvBridgeError

import numpy

import rospy

from sensor_msgs.msg import Image


class ColorSegmentator(object):
    def __init__(self):
        # create an instance of CVBridge, this will allow to convert ROS messages
        #  to OpenCV CVMat and vice versa
        self.bridge = CvBridge()
        # create a publisher to publish the modified image to
        self.segmented_img_pub = rospy.Publisher(
            '/servicebot/camera_front/segmented_image', Image, queue_size=1)
        # create a subscriber to receive the robots front camera image
        self.image_sub = rospy.Subscriber(
            '/servicebot/camera_front/image_raw', Image, self.image_callback)
        # define a range of color to detect guest shirts
        self.boundaries = [
            # keep only dark red pixels
            # ([50, 17, 10], [200, 50, 56]),
            # keep only green pixels
            # ([10, 40, 5], [200, 50, 30]),
            # keep only blue pixels
            ([5, 5, 50], [25, 25, 145])
        ]

    def image_callback(self, msg):
        # Function called each time an image is received from the robot
        try:
            # convert the image to a CVMat
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        for (lower, upper) in self.boundaries:
            # create NumPy arrays from the boundaries
            lower = numpy.array(lower, dtype='uint8')
            upper = numpy.array(upper, dtype='uint8')

            # find the colors within the specified boundaries and create a mask out of it
            mask = cv2.inRange(cv_image, lower, upper)
            # apply the mask to the image
            output = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        try:
            # convert the resulting image to a ros Image message
            masked_image_msg = self.bridge.cv2_to_imgmsg(output, 'rgb8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # publish the resulting image
        self.segmented_img_pub.publish(masked_image_msg)


def blob_detector():
    rospy.init_node('blob_detector')
    node = ColorSegmentator()
    rospy.spin()


if __name__ == '__main__':
    blob_detector()
