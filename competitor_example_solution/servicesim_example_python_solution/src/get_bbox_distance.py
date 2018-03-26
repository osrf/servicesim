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

from servicesim_example_python_solution.msg import Contour

from sensor_msgs.msg import Image


class BboxDetector(object):
    def __init__(self):
        # create an instance of CVBridge, this will allow to convert ROS messages
        #  to OpenCV CVMat and vice versa
        self.bridge = CvBridge()
        # create a publisher to publish the modified image to
        self.bbox_img_pub = rospy.Publisher(
            '/servicebot/camera_front/guest_bbox_detection', Image, queue_size=1)
        # create a publisher to publish the height, width and area of the contourArea
        self.pub = rospy.Publisher(
            '/servicebot/bbox_distance', Contour, queue_size=1)
        # create a subscriber to receive the robots front camera image
        self.image_sub = rospy.Subscriber(
            '/servicebot/camera_front/image_raw', Image, self.image_callback)

    # Returns the bounding box coordinates of the detected contour area,
    # given the threshold of the color of contour to be detected
    def get_bbox_coordinates(self, boundaries, cv_image):
        for (lower, upper) in boundaries:
            # create NumPy arrays from the boundaries
            lower = numpy.array(lower, dtype='uint8')
            upper = numpy.array(upper, dtype='uint8')

            # find the colors within the specified boundaries and create a mask out of it
            mask = cv2.inRange(cv_image, lower, upper)
            # apply the mask to the image
            output = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # creating a gray scale image of the masked output
        output_gray = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
        kernel = numpy.ones((15,15), numpy.uint8)
        # Creating a binary image with some threshold and then dilating and eroding \
        # the resulted image for a cleaner contour detection
        thresh = cv2.threshold(output_gray, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations = 2)
        thresh = cv2.dilate(thresh, kernel, iterations = 2)
        # Detecting the contours
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts  = cnts[1]
        if len(cnts)!=0:
            # Detecting the contour with maximum area
            c = max(cnts, key = cv2.contourArea)
            area = cv2.contourArea(c)
            # Getting the extremities of the max contour
            extLeft = tuple(c[c[:, :, 0].argmin()][0])
            extRight = tuple(c[c[:, :, 0].argmax()][0])
            extTop = tuple(c[c[:, :, 1].argmin()][0])
            extBot = tuple(c[c[:, :, 1].argmax()][0])
            # Translating the extreme points of the max contour to bounding box coordinates \
            # which encloses the max contour
            x_min = min(extLeft[0], extRight[0], extTop[0], extBot[0])
            x_max = max(extLeft[0], extRight[0], extTop[0], extBot[0])
            y_min = min(extLeft[1], extRight[1], extTop[1], extBot[1])
            y_max = max(extLeft[1], extRight[1], extTop[1], extBot[1])
            return [x_min, x_max, y_min, y_max]


    def image_callback(self, msg):
        # Function called each time an image is received from the robot
        try:
            # convert the image to a CVMat
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # define a range of color to detect guest's shirt, here it is Blue
        boundaries_shirt = [
            # keep only blue pixels
            ([5, 5, 50], [25, 25, 145])]
        # define a range of color to detect guest's pants, here it is Red
        boundaries_pants = [
            # keep only red pixels
            ([90, 0, 0], [115, 10, 10])]

        # Getting the bounding box coordinates of the shirt and pants of the guest
        coordinates_pants = self.get_bbox_coordinates(boundaries_pants,cv_image)
        coordinates_shirts = self.get_bbox_coordinates(boundaries_shirt,cv_image)
        if (coordinates_pants and coordinates_shirts):
            # checking if the detected contour for the shirt \
            #  is above pants along the same vertical height
            if (coordinates_pants[3]>coordinates_shirts[3] and \
                coordinates_shirts[2]<coordinates_pants[2] and \
                coordinates_shirts[0]<coordinates_pants[1] and \
                coordinates_pants[0]<coordinates_shirts[1]):

                # Calculating the bounding box coordinates enclosing the whole of guest's body.
                x_min = min(coordinates_shirts[0], coordinates_pants[0])
                x_max = max(coordinates_shirts[1], coordinates_pants[1])
                y_min = min(coordinates_shirts[2], coordinates_pants[2])
                y_max = max(coordinates_shirts[3], coordinates_pants[3])
                extLeft = (x_min, y_min)
                extRight = (x_min, y_max)
                extTop = (x_max, y_min)
                extBot = (x_max, y_max)

                cv2.circle(cv_image, extLeft, 8, (0, 0, 255), -1)
                cv2.circle(cv_image, extRight, 8, (0, 255, 0), -1)
                cv2.circle(cv_image, extTop, 8, (255, 0, 0), -1)
                cv2.circle(cv_image, extBot, 8, (255, 255, 0), -1)

                # getting the height and width of the bounding box enclosing the contour
                width = x_max - x_min
                height = y_max - y_min

                # getting the center of the bounding box enclosing the contour
                x_center = int((x_max + x_min)/2)
                y_center = int((y_max + y_min)/2)

                ### Calculating the distance of the person from the robot's camera
                # height of the person/guest
                H = 1.5
                # focal length of the camera
                F = 550
                # Pixel value of the height
                P = height
                # distance of the person from the camera
                D = (F*H)/P
                contour_msg = Contour()

                # populating the contour msg with guest's distance and bbox coordinates
                contour_msg.distance = D
                contour_msg.bbox.center.x = x_center
                contour_msg.bbox.center.y = y_center
                contour_msg.bbox.size_x = width
                contour_msg.bbox.size_y = height

                # publish the contour details
                if (contour_msg.distance):
                    self.pub.publish(contour_msg)

        try:
            # convert the resulting image to a ros Image message
            image_with_msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # publish the resulting image
        self.bbox_img_pub.publish(image_with_msg)


def get_bbox_distance():
    rospy.init_node('bbox_detector')
    node = BboxDetector()
    rospy.spin()


if __name__ == '__main__':
    get_bbox_distance()
