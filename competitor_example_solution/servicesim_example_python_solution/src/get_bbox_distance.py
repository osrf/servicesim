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
import imutils
import rospy
from servicesim_example_python_solution.msg import Contour
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
class ColorSegmentator(object):
    def __init__(self):
        # create an instance of CVBridge, this will allow to convert ROS messages
        #  to OpenCV CVMat and vice versa
        self.bridge = CvBridge()
        # create a publisher to publish the modified image to
        self.segmented_img_pub = rospy.Publisher(
            '/servicebot/camera_front/segmented_image', Image, queue_size=1)
        # create a publisher to publish the height, width and area of the contourArea
        self.pub = rospy.Publisher(
        '/servicebot/bbox_distance', Contour, queue_size=1)
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
            # ([5, 5, 50], [25, 25, 145])
            #head color
            # ([155,105, 75], [165, 115, 85])
            #pant color
            ([95,95, 95], [110, 110, 110])


        ]

    def get_bbox_coordinates(self, boundaries, cv_image):
        self.boundaries = boundaries

        for (lower, upper) in self.boundaries:
            # create NumPy arrays from the boundaries
            lower = numpy.array(lower, dtype='uint8')
            upper = numpy.array(upper, dtype='uint8')

            # find the colors within the specified boundaries and create a mask out of it
            mask = cv2.inRange(cv_image, lower, upper)
            # apply the mask to the image
            output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        output_gray = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
        kernel = numpy.ones((15,15), numpy.uint8)
        thresh = cv2.threshold(output_gray, 5, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations = 2)
        thresh = cv2.dilate(thresh, kernel, iterations = 2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts  = cnts[0] if imutils.is_cv2() else cnts[1]
        if len(cnts)!=0:
            c = max(cnts, key = cv2.contourArea)
            area = cv2.contourArea(c)

            extLeft = tuple(c[c[:, :, 0].argmin()][0])
            extRight = tuple(c[c[:, :, 0].argmax()][0])
            extTop = tuple(c[c[:, :, 1].argmin()][0])
            extBot = tuple(c[c[:, :, 1].argmax()][0])

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

        boundaries_shirt = [
            # keep only blue pixels
            ([5, 5, 50], [25, 25, 145])   ]
        # define a range of color to detect guest shirts
        boundaries_pants = [
            #pants color
            ([95,95, 95], [110, 110, 110]) ]

        coordinates_pants = self.get_bbox_coordinates(boundaries_pants,cv_image)
        coordinates_shirts = self.get_bbox_coordinates(boundaries_shirt,cv_image)

        if (coordinates_pants and coordinates_shirts):
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
            # print(x_min, x_max, y_min, y_max)
            # print(width, height)

            ### calculating the distance
            # height of the person
            H = 1.5
            # focal length of the camera
            F = 550
            # Pixel value of the height
            P = height
            # distance of the person from the camera
            D = (F*H)/P
            contour_msg = Contour()
            contour_msg.distance = D
            contour_msg.width = width
            contour_msg.height = height
            contour_msg.x_min = x_min
            contour_msg.y_min = y_min
            contour_msg.x_max = x_max
            contour_msg.y_max = y_max

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
        self.segmented_img_pub.publish(image_with_msg)


def blob_detector():
    rospy.init_node('blob_detector')
    node = ColorSegmentator()
    rospy.spin()


if __name__ == '__main__':
    blob_detector()
