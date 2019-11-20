#!/usr/bin/env python
"""
Goal for this part of the project:
Subscribe to the topic created during by the publisher node
and extract the color of the image.
Create two nodes in your .launch file using one python file
The first node:
-Detects the color red
Second node:
-Detects the color yellow
 You should use params within your node tag to let your
 detector know whether it is supposed to detect red/yellow.
Run on laptop.  Pass the required environment variables
to connect your laptop to the rosmaster of your duckiebot
using docker run.
"""
import os
import io
import rospy
import thread
import numpy as np
import cv2

from duckietown import DTROS
from picamera import PiCamera
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String

class SubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SubscriberNode, self).__init__(node_name=node_name)
        #super(PublisherNode, self).__init__(node_name=publisher_node_name)

        # get environment variables, if no color seleted, search for duckies (yellow)
        if 'color_selected' in os.environ:
            self.color_selected = os.environ['color_selected']
        else:
            self.color_selected = 'yellow'

        # construct publisher to receive CompressedImage msgs through the ~image/compressed topic
        self.sub = rospy.Subscriber("~image/compressed", CompressedImage, self.callback)
        self.pub_debug = rospy.Publisher("~image_debug/compressed", CompressedImage)

    def callback(self, msg):
        """ This function received the image data and processes the information"""

        # The compressedImage msg has three contents: header, format and data.
        # msg.data contains the comrpessed image buffer. Collect and convert to HSV format
        cv_img = bridge.imgmsg_to_cv2(msg.data, desired_encoding="passthrough")
        cv_img_debug = cv_img
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)

        if self.color_selected == "yellow":
            # yellow
            lower_range = np.array([81, 100, 100], dtype=np.uint8)
            upper_range = np.array([97, 255, 255], dtype=np.uint8)
        else if self.color_selected == "red":
            # red
            lower_range = np.array([105, 100, 100], dtype=np.uint8)
            upper_range = np.array([119, 255, 255], dtype=np.uint8)

        # # # begin the image processing # # #
        # create a mask for image
        mask = cv2.inRange(hsv, lower_range, upper_range)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=3)

        # find the eternal contours in the mask. Then, draw them in the original image.
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(mask, contours, 0, (0, 255, 0), 2)

        # iterate through all the contours found
        for ii in range(len(contours)):
            rotrect = cv2.minAreaRect(contours[ii])
            box = cv2.boxPoints(rotrect)
            box = np.int0(box)
            cv2.drawContours(cv_img_debug, [box], 0, (0,0,255), 2)



        # Publish the debug image onto the ~/image_debug/compressed topic
        self.pub_debug.publish(cv_img_debug)
        # convert data back into ros data types
        msg_processed = bridge.cv2_to_compressed_imgmsg(cv_image_drawn, dst_format='jpg')

        # save into rosbag file
        bag_processed.write('/patoloco/camera_node/image/compressed', msg_processed, t)
if __name__ == '__main__':
    # create the node
    subscriber_node = SubscriberNode(node_name='subscriber_node')
    thread.start_new_thread(subscriber_node.colorDetector, ())
    # keep spinning
    rospy.spin()
