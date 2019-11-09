#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import time
import yaml


class DuckieDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.active = True
        self.config = self.setupParam("~config", "baseline")
        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        rospack = rospkg.RosPack()


        self.lock = mutex()
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600,
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)
        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)

        self.pub_boundingbox_image = rospy.Publisher("~duckiedetected_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.detected_duckie_distance = rospy.Publisher("~detected_duckie_distance",
                                                Float32, queue_size=1)

        self.publish_boundingbox=1

        rospy.loginfo("[%s] Initialization completed" % (self.node_name))

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processImage(self, image_msg):

        if not self.active:
            return

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now


        duckie_detected_msg_out = BoolStamped()
        distance_msg_out = Float32()

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #insert yellow detector here
        duckiefound=1
        distance=2
        duckie_detected_msg_out.data = duckiefound
        distance_msg_out=distance
        self.pub_detection.publish(duckie_detected_msg_out)
        if duckiefound:
            self.detected_duckie_distance.publish(distance_msg_out)

        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_boundingbox:
            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub_boundingbox_image.publish(image_msg_out)


if __name__ == '__main__':
    rospy.init_node('duckie_detection', anonymous=False)
    duckie_detection_node = DuckieDetectionNode()
    rospy.spin()
