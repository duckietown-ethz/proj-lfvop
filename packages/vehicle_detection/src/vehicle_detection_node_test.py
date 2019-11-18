#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage, Image
import cv2
import io
import numpy as np
import rospy
import threading

class VehicleDetectionTestNode(object):
	def __init__(self):
		self.node_name = "Vehcile Detection Test"
		self.bridge = CvBridge()
		self.pub_image = rospy.Publisher("~image", Image, queue_size=1)
		self.sub_image = rospy.Subscriber("~corners", VehicleCorners, 
				self.processCorners, queue_size=1)

		self.original_filename = rospy.get_param('~original_image_file')
		self.original_image = cv2.imread(self.original_filename)
	
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		pub_period = rospy.get_param("~pub_period", 1.0)
		rospy.Timer(rospy.Duration.from_sec(pub_period), self.pubOrig)

	def pubOrig(self, args=None):			
		np_arr = np.fromstring(np.array(cv2.imencode('.png',
 				self.original_image)[1]).tostring(), np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		img_msg.header.stamp = rospy.Time.now()
		self.pub_image.publish(img_msg)

		rospy.loginfo("Publishing original image")
		

	def processCorners(self, corners_msg):
		for i in np.arange(len(corners_msg.corners)):
			rospy.loginfo('Corners received : (x = %.2f, y = %.2f)' %
					(corners_msg.corners[i].x, corners_msg.corners[i].y))


if __name__ == '__main__': 
	rospy.init_node('vehicle_detection_test_node', anonymous=False)
	virtual_mirror_test_node = VehicleDetectionTestNode()
	rospy.spin()


