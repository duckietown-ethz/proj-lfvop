#!/bin/bash/env python

import argparse
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from obst_avoid.detector import Detector

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify



class Visualizer():
    '''class for visualizing detected obstacles'''
    def __init__(self, robot_name=''):
        # Robot name
    	self.robot_name = robot_name
        self.detector = Detector(robot_name=self.robot_name)
        #create detector object to get access to all the trafos


    def visualize_marker(self, obst_list):
    	marker_list=MarkerArray()



   	size = obst_list.poses.__len__()
   	for i in range(0,size):

                marker = Marker()
                marker.type = marker.CYLINDER
                marker.header.frame_id=self.robot_name
                marker.frame_locked=False
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                if (obst_list.poses[i].position.z<0):
                        marker.color.g = 1.0
                else:
                        marker.color.r = 1.0

                marker.pose.orientation.w = 1.0
                marker.lifetime = rospy.Time(7.0)
                #each marker if not replaced earlier by same id will dissapear after max 1 second
   		marker.id = i
	   	marker.scale.x = abs(obst_list.poses[i].position.z) #since is negative if not relevant
                #marker.scale.x = 0.2
                marker.scale.y = abs(obst_list.poses[i].position.z)
                #marker.scale.y = 0.2
	   	marker.pose.position.x = obst_list.poses[i].position.x
	   	marker.pose.position.y = obst_list.poses[i].position.y
	   	marker.pose.position.z = 0
	   	marker_list.markers.append(marker)

    	#print marker_list.markers.__len__()
    	return marker_list

    def visualize_image(self, image,obst_list):

    	size = obst_list.poses.__len__()
       	for i in range(0,size):
                    top = obst_list.poses[i].orientation.x
                    bottom = obst_list.poses[i].orientation.y
                    left = obst_list.poses[i].orientation.z
                    right = obst_list.poses[i].orientation.w
                    points = np.float32([[left,left,right,right],[top,bottom,bottom,top]])
                    pts = self.detector.bird_view_pixel2real_pic_pixel(points)
                    if (obst_list.poses[i].position.z<0):
                            cv2.polylines(image,np.int32([np.transpose(pts)]),True,(0,255,0),3)
                    else:
                            cv2.polylines(image,np.int32([np.transpose(pts)]),True,(255,0,0),3)

    	return d8_compressed_image_from_cv_image(image[:,:,::-1])

    def visualize_bird_perspective(self, image,obst_list):
        #hsv = cv2.cvtColor(image[self.crop:, :, :], cv2.COLOR_RGB2HSV)
    
        bird_img = cv2.warpPerspective(image, self.detector.M, (self.detector.img_width, self.detector.img_height))
    	size = obst_list.poses.__len__()
       	for i in range(0,size):
                    top = obst_list.poses[i].orientation.x
                    bottom = obst_list.poses[i].orientation.y
                    left = obst_list.poses[i].orientation.z
                    right = obst_list.poses[i].orientation.w
                    points = np.float32([[left,left,right,right],[top,bottom,bottom,top]])
                    if (obst_list.poses[i].position.z<0):
                            cv2.polylines(bird_img,np.int32([np.transpose(points)]),True,(0,255,0),3)
                    else:
                            cv2.polylines(bird_img,np.int32([np.transpose(points)]),True,(255,0,0),3)

    	return d8_compressed_image_from_cv_image(bird_img[:,:,::-1])
