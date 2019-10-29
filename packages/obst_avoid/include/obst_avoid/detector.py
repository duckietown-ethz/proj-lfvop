#!/bin/bash/env python

import argparse
import cv2
import numpy as np
from numpy.linalg import inv
from numpy import linalg as LA
from os.path import basename, expanduser, isfile, join, splitext
import socket
from matplotlib import pyplot as plt
import time
from skimage import measure

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify


class Detector():
    '''class for detecting obstacles'''

    def __init__(self, robot_name=''):
        # Robot name
        self.robot_name = robot_name

        # Array which saves known obstacles to track them better
        self.track_array = np.int_([])

        # Load camera calibration parameters
        self.intrinsics = load_camera_intrinsics(robot_name)
        self.H = load_homography(self.robot_name)
        self.inv_H = inv(self.H)

        # define where to cut the image, color range,...
        self.crop = 150  # default value but is overwritten in init_homo
        # self.lower_yellow = np.array([20,100,150]) #more restrictive -> if you can be sure for "no reddish yellow"
        self.lower_yellow = np.array([15, 100, 200])
        self.upper_yellow = np.array([35, 255, 255])
        self.lower_orange = np.array([0, 100, 100])
        self.upper_orange = np.array([15, 255, 255])
        self.lower_white = np.array([0, 0, 150])
        self.upper_white = np.array([255, 25, 255])

        self.ref_world_point_x = 1.7  # this is the reference world point where we crop the img
        self.major_intertia_thres = 10  # if you want to detect very little ducks might lower it to 10, not smaller,..

        self.img_width = 0  # to be set in init_inv_homography
        self.img_height = 0  # to be set in init_inv_homography
        self.maximum_height = 0  # to be set in ground2bird_view_pixel_init
        self.maximum_left = 0
        self.factor = 1.0  # to be set in ground2bird_view_pixel_init
        self.obst_thres = 20  # to be set in init_inv_homography, this is default was 35
        self.minimum_tracking_distance = 60
        self.min_consec_tracking = 2 #number if times you must have seen critical object before adding as obstacle
        self.new_track_array = np.array([])

        self.M = self.init_inv_homography()
        self.inv_M = inv(self.M)

        center_world_coords = np.float32([[0.0], [0.0], [1.0]])
        center_pixels = self.ground2bird_view_pixel(center_world_coords)
        self.center_x = int(center_pixels[0])
        self.center_y = int(center_pixels[1])

    # initializes where the robot is in the bird view image

    def init_inv_homography(self):
        reference_world_point = np.float32([[self.ref_world_point_x], [0.0], [1.0]])  # adaptive cropping is dangerous
        real_pix_of_ref_point = self.ground2real_pic_pixel(reference_world_point)
        image_height = 480  # default height of image
        self.crop = int(real_pix_of_ref_point[1])
        x0 = 0  # take full width of image
        x1 = 640  # take full width of image
        y0 = 0  # take top of cropped image!
        y1 = image_height - self.crop  # complete bottom
        x_center = 320
        y_center = image_height - self.crop
        pts1 = np.float32([[x0, y0], [x0, y1], [x1, y1], [x1, y0]])
        pts1_h = np.float32(
            [[x0, y0 + self.crop, 1], [x0, y1 + self.crop, 1], [x1, y1 + self.crop, 1], [x1, y0 + self.crop, 1]])
        # add the crop offset to being able to calc real world coordinates correctly!!!
        pts2_h = self.real_pic_pixel2ground(np.transpose(pts1_h))
        bird_view_pixel = self.ground2bird_view_pixel_init(pts2_h)
        # ATTENTION: bird view pixel with (x,y)
        self.img_width = int(np.max(bird_view_pixel[0]))
        self.img_height = int(np.max(bird_view_pixel[1]))
        self.obst_thres = self.obst_thres * (self.img_height / 218.0)  # adaptive threshold
        # print self.obst_thres
        return cv2.getPerspectiveTransform(pts1, np.transpose(bird_view_pixel))

    def process_image(self, image):
        obst_list = PoseArray()
        # FILTER CROPPED IMAGE
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image[self.crop:, :, :], cv2.COLOR_RGB2HSV)
        # Threshold the HSV image to get only yellow colors
        im_test = cv2.warpPerspective(hsv, self.M, (self.img_width, self.img_height))

        mask1 = cv2.inRange(im_test, self.lower_yellow, self.upper_yellow)
        mask2 = cv2.inRange(im_test, self.lower_orange, self.upper_orange)
        mask = np.bitwise_or((mask1 / 2), mask2)
        # mask = mask1/2 #to only test yellow
        # mask = mask2 #to only test orange
        # yellow objects have value 127, orange 255
        if (np.sum(mask != 0) != 0):  # there were segment detected then
            # SEGMENT IMAGE
            segmented_image = self.segment_img(mask)
            props = measure.regionprops(segmented_image, mask)
            no_elements = np.max(segmented_image)
            # apply filter on elements-> only obstacles remain and mark them in original picture
            # in the future: might be separated in 2 steps 1)extract objects 2)visualisation
            obst_list = self.object_filter(props, no_elements, im_test)

        return obst_list

    def segment_img(self, image):
        # returns segmented image on Grayscale where all interconnected pixels have same number
        return measure.label(image)

    def object_filter(self, props, no_elements, image):
        # for future: filter has to become adaptive to depth
        obst_list = PoseArray()
        obst_list.header.frame_id = self.robot_name
        self.new_track_array = np.array([])
        for k in range(1, no_elements + 1):  # iterate through all segmented numbers
            # first only keep large elements then eval their shape
            if (props[k - 1]['area'] > self.obst_thres):  # skip all those who are too small
                # Coordinates in the bord view
                top = props[k - 1]['bbox'][0]
                bottom = props[k - 1]['bbox'][2]
                left = props[k - 1]['bbox'][1]
                right = props[k - 1]['bbox'][3]
                # calc center in top view image
                total_width = right - left
                total_height = bottom - top
                # yellow: 127, orange: 255
                color_info = props[k - 1]['max_intensity']

                # to take small duckies into account:(color_info == 127 and props[k-1]['inertia_tensor_eigvals'][0]>10)
                if ((color_info == 127 and props[k - 1]['inertia_tensor_eigvals'][0] > self.major_intertia_thres) or \
                    (color_info == 255 and \
                     props[k - 1]['inertia_tensor_eigvals'][0] / props[k - 1]['inertia_tensor_eigvals'][1] > 50)):

                    obst_object = Pose()
                    if (color_info == 127):  # means: yellow object:
                        # old fill weight tracker:
                        # new_position = np.array([[left+0.5*total_width],[bottom],[props[k-1]['inertia_tensor_eigvals'][0]],[props[k-1]['inertia_tensor_eigvals'][1]],[-1],[0],[0],[0]])
                        # new leightweight version:
                        new_position = np.array(
                            [[left + 0.5 * total_width], [bottom], [props[k - 1]['inertia_tensor_eigvals'][0]], [0]])
                        # Checks if there is close object from frame before
                        checker = self.obst_tracker(new_position)
                    else:
                        checker = True

                    # Checks if there is close object from frame before
                    if (checker):
                        point_calc = np.zeros((3, 2), dtype=np.float)
                        point_calc = self.bird_view_pixel2ground(
                            np.array([[left + 0.5 * total_width, left], [bottom, bottom]]))
                        obst_object.position.x = point_calc[0, 0]  # obstacle coord x
                        # if (point_calc[0, 0] < 0.35):
                            # print "DANGEROUS OBSTACLE:"
                            # print  point_calc[0:2, 0]
                        obst_object.position.y = point_calc[1, 0]  # obstacle coord y
                        # calculate radius:
                        obst_object.position.z = point_calc[1, 1] - point_calc[1, 0]  # this is the radius!
                        # determine wheter obstacle is out of bounds:
                        if (obst_object.position.y > 0):  # obstacle is left of me
                            line1 = np.array([measure.profile_line(image, (self.center_y, self.center_x),
                                                                   (bottom, right), linewidth=1, order=1,
                                                                   mode='constant')])
                        else:
                            line1 = np.array([measure.profile_line(image, (self.center_y, self.center_x),
                                                                   (bottom, left), linewidth=1, order=1,
                                                                   mode='constant')])
                        # bottom,left
                        line1 = cv2.inRange(line1, self.lower_white, self.upper_white)
                        if (np.sum(line1 == 255) > 3):
                            obst_object.position.z = -1 * obst_object.position.z  # means it is out of bounds!

                        # fill in the pixel boundaries of bird view image!!!
                        obst_object.orientation.x = top
                        obst_object.orientation.y = bottom
                        obst_object.orientation.z = left
                        obst_object.orientation.w = right

                        obst_list.poses.append(obst_object)

                # explanation: those parameters published here are seen from the !center of the axle! in direction
                # of drive with x pointing in direction and y to the left of direction of drive in [m]
                # cv2.rectangle(orig_img,(np.min(C[1]),np.min(C[0])),(np.max(C[1]),np.max(C[0])),(0,255,0),3)

        # eig box np.min breite und hoehe!! if they passed the test!!!!
        # print abc
        self.track_array = self.new_track_array
        return obst_list

    def obst_tracker(self, new_position):
        # input: array of the current track_array which will be used in the next frame
        # input: new_position in 2D image which has to be checked if it could be tracked from the image before
        # outout: minimum distance to an obstacle in the image before, 2*minimum_tracking_distance if there haven't been obstacles in the frame before
        if self.track_array.size != 0:
            distances = -(self.track_array[0:2, :] - new_position[0:2, :])  # only interested in depth change
            distances_norms = LA.norm(distances, axis=0)
            distance_min = np.amin(distances_norms)
            distance_min_index = np.argmin(distances_norms)
            y_distance = distances[1, distance_min_index]

            if (y_distance < -5):  # CHANGE OF DEPTH in PIXELS
                # print "DISTANCE CRITAERIA"
                distance_min = 2 * self.minimum_tracking_distance
        else:
            distance_min = 2 * self.minimum_tracking_distance

        # FINALE AUSWERTUNG:
        if (distance_min < self.minimum_tracking_distance):  # only then modify the entry!
            #major_mom_change = abs(1 - abs(self.track_array[2, distance_min_index] / new_position[2, :]))
            # LEFT OUT DUE TO NO EFFICIENCY BUT ATTNETION: NOW ONE ELEMENT CAN SAY IT IS CLOSE TO MULTIPLE,...
            # new_position[4,:] = distance_min_index #show where we reference to
            # new_position[5,:] = distance_min
            # self.track_array[6,distance_min_index] = self.track_array[6,distance_min_index] +1 #indicate someone refers to this
            # if (self.track_array[6,distance_min_index]>1):
            #		self.track_array[6,distance_min_index] = self.track_array[6,distance_min_index] - 1 #decr again
            #		#print "!!!!!!!!!MULTIPLE REFERENCES!!!!!!!!!!!!!!!!!!" #we must get active
            #		competitor_idx = np.argmax(self.new_track_array[4,:]==distance_min_index)
            #		if (self.new_track_array[4,competitor_idx]<new_position[5,:]): #competitor is closer and can stay
            #			distance_min = 2*self.minimum_tracking_distance
            #			new_position[4,:] = -1 #track lost!
            #		else: #current point is closer
            #			distance_min = 2*self.minimum_tracking_distance
            #			self.new_track_array[4,competitor_idx] = -1 #track lost
            #			self.new_track_array[7,competitor_idx] = 0 #track lost
            #			new_position[7,:]= self.track_array[7,distance_min_index]+1
            #	else:
            new_position[3, :] = self.track_array[3, distance_min_index] + 1

            if ((new_position[2, :] < 100) and new_position[3,
                                                                         :] < self.min_consec_tracking):  # not seen 2 times yet
                # print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                distance_min = 2 * self.minimum_tracking_distance

        if self.new_track_array.size == 0:
            self.new_track_array = new_position
        else:
            self.new_track_array = np.append(self.new_track_array, new_position, axis=1)
        return (distance_min < self.minimum_tracking_distance)

    def real_pic_pixel2ground(self, real_pic_pixel):
        # input: pixel coordinates of real picture in homogeneous coords (3byN)
        # output: real world coordinates (z-component is equal to 1!!!)
        # taking pixel coordinates with (column,row) <-> (x,y) and returning real world coordinates!!! (3byN)
        point_calc = np.zeros(np.shape(real_pic_pixel), dtype=np.float32)
        point_calc = np.dot(self.H, real_pic_pixel)  # calculating realWorldcoords
        point_calc = np.concatenate(([(point_calc[0, :]) / point_calc[2, :], (point_calc[1, :]) / point_calc[2, :]],
                                     np.ones((1, np.shape(real_pic_pixel)[1]))), axis=0)
        return point_calc

    def ground2real_pic_pixel(self, ground):
        # input: real world coordinates (z-component is equal to 1!!!)
        # output: pixel coordinates of real picture in homogeneous coords (3byN)
        # taking real world coordinates and returning (column,row) <-> (x,y) of real_pic_pixels! (2byN)
        point_calc = np.zeros(np.shape(ground), dtype=np.float32)
        point_calc = np.dot(self.inv_H, ground)  # calculating realWorldcoords
        return ([(point_calc[0, :]) / point_calc[2, :], (point_calc[1, :]) / point_calc[2, :]])

    def ground2bird_view_pixel_init(self, ground):
        # input: real world coordinate (3byN)
        # output: bird view pixel (column,row) <-> (x,y)
        # this is initialisation function to set the class parameters!!!!
        # goal: find correct factor:
        min_width = np.min([ground[1, :]])
        max_width = np.max([ground[1, :]])
        total_width = max_width - min_width
        # this width should be 640 pixel, since image is 640pix wide!!!
        self.factor = 640.0 / total_width

        ground = np.float32((ground[0:2, :] / ground[2, :] * self.factor))
        self.maximum_height = np.max([ground[0, :]])
        self.maximum_left = np.max([ground[1, :]])
        return np.flipud((np.float32((np.float32([[self.maximum_height], [self.maximum_left]]) - ground))))

    def ground2bird_view_pixel(self, ground):
        # input: real world coordinate (3byN)
        # output: bird view pixel (column,row) <-> (x,y) (2byN)
        ground = np.float32((ground[0:2, :] / ground[2, :] * self.factor))
        return np.flipud((np.float32((np.float32([[self.maximum_height], [self.maximum_left]]) - ground))))

    def bird_view_pixel2ground(self, bird_view_pixel):
        # input: bird view pixel (column,row) <-> (x,y) (2byN)
        # output: real world coordinate (3byN)
        bird_view_pixel = np.flipud(bird_view_pixel)
        bird_view_pixel = np.float32((np.float32([[self.maximum_height], [self.maximum_left]])) - bird_view_pixel)
        return np.concatenate((bird_view_pixel / self.factor, np.ones((1, np.shape(bird_view_pixel)[1]))), axis=0)

    def bird_view_pixel2real_pic_pixel(self, bird_view_pixel):
        # input: bird view pixel (column,row) <-> (x,y) (2byN)
        # output: real pic pixel (2byN)!!! uncropped!!!
        points = np.transpose(np.float32(bird_view_pixel))
        trans_points = np.float32(cv2.perspectiveTransform(np.array([points]), self.inv_M))
        return np.concatenate(
            (np.reshape(trans_points[:, :, 0], (1, -1)), np.reshape(trans_points[:, :, 1] + self.crop, (1, -1))),
            axis=0)
