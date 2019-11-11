#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners, Pixel, SegmentList, Vector2D
from geometry_msgs.msg import Point32, Point
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
from duckietown_utils import (logger, get_duckiefleet_root)#, ground2pixel, pixel2ground)
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)


class DuckieDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.robot_name = rospy.get_namespace().strip("/")
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

        self.sub_lane_segments = rospy.Subscriber("~seglist_filtered", SegmentList, self.laneHandling, queue_size=1)

        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)

        self.pub_boundingbox_image = rospy.Publisher("~duckiedetected_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.detected_duckie_distance = rospy.Publisher("~detected_duckie_distance",
                                                Float32, queue_size=1)

        self.publish_boundingbox=1
        #self.rectified_input=1 #change to rectify, antiinstagram!: subscribe to anti_instagram_node/corrected_image/compressed
        #subscribe to /lane_filter_node/seglist_filtered and take pixels_normalized with color yellow and set to zero in mask to ignore yellow lane
        self.resolution=np.array([0,0])#np.empty(shape=[0,2])
        self.resolution[0]=rospy.get_param('/%s/camera_node/res_w' %self.robot_name)
        self.resolution[1]=rospy.get_param('/%s/camera_node/res_h' %self.robot_name)
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)
        #self.pcm_ = PinholeCameraModel()
        #self.p0 = Pixel()
        #self.p1 = Pixel()
        self.lane_seg=np.array([])
        self.lane_detected = 0
        self.lane_width_thresh =10 # erase also all yellow pixel close to lane, not used yet

        self.yellow_low = np.array([20,100,100]) #load from yaml file in the future
        self.yellow_high = np.array([30,255,255])

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

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


        mask = cv2.inRange(hsv_img, self.yellow_low, self.yellow_high)
        if self.lane_detected:#set all yellow lane segments to zero in mask
            self.lane_detected = 0
            self.lane_seg=self.lane_seg.astype(int)
            #print self.lane_seg.shape
            for i in range(self.lane_seg.shape[0]):
                mask[self.lane_seg[i][0]:self.lane_seg[i][2],self.lane_seg[i][1]:self.lane_seg[i][3]]=0

                cv2.rectangle(cv_image,(self.lane_seg[i][0],self.lane_seg[i][1]),(self.lane_seg[i][2],self.lane_seg[i][3]),(0,0,255),2) #for debuging mark areas that have been erased from mask in red

        #mask1 = mask[mask.shape[0]/4:mask.shape[0]/4*3]: crop image!
        img_cont, contours, hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#threshold
        #cv2.drawContours(cv_image,contours,-1,(0,0,255),3) #for debugging
        duckiefound=0 #init
        distance=0#init
        duckie_loc_pix = Pixel()

        for item in range(len(contours)):
            cnt = contours[item]
            if len(cnt)>20:
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
                duckie_loc_pix.u=int(x+w/2)
                duckie_loc_pix.v=y+h
                duckie_loc_world = self.pixel2ground(duckie_loc_pix)
                distance = duckie_loc_world.y #currently just takes last contor, change this to biggest contor?
                duckiefound = 1


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

    def laneHandling(self,lane_segments_msg):

        seg_points1 = Point()
        seg_points2 = Point()
        #seg_points1 = Vector2D()
        #seg_points2 = Vector2D()
        seg_points = [seg_points1,seg_points2] #not needed?
        self.lane_seg = np.empty(shape=[0,4])

        for s in range (len(lane_segments_msg.segments)):
            if lane_segments_msg.segments[s].color==1: #0: white 1: yellow, 2:red

                self.lane_detected = 1
                seg_points = lane_segments_msg.segments[s].points
                self.p0=np.array([self.ground2pixel(seg_points[0]).u,self.ground2pixel(seg_points[0]).v])
                self.p1=np.array([self.ground2pixel(seg_points[1]).u,self.ground2pixel(seg_points[1]).v])
                #seg_points = lane_segments_msg.segments[s].pixels_normalized
                #self.p0=np.multiply(np.array([seg_points[0].x,seg_points[0].y]),self.resolution) #muliply with resolution to unnormalize coordinates
                self.p0=self.p0.astype(int)
                self.p1=self.p1.astype(int)
                #self.p1=np.multiply(np.array([seg_points[1].x,seg_points[1].y]),self.resolution)


                #print np.array([self.p0[0],self.p0[1],self.p1[0],self.p1[1]]) # stack coordinates of the two corners in a 1x4 row

                self.lane_seg = np.vstack((self.lane_seg,np.array([self.p0[0],self.p0[1],self.p1[0],self.p1[1]])))



    def pixel2ground(self,pixel):
        uv_raw = np.array([pixel.u, pixel.v])
        #if not self.rectified_input:
        #    uv_raw = self.pcm_.rectifyPoint(uv_raw)
        #uv_raw = [uv_raw, 1]
        uv_raw = np.append(uv_raw, np.array([1]))
        ground_point = np.dot(self.H, uv_raw)
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x/z
        point.y = y/z
        point.z = 0.0
        return point

    def ground2pixel(self, point):
        ground_point = np.array([point.x, point.y, 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]


        pixel = Pixel()
        # if not self.rectified_input:
        #     distorted_pixel = self.pcm_.project3dToPixel(image_point)
        #     pixel.u = distorted_pixel[0]
        #     pixel.v = distorted_pixel[1]
        #else:
        pixel.u = int(image_point[0])
        pixel.v = int(image_point[1])
        return pixel

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + self.robot_name + ".yaml")
        if not os.path.isfile(filename):
            logger.warn("no extrinsic calibration parameters for {}, trying default".format(self.robot_name))
            filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/default.yaml")
            if not os.path.isfile(filename):
                logger.error("can't find default either, something's wrong")
            else:
                data = yaml_load_file(filename)
        else:
            rospy.loginfo("Using extrinsic calibration of " + self.robot_name)
            data = yaml_load_file(filename)
        logger.info("Loaded homography for {}".format(os.path.basename(filename)))
        return np.array(data['homography']).reshape((3,3))


if __name__ == '__main__':
    rospy.init_node('duckie_detection', anonymous=False)
    duckie_detection_node = DuckieDetectionNode()
    rospy.spin()
