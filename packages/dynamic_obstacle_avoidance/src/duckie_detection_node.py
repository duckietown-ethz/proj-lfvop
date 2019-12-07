#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners, Pixel, SegmentList, Vector2D, LanePose
from geometry_msgs.msg import Point32, Point
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float64MultiArray
import cv2
import numpy as np
import math
import os
import rospkg
import rospy
import threading
import time
import yaml
from duckietown_utils import (logger, get_duckiefleet_root)#, pixel2ground)
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from dynamic_obstacle_avoidance.msg import dynamic_obstacle


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

        self.publish_boundingbox=1
        self.lanestrip_width = 0.05#0.024 #half of lanestrip width)(including buffer), to the left and to the right
        self.lane_width = 0.1145#0.1025 #(half of lane + half of strip)
        self.crop_factor = float(1.0/3)
        self.d = 0.0
        self.phi = 0.0

        #self.rectified_input=1 #change to rectify, antiinstagram!: subscribe to anti_instagram_node/corrected_image/compressed
        self.resolution=np.array([0,0])#np.empty(shape=[0,2])
        self.resolution[0]=rospy.get_param('/%s/camera_node/res_w' %self.robot_name)
        self.resolution[1]=rospy.get_param('/%s/camera_node/res_h' %self.robot_name)
        self.load_intrinsics()
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)

        self.yellow_low = np.array([25,180,180])
        self.yellow_high = np.array([35,255,255])

        rospy.loginfo("[%s] Initialization completed" % (self.node_name))

        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600,
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)

        # self.pub_detection = rospy.Publisher("~detected_duckie_state",
        #                                      BoolStamped, queue_size=1)

        self.pub_boundingbox_image = rospy.Publisher("~duckiedetected_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.pub_duckie = rospy.Publisher("~detected_duckie",
                                                dynamic_obstacle, queue_size=1)

        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.cbLanePose, queue_size=1)



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
        duckie_locations_msg_out = Float64MultiArray()

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()
        #cv_image = self.rectify_image(cv_image) #does this really help?
        cv_image_crop = cv_image[cv_image.shape[0]*self.crop_factor:cv_image.shape[0]*3/4][:] #crop upper 1/3 and lower 1/4 of image
        hsv_img = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.yellow_low, self.yellow_high)

        duckiefound = False
        duckie_loc_pix = Pixel()
        duckie_msg = dynamic_obstacle()
        duckie_pos_arr = []
        duckie_state_arr = []
        keypoints=[]
        i=0

        params = cv2.SimpleBlobDetector_Params()

        params.filterByColor = True
        params.blobColor = 255

        params.filterByArea = True
        params.minArea = 50 #50, good
        params.filterByInertia = True
        params.minInertiaRatio = 0.5 #0.5, good!!
        params.filterByConvexity = False
        params.maxConvexity = 0.99 #>0.99, not so good
        params.filterByCircularity = False
        params.minCircularity = 0.5 #<0.5, not so good
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.

        keypoints = detector.detect(mask)

        t = cv2.drawKeypoints(cv_image_crop, keypoints, cv_image_crop, color=(0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if keypoints:
            duckiefound = True
            for key in keypoints:
                duckie_loc_pix.u=key.pt[0]
                duckie_loc_pix.v=key.pt[1]+key.size/2+float(self.resolution[1])*self.crop_factor #to compensate for crop
                duckie_loc_world = self.pixel2ground(duckie_loc_pix)

                #self.duckie_rel[0] = duckie_loc_world.x/np.cos(self.phi)
                duckie_side = np.cos(self.phi)*(duckie_loc_world.y+self.d)+np.sin(self.phi)*duckie_loc_world.x
                # print ("duckie position: ", duckie_loc_world)
                # print ("duckie side: ", duckie_side)
                duckie_pos_arr.append([duckie_loc_world.x,duckie_loc_world.y])
                if abs(duckie_side)<self.lane_width: #right lane
                    duckie_state_arr.append([1])
                elif duckie_side>self.lane_width and duckie_side<self.lane_width*3: #left lane
                    duckie_state_arr.append([2])
                else:
                    duckie_state_arr.append([0]) #write zero if no duckie detected

                i=i+1
        else:
            duckie_state_arr.append([0]) #write zero if no duckie detected

        #print duckie_locations
        # duckie_detected_msg_out.data = duckiefound
        # self.pub_detection.publish(duckie_detected_msg_out)
        # if duckiefound:
            # duckie_locations_msg_out.data = np.array(duckie_locations).ravel()
            # self.pub_duckie_locations.publish(duckie_locations_msg_out)

        duckie_msg.pos=np.array(duckie_pos_arr).ravel()
        duckie_msg.state=np.array(duckie_state_arr).ravel()
        duckie_msg.vel=np.array([])
        duckie_msg.header.stamp = rospy.Time.now()
        self.pub_duckie.publish(duckie_msg)


        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_boundingbox:
            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image_crop,"bgr8") #cv_image_crop, "bgr8"
            self.pub_boundingbox_image.publish(image_msg_out)

    def cbLanePose(self,lane_pose_msg):
        self.d = lane_pose_msg.d
        self.phi = lane_pose_msg.phi

    def pixel2ground(self,pixel):
        uv_raw = np.array([pixel.u, pixel.v])
        #if not self.rectified_input:
        #    uv_raw = self.pcm_.rectifyPoint(uv_raw)

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

    def rectify_image(self, image): #does this really help?
        height, width, _ = image.shape
        rectified_image = np.zeros(np.shape(image))
        mapx = np.ndarray(shape=(height, width, 1), dtype='float32')
        mapy = np.ndarray(shape=(height, width, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.intrinsics['K'],
                                                 self.intrinsics['D'],
                                                 self.intrinsics['R'],
                                                 self.intrinsics['P'],
                                                 (width, height), cv2.CV_32FC1, mapx, mapy)
        out_img = cv2.remap(image, mapx, mapy, cv2.INTER_CUBIC, rectified_image)
        return out_img

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

    def load_intrinsics(self):

        filename = (get_duckiefleet_root() + "/calibrations/camera_intrinsic/" + self.robot_name + ".yaml")
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(filename):
            self.log("Intrinsic calibration file %s does not exist! Using the default file." % filename, type='warn')
            filename = (get_duckiefleet_root() + "/calibrations/camera_intrinsic/default.yaml")

        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()

        self.intrinsics = {}
        self.intrinsics['K'] = np.array(data['camera_matrix']['data']).reshape(3, 3)
        self.intrinsics['D'] = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
        self.intrinsics['R'] = np.array(data['rectification_matrix']['data']).reshape(3, 3)
        self.intrinsics['P'] = np.array(data['projection_matrix']['data']).reshape((3, 4))
        self.intrinsics['distortion_model'] = data['distortion_model']


if __name__ == '__main__':
    rospy.init_node('duckie_detection', anonymous=False)
    duckie_detection_node = DuckieDetectionNode()
    rospy.spin()
