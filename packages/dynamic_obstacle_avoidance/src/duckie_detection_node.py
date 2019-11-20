#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners, Pixel, SegmentList, Vector2D, LanePose
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


        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)

        self.pub_boundingbox_image = rospy.Publisher("~duckiedetected_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.detected_duckie_point = rospy.Publisher("~detected_duckie_point",
                                                Point, queue_size=1)

        self.publish_boundingbox=1
        self.lanestrip_width = 0.05#0.024 #half of lanestrip width)(including buffer), to the left and to the right
        self.lane_width = 0.1145#0.1025 #(half of lane + half of strip)
        self.lane_factor = 20

        #self.rectified_input=1 #change to rectify, antiinstagram!: subscribe to anti_instagram_node/corrected_image/compressed
        #subscribe to /lane_filter_node/seglist_filtered and take pixels_normalized with color yellow and set to zero in mask to ignore yellow lane
        self.resolution=np.array([0,0])#np.empty(shape=[0,2])
        self.resolution[0]=rospy.get_param('/%s/camera_node/res_w' %self.robot_name)
        self.resolution[1]=rospy.get_param('/%s/camera_node/res_h' %self.robot_name)
        self.load_intrinsics()
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)
        #self.pcm_ = PinholeCameraModel()
        #self.p0 = Pixel()
        #self.p1 = Pixel()
        self.lane_seg=np.array([])
        self.lane_detected = 0

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
        duckie_point_msg_out = Point()

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()
        #cv_image = self.rectify_image(cv_image) #does this really help?
        cv_image_crop = cv_image[cv_image.shape[0]/3:cv_image.shape[0]*3/4][:] #crop upper and lower 1/4 of image
        hsv_img = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.yellow_low, self.yellow_high)
        mask = cv2.erode(mask,None, iterations=1)
        mask = cv2.dilate(mask,None, iterations=3)

        img_cont, contours, hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#threshold
        ###lane detection
        segments_info = []

        for item in range(len(contours)):
            cnt = contours[item]
            if len(cnt)>20:
                #((x1,y1),(w1,h1),angle) = cv2.minAreaRect(cnt)
                rect = cv2.minAreaRect(cnt)
                ((x1,y1),(w1,h1),angle) = rect
                if w1<h1:#wrong axis->add 90deg
                    angle-=90
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(cv_image_crop,[box],0,(0,255,0),2)
                segments_info.append([box[0],box[1],box[2],box[3],angle])

                self.lane_detected=True

        if self.lane_detected:#set all yellow lane segments to zero in mask
            #print segments_info
            segments_info = np.asarray(segments_info)
            lane_angle = np.median(segments_info[:,4])
            #sort nach winkel, mittelwert
            # segments_sorted = segments_info[segments_info[:,4].argsort()]
            # (p1,p2,p3,p4,angle) = segments_sorted[segments_sorted.shape[0]//2,:]
            #sort by y coord of zerost point
            segments_sorted = segments_info[segments_info[:,0][0].argsort()]
            (p1,p2,p3,p4,angle) = segments_sorted[-1,:] #take last ->largest element
            print p1,p2,p3,p4,angle,lane_angle
            lane_angle=angle #better?
            cv2.circle(cv_image_crop, (p1[0],p1[1]), 5, (255, 0, 0),1)
            cv2.circle(cv_image_crop, (p2[0],p2[1]), 5, (255, 0, 0),1)
            laneR_pixel0 = (int(p1[0]),int(p1[1]))
            laneL_pixel0 = (int(p2[0]),int(p2[1]))
            #laneR_pixel0 = (int(p1[0]-np.cos(lane_angle)*self.lane_factor),int(p1[1]-np.sin(lane_angle)*self.lane_factor))
            #laneL_pixel0 = (int(p2[0]-np.cos(lane_angle)*self.lane_factor),int(p2[1]-np.sin(lane_angle)*self.lane_factor))
            laneR_pixel1 = (int(laneR_pixel0[0]+np.sin(lane_angle)*2*self.lane_factor), int(laneR_pixel0[1]+np.cos(lane_angle)*2*self.lane_factor))
            laneL_pixel1 = (int(laneL_pixel0[0]+np.sin(lane_angle)*2*self.lane_factor), int(laneL_pixel0[1]+np.cos(lane_angle)*2*self.lane_factor))

            print laneL_pixel0,laneR_pixel0


            ret,self.laneL_pixel0_clipped,self.laneL_pixel1_clipped=cv2.clipLine((0,0,mask.shape[0],mask.shape[1]),laneL_pixel0,laneL_pixel1)
            ret,self.laneR_pixel0_clipped,self.laneR_pixel1_clipped=cv2.clipLine((0,0,mask.shape[0],mask.shape[1]),laneR_pixel0,laneR_pixel1)
            points = np.array([self.laneL_pixel0_clipped,self.laneL_pixel1_clipped,self.laneR_pixel1_clipped,self.laneR_pixel0_clipped])
            #points = np.array([laneL_pixel0,laneL_pixel1,laneR_pixel1,laneR_pixel0])

            lane_mask = np.ones_like(mask)

            #print mask.shape
            print points
            cv2.fillPoly(lane_mask,np.int32([points]),0)
            mask_out = cv2.bitwise_and(mask,lane_mask)
            mask = mask_out
            cv2.drawContours(cv_image_crop,np.int32([points]),-1,(0,0,255),3)

        #cv2.drawContours(cv_image,contours,-1,(0,0,255),3) #for debugging
        duckiefound=0 #init
        distance=0#init
        duckie_loc_pix = Pixel()

        img_cont, contours, hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#threshold

        for item in range(len(contours)):
            cnt = contours[item]
            if len(cnt)>20:
                x,y,w,h = cv2.boundingRect(cnt)
                #cv2.rectangle(cv_image_crop,(x,y),(x+w,y+h),(0,255,0),2)

                duckie_loc_pix.u=int(x+w/2)
                duckie_loc_pix.v=y+h+mask.shape[0]/3 #to compensate for crop
                duckie_loc_world = self.pixel2ground(duckie_loc_pix)
                distance = duckie_loc_world.y #currently just takes last contor, change this to biggest contor?
                #print("duckie distance: ", distance)
                duckiefound = 1


        duckie_detected_msg_out.data = duckiefound

        self.pub_detection.publish(duckie_detected_msg_out)
        if duckiefound:
            duckie_point_msg_out=duckie_loc_world
            self.detected_duckie_point.publish(duckie_point_msg_out)

        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_boundingbox:
            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image_crop, "bgr8")
            self.pub_boundingbox_image.publish(image_msg_out)


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
