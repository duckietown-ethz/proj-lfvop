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


        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.laneHandlingPose, queue_size=1) #remove for debugging

        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)

        self.pub_boundingbox_image = rospy.Publisher("~duckiedetected_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.pub_duckie_locations = rospy.Publisher("~detected_duckie_locations",
                                                Float64MultiArray, queue_size=1)

        self.publish_boundingbox=1
        self.lanestrip_width = 0.05#0.024 #half of lanestrip width)(including buffer), to the left and to the right
        self.lane_width = 0.1145#0.1025 #(half of lane + half of strip)
        self.lane_factor = 1#3
        self.crop_factor = float(1.0/3)

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
        self.phi_new=0
        self.memory_size=[3,4] #3 timesteps, 4 duckies
        self.detection_memory=np.zeros((3,8)) #memory of 3, for max 4 duckies

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
        duckie_locations_msg_out = Float64MultiArray()
        # lane_pose_msg_sim = LanePose()#for debugging, remove!!
        # lane_pose_msg_sim.d=0
        # lane_pose_msg_sim.phi=0
        # self.laneHandlingPose(lane_pose_msg_sim)#for debugging, remove!!

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()
        cv_image = self.rectify_image(cv_image) #does this really help?
        cv_image_crop = cv_image[float(cv_image.shape[0])*self.crop_factor:cv_image.shape[0]*3/4][:] #crop upper 1/3 and lower 1/4 of image
        hsv_img = cv2.cvtColor(cv_image_crop, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.yellow_low, self.yellow_high)
        mask = cv2.erode(mask,None, iterations=1)
        mask = cv2.dilate(mask,None, iterations=3)

        if self.lane_detected:#set all yellow lane segments to zero in mask
            self.lane_detected = 0

            #print "before clipping"
            #print self.laneL_pixel0,self.laneL_pixel1
    #        ret,self.laneL_pixel0_clipped,self.laneL_pixel1_clipped=cv2.clipLine((0,0,mask.shape[0],mask.shape[1]),(self.laneL_pixel0.u,self.laneL_pixel0.v),(self.laneL_pixel1.u,self.laneL_pixel1.v))
    #        ret,self.laneR_pixel0_clipped,self.laneR_pixel1_clipped=cv2.clipLine((0,0,mask.shape[0],mask.shape[1]),(self.laneR_pixel0.u,self.laneR_pixel0.v),(self.laneR_pixel1.u,self.laneR_pixel1.v))
    #        points = np.array([self.laneL_pixel0_clipped,self.laneL_pixel1_clipped,self.laneR_pixel1_clipped,self.laneR_pixel0_clipped])
            points = np.array([[[self.laneL_pixel0.u,self.laneL_pixel0.v]],[[self.laneL_pixel1.u,self.laneL_pixel1.v]],[[self.laneR_pixel1.u,self.laneR_pixel1.v]],[[self.laneR_pixel0.u,self.laneR_pixel0.v]]])
            lane_mask = np.ones_like(mask)
            print points
            cv2.fillPoly(lane_mask,np.int32([points]),0)
            mask_out = cv2.bitwise_and(mask,lane_mask)
            mask = mask_out
            cv2.drawContours(cv_image_crop,np.int32([points]),-1,(0,0,255),3)


        img_cont, contours, hierarchy=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#threshold
        duckiefound=0 #init
        distance=0#init
        duckie_loc_pix = Pixel()
        duckie_locations = []
        i=0

        np.delete(self.detection_memory,0,0) #delete first row
        np.append(self.detection_memory,np.zeros((1,8)),0)#add empty row at the end
        #self.detection_memory[0][:]=self.detection_memory[1][:]
        #self.detection_memory[1][:]=np.zeros(1,2) #for 4 duckies max

        for item in range(len(contours)):
            cnt = contours[item]
            if len(cnt)>20:
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_image_crop,(x,y),(x+w,y+h),(0,255,0),2)

                duckie_loc_pix.u=int(x+w/2) #transpose??
                duckie_loc_pix.v=y+h+float(self.resolution[1])*self.crop_factor #to compensate for crop
                #print x,y
                duckie_loc_world = self.pixel2ground(duckie_loc_pix)
                duckie_locations.append([duckie_loc_world.x,duckie_loc_world.y])
                distance = duckie_loc_world.x #currently just takes last contor, change this to biggest contor?
                print("duckie nr ",i,": distance: ", distance)
                duckiefound = 1
                self.detection_memory[-1][i*2:i*2+2]=[duckie_loc_world.x,duckie_loc_world.y]
                #np.append(self.detection_memory, [duckie_loc_world.x,duckie_loc_world.y], 0)
                i=i+1



        if duckiefound:#change. test for every other car for all times in array
            dist_prev_duckie = np.array([
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[0][0])**2+(self.detection_memory[2][1]-self.detection_memory[0][1])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[0][2])**2+(self.detection_memory[2][1]-self.detection_memory[0][3])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[0][4])**2+(self.detection_memory[2][1]-self.detection_memory[0][5])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[0][6])**2+(self.detection_memory[2][1]-self.detection_memory[0][7])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[0][0])**2+(self.detection_memory[2][3]-self.detection_memory[0][1])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[0][2])**2+(self.detection_memory[2][3]-self.detection_memory[0][3])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[0][4])**2+(self.detection_memory[2][3]-self.detection_memory[0][5])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[0][6])**2+(self.detection_memory[2][3]-self.detection_memory[0][7])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[0][0])**2+(self.detection_memory[2][5]-self.detection_memory[0][1])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[0][2])**2+(self.detection_memory[2][5]-self.detection_memory[0][3])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[0][4])**2+(self.detection_memory[2][5]-self.detection_memory[0][5])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[0][6])**2+(self.detection_memory[2][5]-self.detection_memory[0][7])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[0][0])**2+(self.detection_memory[2][7]-self.detection_memory[0][1])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[0][2])**2+(self.detection_memory[2][7]-self.detection_memory[0][3])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[0][4])**2+(self.detection_memory[2][7]-self.detection_memory[0][5])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[0][6])**2+(self.detection_memory[2][7]-self.detection_memory[0][7])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[1][0])**2+(self.detection_memory[2][1]-self.detection_memory[1][1])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[1][2])**2+(self.detection_memory[2][1]-self.detection_memory[1][3])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[1][4])**2+(self.detection_memory[2][1]-self.detection_memory[1][5])**2),
            math.sqrt((self.detection_memory[2][0]-self.detection_memory[1][6])**2+(self.detection_memory[2][1]-self.detection_memory[1][7])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[1][0])**2+(self.detection_memory[2][3]-self.detection_memory[1][1])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[1][2])**2+(self.detection_memory[2][3]-self.detection_memory[1][3])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[1][4])**2+(self.detection_memory[2][3]-self.detection_memory[1][5])**2),
            math.sqrt((self.detection_memory[2][2]-self.detection_memory[1][6])**2+(self.detection_memory[2][3]-self.detection_memory[1][7])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[1][0])**2+(self.detection_memory[2][5]-self.detection_memory[1][1])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[1][2])**2+(self.detection_memory[2][5]-self.detection_memory[1][3])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[1][4])**2+(self.detection_memory[2][5]-self.detection_memory[1][5])**2),
            math.sqrt((self.detection_memory[2][4]-self.detection_memory[1][6])**2+(self.detection_memory[2][5]-self.detection_memory[1][7])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[1][0])**2+(self.detection_memory[2][7]-self.detection_memory[1][1])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[1][2])**2+(self.detection_memory[2][7]-self.detection_memory[1][3])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[1][4])**2+(self.detection_memory[2][7]-self.detection_memory[1][5])**2),
            math.sqrt((self.detection_memory[2][6]-self.detection_memory[1][6])**2+(self.detection_memory[2][7]-self.detection_memory[1][7])**2)])
            print dist_prev_duckie
            if not dist_prev_duckie.any()<0.2: #only set to true if prev close detected
                duckiefound = 0

        duckie_detected_msg_out.data = duckiefound
        self.pub_detection.publish(duckie_detected_msg_out)

        if duckiefound:
            duckie_locations_msg_out.data = np.asarray(duckie_locations)
            self.pub_duckie_locations.publish(duckie_locations_msg_out)


        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_boundingbox:
            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image_crop, "bgr8")
            self.pub_boundingbox_image.publish(image_msg_out)

    def laneHandlingPose(self,lane_pose_msg):
        self.lane_detected = 1
        self.d = lane_pose_msg.d
        self.phi_prev = self.phi_new
        self.phi_new = lane_pose_msg.phi
        #self.phi=self.phi_new
        self.phi=0.8*self.phi_new+0.2*self.phi_prev #moving average, does it makes sense??
        laneL_point0 = Point() #left side of lane
        laneL_point1 = Point()
        laneL_point0.x = np.sin(-self.phi)*(self.lane_width-self.d+self.lanestrip_width) + np.cos(-self.phi)*self.lane_factor*0.1
        laneL_point0.y = np.cos(-self.phi)*(self.lane_width-self.d+self.lanestrip_width) + np.sin(-self.phi)*self.lane_factor*0.1
        laneL_point1.x = laneL_point0.x + np.cos(-self.phi)*self.lane_factor*1
        laneL_point1.y = laneL_point0.y + np.sin(-self.phi)*self.lane_factor*1
        #print "ground L0,L1"
        #print laneL_point0,laneL_point1

        self.laneL_pixel0=self.ground2pixel(laneL_point0)
        self.laneL_pixel1=self.ground2pixel(laneL_point1)
        self.laneL_pixel0.v -= float(self.resolution[1])*self.crop_factor #to compensate for crop
        self.laneL_pixel1.v -= float(self.resolution[1])*self.crop_factor#to compensate for crop
        #print "psxel L"
        #print self.laneL_pixel0, self.laneL_pixel1

        laneR_point0 = Point()#right side of lane
        laneR_point1 = Point()
        laneR_point0.x = laneL_point0.x - np.sin(-self.phi)*2*self.lanestrip_width #np.sin(-self.phi)*(self.lane_width-self.d-self.lanestrip_width) - np.cos(-self.phi)*self.lane_factor #now taking lane in range (0.5m,1m-->shouldnt be over image border)
        laneR_point0.y = laneL_point0.y - np.cos(-self.phi)*2*self.lanestrip_width#np.cos(-self.phi)*(self.lane_width-self.d-self.lanestrip_width) - np.sin(-self.phi)*self.lane_factor
        laneR_point1.x = laneL_point1.x - np.sin(-self.phi)*2*self.lanestrip_width#laneR_point0.x + np.cos(-self.phi)*self.lane_factor*10
        laneR_point1.y = laneL_point1.y - np.cos(-self.phi)*2*self.lanestrip_width#laneR_point0.y + np.sin(-self.phi)*self.lane_factor*10
        #print "ground R"
        #print laneR_point0, laneR_point1

        self.laneR_pixel0 = self.ground2pixel(laneR_point0)
        self.laneR_pixel1 = self.ground2pixel(laneR_point1)
        #print "pixel R before crop corr"
        #print self.laneR_pixel0, self.laneR_pixel1
        #print float(self.resolution[1])*self.crop_factor
        self.laneR_pixel0.v -= float(self.resolution[1])*self.crop_factor #to compensate for crop
        self.laneR_pixel1.v -= float(self.resolution[1])*self.crop_factor#to compensate for crop



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
