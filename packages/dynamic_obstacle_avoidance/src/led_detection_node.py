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
import copy
from duckietown_utils import load_camera_intrinsics

class LEDDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.active = True
        self.config = self.setupParam("~config", "baseline")
        self.cali_file_name = self.setupParam("~cali_file_name", "default")
        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        self.robot_name=os.environ.get("ROBOT_NAME")
        self.frontorback=os.environ.get("FRONT_OR_BACK")
        rospack = rospkg.RosPack()
        self.cali_file = "/code/catkin_ws/src/dt-core/packages/dynamic_obstacle_avoidance/config/led_detection_node/" +  \
            self.cali_file_name + ".yaml"
        # self.cali_file = rospack.get_path('duckietown') + \
        #     "/config/" + self.config + \
        #     "/vehicle_detection/vehicle_detection_node/" +  \
        #     self.cali_file_name + ".yaml"
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\n"
                          % (self.node_name, self.cali_file))
        self.loadConfig(self.cali_file)

        self.lock = mutex()
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600,
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)
        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)
        self.pub_corners = rospy.Publisher("~corners",
                                           VehicleCorners, queue_size=1)
        self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.detected_vehicle_distance = rospy.Publisher("~detected_vehicle_distance",
                                                Float32, queue_size=1)
        self.intrinsics = load_camera_intrinsics(self.robot_name)
        self.fx=self.intrinsics['K'][0][0]
        self.fy=self.intrinsics['K'][1][1]
        self.radialparam=self.intrinsics['D']


    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.circlepattern_dims = data['circlepattern_dims']['dist']
        self.blobdetector_min_area = data['blobdetector_min_area']
        self.blobdetector_min_dist_between_blobs = data['blobdetector_min_dist_between_blobs']
        self.publish_circles = data['publish_circles']
        rospy.loginfo('[%s] circlepattern_dim : %s' % (self.node_name,
                                                       self.circlepattern_dims,))
        rospy.loginfo('[%s] blobdetector_min_area: %.2f' % (self.node_name,
                                                            self.blobdetector_min_area))
        rospy.loginfo('[%s] blobdetector_min_dist_between_blobs: %.2f' % (self.node_name,
                                                                          self.blobdetector_min_dist_between_blobs))
        rospy.loginfo('[%s] publish_circles: %r' % (self.node_name,
                                                    self.publish_circles))

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def undistort(self, keypoints):
        #undistort radially
        #section initUndistortRectifyMap from https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
        for key in keypoints:
            k1=self.radialparam[0][0]
            k2=self.radialparam[0][1]
            p1=self.radialparam[0][2]
            p2=self.radialparam[0][3]
            imheight, imwidth = self.cv_image.shape[:2]
            cx=imwidth/2
            cy=imheight/2
            xr=(key.pt[0]-cx)/self.fx
            yr=(key.pt[1]-cy)/self.fy
            rd=np.sqrt(xr*xr+yr*yr)
            xun=cx+self.fx*(xr*(1+k1*rd*rd+k2*rd*rd*rd*rd)+2*p1*xr*yr+p2*(rd*rd+2*xr*xr))
            yun=cy+self.fy*(yr*(1+k1*rd*rd+k2*rd*rd*rd*rd)+p1*(rd*rd+2*yr*yr)+2*p2*xr*yr)
            key.pt=(xun,yun)
            key.size=key.size

        return keypoints


    def features_deepcopy(self,f):
        #because deepcopy does not work with keypoint
        return [cv2.KeyPoint(x = k.pt[0], y = k.pt[1], 
                _size = k.size, _angle = k.angle, 
                _response = k.response, _octave = k.octave, 
                _class_id = k.class_id) for k in f]



    def processImage(self, image_msg):

        if not self.active:
            return

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now


        vehicle_detected_msg_out = BoolStamped()
        vehicle_corners_msg_out = VehicleCorners()
        try:
            cv_image_color = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()



        cv_image_color = cv_image_color[cv_image_color.shape[0]/4:cv_image_color.shape[0]/4*3]

        cv_image1 = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)
        #cv_image1 = cv_image1[cv_image1.shape[0]/4:cv_image1.shape[0]/4*3]
        

        ret,cv_image = cv2.threshold(cv_image1,220,255,cv2.THRESH_BINARY)

        # Set up the detector with default parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10;    # the graylevel of images
        params.maxThreshold = 200;

        params.filterByColor = True
        params.blobColor = 255

        # Filter by Area
        params.filterByArea = False
        params.minArea = 10000
        params.filterByInertia = False
        params.filterByConvexity = False
        params.filterByCircularity = True
        params.minCircularity = 0.7
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(cv_image)

        #undistort radially the points
        self.cv_image=cv_image
        keyin=self.features_deepcopy(keypoints)
        
        keypoints_un=self.undistort(keyin)
        
       

        #print(keypoints[0].pt)
        x1=0
        x2=0
        y1=0
        y2=0
        x1d=0
        x2d=0
        y1d=0
        y2d=0
        carfound=0
        for i,key1 in enumerate(keypoints_un):
            for j,key2 in enumerate(keypoints_un):
                if key1!=key2:
                    if abs((key1.size-key2.size)/key1.size)<0.4: #same size keys maybe change parameter
                        if abs((key1.pt[1]-key2.pt[1]))<key1.size/1: #same y coordinate maybe change parameter
                            dist=abs((key1.pt[0]-key2.pt[0]))
                            print(dist/key1.size)
                            #print(key1.size*4+key1.size)
                            dist_est=0.12*key1.size/0.01
                            if dist>dist_est*0.6 and dist <dist_est*1.1: #roughly right distance compared to light size
                                pixel1= cv_image_color[int(keypoints[i].pt[1]), int(keypoints[i].pt[0])]
                                pixel2= cv_image_color[int(keypoints[j].pt[1]), int(keypoints[j].pt[0])]
                                blue1=pixel1[0]
                                blue2=pixel2[0]
                                print("blue value: "+str(blue1))
                                #print(dist/key1.size)
                                bluethreshold=235
                                #check if the blue value of the led light is matching the red back or the white front
                                if (self.frontorback=="back" and blue1<bluethreshold and blue2<bluethreshold) or(self.frontorback=="front" and blue1>=bluethreshold and blue2>=bluethreshold):
                                    x1=key1.pt[0]
                                    x2=key2.pt[0]
                                    y1=key1.pt[1]
                                    y2=key2.pt[1]
                                    x1d=keypoints[i].pt[0]
                                    x2d=keypoints[j].pt[0]
                                    y1d=keypoints[i].pt[1]
                                    y2d=keypoints[j].pt[1]

                                carfound=1

        if carfound==1:
            #fn = dtu.get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + self.veh_name + ".yaml"
            #f=318 #figure out how to get focal length of robot calibration

            depth=0.12*self.fy/abs(x2-x1)
            print("Depth: " +str(depth))
            imheight, imwidth = cv_image.shape[:2]
            midt=(x1+x2)/2-imwidth/2
            Midt=midt/self.fx*depth
            #print(Midt)
            #self.detected_log.append((Midt,depth))
        else:
            print("no car found")

        # print(corners)

        vehicle_detected_msg_out.data = carfound
        self.pub_detection.publish(vehicle_detected_msg_out)
        if carfound:
            self.detected_vehicle_distance.publish(depth)
        #     # print(corners)
        #     points_list = []
        #     for point in corners:
        #         corner = Point32()
        #         # print(point[0])
        #         corner.x = point[0, 0]
        #         # print(point[0,1])
        #         corner.y = point[0, 1]
        #         corner.z = 0
        #         points_list.append(corner)
        #     vehicle_corners_msg_out.header.stamp = rospy.Time.now()
        #     vehicle_corners_msg_out.corners = points_list
        #     vehicle_corners_msg_out.detection.data = detection
        #     vehicle_corners_msg_out.H = self.circlepattern_dims[1]
        #     vehicle_corners_msg_out.W = self.circlepattern_dims[0]
        #     self.pub_corners.publish(vehicle_corners_msg_out)
        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_circles:
            # cv2.drawChessboardCorners(image_cv,
            #                             self.circlepattern_dims, corners, detection)
                    # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv_image1 = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            cv_image1=cv2.circle(cv_image1,(int(x1),int(y1)), 5, (0,255,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x2),int(y2)), 5, (0,255,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x1d),int(y1d)), 5, (255,0,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x2d),int(y2d)), 5, (255,0,0), 2)
            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image1, "bgr8")
            self.pub_circlepattern_image.publish(image_msg_out)



if __name__ == '__main__':
    rospy.init_node('led_detection', anonymous=False)
    led_detection_node = LEDDetectionNode()
    rospy.spin()
