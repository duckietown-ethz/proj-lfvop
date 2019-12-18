#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float64MultiArray
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
        self.veh_name = rospy.get_namespace().strip("/")
        self.bridge = CvBridge()
        self.active = True
        self.config = self.setupParam("~config", "baseline")
        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        self.frontorback="back"
        rospack = rospkg.RosPack()

        self.publish_circles = True

        self.lock = mutex()
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600,
                                          queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
                                           self.cbSwitch, queue_size=1)

        self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        self.pub_detected_duckiebot_head = rospy.Publisher("~detected_duckiebot_head",
                                                Float64MultiArray, queue_size=1)
        self.pub_detected_duckiebot_tail = rospy.Publisher("~detected_duckiebot_tail",
                                                Float64MultiArray, queue_size=1)

        self.pub_detected_duckiebot_front_state = rospy.Publisher("~detected_duckiebot_head_state",
                                             BoolStamped, queue_size=1)

        self.pub_detected_duckiebot_tail_state = rospy.Publisher("~detected_duckiebot_tail_state",
                                             BoolStamped, queue_size=1)

        self.intrinsics = load_camera_intrinsics(self.veh_name)
        self.fx=self.intrinsics['K'][0][0]
        self.fy=self.intrinsics['K'][1][1]
        self.radialparam=self.intrinsics['D']
        self.Midtold=0
        self.depthold=0
        self.Midtoldb=0
        self.deptholdb=0
        self.time = rospy.get_rostime().to_sec()
        self.timeb = rospy.get_rostime().to_sec()

        self.threshold = 235
        try:
            self.threshold = int(os.environ["THRESHOLD"])
        except:
            pass

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def undistort(self, keypoints):
        #undistort specific points radially
        #similar to section initUndistortRectifyMap from https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
        #here the function is made to use on single points and not undistort the entire image as it is not necessary and much faster
        for key in keypoints: #for all keypoints of interest
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

        try:
            cv_image_color = self.bridge.compressed_imgmsg_to_cv2(
                image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        start = rospy.Time.now()


        #crop top and buttom image as the leds can not be here
        cv_image_color = cv_image_color[int(cv_image_color.shape[0]*0.3):cv_image_color.shape[0]/4*3]

        #to greyscale
        cv_image1 = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)
        #cv_image1 = cv_image1[cv_image1.shape[0]/4:cv_image1.shape[0]/4*3]

        #binary image based on high threshold to get bright parts (bright LEDs etc.)
        ret,cv_image = cv2.threshold(cv_image1,220,255,cv2.THRESH_BINARY)

        # Set up the blob detector
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
        params.minCircularity = 0.7 #leds are close to circular 1
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(cv_image)
        #make copy for radial undistortion
        self.cv_image=cv_image
        keyin=self.features_deepcopy(keypoints)
        #undistort radially the points
        keypoints_un=self.undistort(keyin)


        #values for led positions, d indicate distorted positions(used to plot on distorted image), no d is for the undistorted points(used in calculation). b indicate red led and no b white led
        x1=x2=y1=y2=x1d=x2d=y1d=y2d=x1b=x2b=y1b=y2b=x1db=x2db=y1db=y2db=0

        redfound=0
        whitefound=0
        #Trye to find the two keypoints that are the 2 LEDs
        for i,key1 in enumerate(keypoints_un): #for all keypoints
            for j,key2 in enumerate(keypoints_un): #compare with all other keypoints
                if key1!=key2:
                    if abs((key1.size-key2.size)/key1.size)<0.4: #rougly same size keys
                        if abs((key1.pt[1]-key2.pt[1]))<key1.size/1: #rougly same y coordinate
                            #get color fo the keypoints center
                            pixel1= cv_image_color[int(keypoints[i].pt[1]), int(keypoints[i].pt[0])]
                            pixel2= cv_image_color[int(keypoints[j].pt[1]), int(keypoints[j].pt[0])]
                            blue1=pixel1[0]
                            blue2=pixel2[0]
                            #Both red and white LEDs completely saturate the red and green channel.
                            bluethreshold=235 #For the blue channel the threshold between white and red was found with experiments

                            #check if the blue value of the led light is matching the red back or the white front
                            if (blue1<bluethreshold and blue2<bluethreshold): #red
                                #save undistorted positions
                                x1b=key1.pt[0]
                                x2b=key2.pt[0]
                                y1b=key1.pt[1]
                                y2b=key2.pt[1]
                                #save distorted positions
                                x1db=keypoints[i].pt[0]
                                x2db=keypoints[j].pt[0]
                                y1db=keypoints[i].pt[1]
                                y2db=keypoints[j].pt[1]
                                redfound=1
                            if  (blue1>=bluethreshold and blue2>=bluethreshold): #white
                                #save undistorted positions
                                x1=key1.pt[0]
                                x2=key2.pt[0]
                                y1=key1.pt[1]
                                y2=key2.pt[1]
                                #save distorted positions
                                x1d=keypoints[i].pt[0]
                                x2d=keypoints[j].pt[0]
                                y1d=keypoints[i].pt[1]
                                y2d=keypoints[j].pt[1]
                                whitefound=1



        detected_duckiebot_tail_state = BoolStamped()
        detected_duckiebot_front_state = BoolStamped()
        detected_duckiebot_tail_state.data = 0
        detected_duckiebot_front_state.data = 0

        if whitefound==1:
            #calculate depth of LED position based on them beeing 0.12 m apart, focal length and pixel distance
            depth=0.12*self.fy/abs(x2-x1)
            imheight, imwidth = cv_image.shape[:2]
            #offset from center axis
            midt=(x1+x2)/2-imwidth/2
            #convert to real distance again
            Midt=midt/self.fx*depth
            #self.detected_log.append((Midt,depth))
            t=rospy.get_rostime().to_sec()
            #calculate the velocity of the points, very sensitive to noise between frames and thus not used further currently
            vMidt=(Midt-self.Midtold)/(t-self.time)
            vdepth=(depth-self.depthold)/(t-self.time)

            self.time = t
            self.Midtold=Midt
            self.depthold=depth
            data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
            data_to_send.data = [depth,-Midt,vdepth,-vMidt] #pos x,y vel x,y, check minus!!
            self.pub_detected_duckiebot_head.publish(data_to_send)
            detected_duckiebot_front_state.data=1

        elif redfound==1:

            #same as for the white case
            depth=0.12*self.fy/abs(x2b-x1b)
            imheight, imwidth = cv_image.shape[:2]
            midt=(x1b+x2b)/2-imwidth/2
            Midt=midt/self.fx*depth
            t=rospy.get_rostime().to_sec()
            vMidt=(Midt-self.Midtoldb)/(t-self.timeb)
            vdepth=(depth-self.deptholdb)/(t-self.timeb)

            self.timeb = t
            self.Midtoldb=Midt
            self.deptholdb=depth
            data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
            data_to_send.data = [depth,-Midt,vdepth,-vMidt] #pos x,y vel x,y, check minus!!
            self.pub_detected_duckiebot_tail.publish(data_to_send)
            detected_duckiebot_tail_state.data=1




        self.pub_detected_duckiebot_tail_state.publish(detected_duckiebot_tail_state)
        self.pub_detected_duckiebot_front_state.publish(detected_duckiebot_front_state)

        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)

        if self.publish_circles:
            #publish debuggin images
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv_image1 = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            #front
            cv_image1=cv2.circle(cv_image1,(int(x1),int(y1)), 5, (0,255,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x2),int(y2)), 5, (0,255,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x1d),int(y1d)), 5, (255,0,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x2d),int(y2d)), 5, (255,0,0), 2)
            #back
            cv_image1=cv2.circle(cv_image1,(int(x1b),int(y1b)), 5, (0,100,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x2b),int(y2b)), 5, (0,100,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x1db),int(y1db)), 5, (100,0,0), 2)
            cv_image1=cv2.circle(cv_image1,(int(x2db),int(y2db)), 5, (100,0,0), 2)

            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image1, "bgr8")
            self.pub_circlepattern_image.publish(image_msg_out)





if __name__ == '__main__':
    rospy.init_node('led_detection', anonymous=False)
    led_detection_node = LEDDetectionNode()
    rospy.spin()
