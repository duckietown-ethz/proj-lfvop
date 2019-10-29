#!/usr/bin/env python

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import os
import shutil
from skimage import measure
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify, rgb_from_ros
from duckietown_utils.image_jpg_create import d8_compressed_image_from_cv_image
from obst_avoid.detector import Detector
from obst_avoid.visualizer import Visualizer
from numpy.linalg import inv

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import time
import argparse
#this script is here to evaulate the performance of our code on whole large datasets
robot_name='arki' #TO BE SET!!!
#----------------SPECIFY FILES TO READ IN ---------------------
#Automated Location assignment via input params:
parser = argparse.ArgumentParser()
parser.add_argument("input_bagpath", help="specify link to input bagfile", type=str)
parser.add_argument("output_directory", help="specify output directory", type=str)
args=parser.parse_args()

dir_path = args.output_directory
im_path = args.input_bagpath

rospy.init_node('obstacle_detection_node',disable_signals=True)
detector = Detector(robot_name=robot_name)
crop = detector.crop
print crop
intrinsics = load_camera_intrinsics(robot_name)
visualizer = Visualizer(robot_name=robot_name)
H = load_homography(robot_name)

obst_list = PoseArray()
marker_list = MarkerArray()


#CREATE NEW DIRECTORY
if os.path.exists(dir_path):
    shutil.rmtree(dir_path)
os.makedirs(dir_path)
#cv2.imwrite(dir+ '/' + str(i) + '.jpg',im1)

nummer=1

while(True):
    filename = im_path+ '/' + str(nummer) + '.jpg'
    im1 = cv2.imread(filename) #reads BGR
    if (im1 is None):
        #stop it!
        break
    
    
    else: #START MODIFYING THE IMAGE!!!
        #-------------HERE GOES THE REAL CODE-----------------------------------------------------------
        #-----------------------------------------------------------------------------------------------

        obst_list = detector.process_image(rectify(im1[:,:,::-1],intrinsics))
        obst_image = CompressedImage()
        obst_image.format = "jpeg"
        obst_image.data = visualizer.visualize_image(rectify(im1[:,:,::-1],intrinsics),obst_list)
        #here i want to display cropped image
        image=rgb_from_ros(obst_image.data)
        image=image[crop:,:,:]
        #THIS part only to visualize the cropped version -> somehow a little inefficient but keeps
        #the visualizer.py modular!!!
        #plt.imshow(image[detector.crop:,:,:]);plt.show() #the cropped image
        #plt.imshow(image);plt.show()                     #normal sized image
        #SAVE THE IMAGE
        conv = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(dir_path+ '/' + str(nummer) + '.jpg', conv)
        nummer+=1
        

print "FERTIG"
os.system("rosnode kill obstacle_detection_node")