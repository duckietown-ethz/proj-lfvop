#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import time
import threading

### note you need to change the name of the robot to yours here
from obst_avoid.detector import Detector
from obst_avoid.visualizer import Visualizer
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics, d8_compressed_image_from_cv_image
from duckietown_msgs.msg import BoolStamped, FSMState

class ObstDetectNode(object):
    """
    Obstacle Detection Node
    """
    def __init__(self):
        self.node_name = "Obstacle Detecion Node"
        robot_name = rospy.get_param("~veh", "")
        self.show_marker = (rospy.get_param("~show_marker", ""))
        self.show_image = (rospy.get_param("~show_image", ""))
        self.show_bird_perspective = (rospy.get_param("~show_bird_perspective",""))
        self.use_ai = (rospy.get_param("~use_ai", ""))

        self.active = False #initialize our node as active!! < -- How about no
        self.r = rospy.Rate(3) # Rate in Hz
        self.thread_lock = threading.Lock()

        self.detector = Detector(robot_name=robot_name)

        # Load camera calibration parameters
        self.intrinsics = load_camera_intrinsics(robot_name)

        # Create a Publisher
        self.pub_topic_arr = "~posearray"
        self.publisher_arr = rospy.Publisher(self.pub_topic_arr, PoseArray, queue_size=1)

        if (self.show_marker or self.show_image):
                self.visualizer = Visualizer(robot_name=robot_name)

        if (self.show_marker):
                self.pub_topic_marker = '/{}/obst_detect/visualize_obstacles'.format(robot_name)
                self.publisher_marker = rospy.Publisher(self.pub_topic_marker, MarkerArray, queue_size=1)
                print "show_marker is active: marker will be published as /veh/obst_detect/visualize_obstacles"

        if (self.show_image):
                self.pub_topic_img = '/{}/obst_detect/image_cropped/compressed'.format(robot_name)
                self.publisher_img = rospy.Publisher(self.pub_topic_img, CompressedImage, queue_size=1)
                print "show_image is active: image will be published as /veh/obst_detect/image_cropped/compressed"

        if (self.show_bird_perspective):
                self.pub_topic_img_bird_perspective = '/{}/obst_detect/image_bird_perspective/compressed'.format(robot_name)
                self.publisher_img_bird_perspective = rospy.Publisher(self.pub_topic_img_bird_perspective, CompressedImage, queue_size=1)
                print "show_image is active: image will be published as /veh/obst_detect/image_bird_perspective/compressed"

        # Create a Subscriber
        if (self.use_ai):
            #self.sub_topic = '/{}/image_transformer_node/corrected_image'.format(robot_name)
            #self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.callback_img,queue_size=1, buff_size=2**24)
            self.sub_topic = rospy.Subscriber('/{}/image_transformer_node/corrected_image/compressed'.format(robot_name), CompressedImage,self.callback_img , queue_size=1,buff_size=2**24)
            #buff size to approximately close to 2^24 such that always most recent pic is taken
            #essentail
        else:
            self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
            self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.callback_img,queue_size=1, buff_size=2**24)


        # FSM
        self.sub_switch = rospy.Subscriber('/{}/obstacle_avoidance_node/switch'.format(robot_name), BoolStamped, self.cbSwitch,  queue_size=1)     # for this topic, no remapping is required, since it is directly defined in the namespace lane_controller_node by the fsm_node (via it's default.yaml file)
        self.sub_fsm_mode = rospy.Subscriber('/{}/fsm_node/mode'.format(robot_name), FSMState, self.cbMode, queue_size=1)


    # FSM
    def cbSwitch(self,fsm_switch_msg):
        self.active = fsm_switch_msg.data # True or False
        rospy.loginfo("IS OBSTACLE_AVOIDANCE_NODE ACTIVE: " + str(self.active))

    # FSM
    def cbMode(self,fsm_state_msg):
        self.fsm_state = fsm_state_msg.state # String of current FSM state
        print "fsm_state changed in obstacle_avoidance_node to: " , self.fsm_state



    def callback_img(self, image):
        if not self.active: #if node is turned off -> nothing happens!!!
                return
        thread = threading.Thread(target=self.callback,args=(image,))
        thread.setDaemon(True)
        thread.start()



    def callback(self, image):
        if not self.active:
            return

        if not self.thread_lock.acquire(False):
            return

        #start = time.time()
        obst_list = PoseArray()
        marker_list = MarkerArray()

        # pass RECTIFIED IMAGE TO DETECTOR MODULE
        #1. EXTRACT OBSTACLES and return the pose array
        obst_list = self.detector.process_image(rectify(rgb_from_ros(image),self.intrinsics))

        obst_list.header.stamp = image.header.stamp #for synchronization
        #interessant um zu schauen ob stau oder nicht!!!!
        #print image.header.stamp.to_sec()
        self.publisher_arr.publish(obst_list)
        #EXPLANATION: (x,y) is world coordinates of obstacle, z is radius of obstacle
        #QUATERNION HAS NO MEANING!!!!

        #3. VISUALIZE POSE ARRAY IN TF
        if (self.show_marker):
                marker_list = self.visualizer.visualize_marker(obst_list)
                self.publisher_marker.publish(marker_list)


        #4. EVENTUALLY DISPLAY !!!!CROPPED!!!!!! IMAGE
        if (self.show_image):
                obst_image = CompressedImage()
                obst_image.header.stamp = image.header.stamp
                obst_image.format = "jpeg"
                obst_image.data = self.visualizer.visualize_image(rectify(rgb_from_ros(image),self.intrinsics),obst_list)
                #here i want to display cropped image
                rgb_image=rgb_from_ros(obst_image.data)
                obst_image.data = d8_compressed_image_from_cv_image(rgb_image[self.detector.crop:,:,::-1])
                #THIS part only to visualize the cropped version -> somehow a little inefficient but keeps
                #the visualizer.py modular!!!
                self.publisher_img.publish(obst_image.data)

        if (self.show_bird_perspective):
            bird_perspective_image = CompressedImage()
            bird_perspective_image.header.stamp = image.header.stamp
            bird_perspective_image.format = "jpeg"
            bird_perspective_image.data = self.visualizer.visualize_bird_perspective(rectify(rgb_from_ros(image),self.intrinsics),obst_list)
            #here i want to display cropped image
            rgb_image=rgb_from_ros(obst_image.data)
            obst_image.data = d8_compressed_image_from_cv_image(rgb_image)
            #THIS part only to visualize the cropped version -> somehow a little inefficient but keeps
            #the visualizer.py modular!!!
            self.publisher_img_bird_perspective.publish(obst_image.data)


        #end = time.time()
        #print "OBST DETECTION TOOK: s"
        #print(end - start)
        self.r.sleep()
        self.thread_lock.release()

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')

# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
    rospy.init_node('obst_detection_node', anonymous=False)
    obst_detection_node = ObstDetectNode()
    rospy.on_shutdown(obst_detection_node.onShutdown)
    rospy.spin()
