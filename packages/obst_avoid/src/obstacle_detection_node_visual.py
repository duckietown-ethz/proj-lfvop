#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import MarkerArray, Marker
import message_filters

### note you need to change the name of the robot to yours here
from obst_avoid.detector import Detector
from obst_avoid.visualizer import Visualizer
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics


class ObstDetectNodeVisual(object):
    """
    Visualisation of Obstacle Detection Node
    """

    def __init__(self):
        self.node_name = "Obstacle Detecion Node"
        robot_name = rospy.get_param("~veh", "")
        # robot_name = "dori"
        self.show_marker = (rospy.get_param("~show_marker", ""))
        # self.show_marker=True
        self.show_image = (rospy.get_param("~show_image", ""))
        # bounding box parameters in mm
        self.show_bb = (rospy.get_param("~show_bb", ""))
        self.bb_len = (rospy.get_param("~bb_len", ""))
        self.bb_wid = (rospy.get_param("~bb_wid", ""))
        print 'Here comes bb_len'
        print self.bb_len

        # Load camera calibration parameters
        self.intrinsics = load_camera_intrinsics(robot_name)
        self.visualizer = Visualizer(robot_name=robot_name)

        # Create Publishers
        if (self.show_marker):
            self.pub_topic_marker = '/{}/obst_detect_visual/visualize_obstacles'.format(robot_name)
            self.publisher_marker = rospy.Publisher(self.pub_topic_marker, MarkerArray, queue_size=1)
            print "YEAH I GIVE YOU THE MARKER"

        if (self.show_image):
            self.pub_topic_img = '/{}/obst_detect_visual/image/compressed'.format(robot_name)
            self.publisher_img = rospy.Publisher(self.pub_topic_img, CompressedImage, queue_size=1)
            print "YEAH I GIVE YOU THE IMAGE"

        # if (self.show_bb):
        self.pub_topic_bb_linelist = '/{}/obst_detect_visual/bb_linelist'.format(robot_name)
        self.publisher_bblinelist = rospy.Publisher(self.pub_topic_bb_linelist, Marker, queue_size=1)
        print "YEAH I GIVE YOU THE BOUNDINGBOXMARKERLIST"

        self.bbmarker = Marker()
        self.bbmarker.header.frame_id = '{}'.format(robot_name)
        # self.bbmarker.ns = "points_and_lines"
        # self.bbmarker.id = 0;
        self.bbmarker.action = Marker.ADD
        self.bbmarker.type = Marker.LINE_STRIP
        self.bbmarker.lifetime = rospy.Time(10.0)
        self.bbmarker.pose.orientation.w = 1.0
        self.bbmarker.scale.x = 0.02
        self.bbmarker.color.b = 1.0  # blue
        self.bbmarker.color.a = 1.0  # alpha

        corner1 = Point()
        corner2 = Point()
        corner3 = Point()
        corner4 = Point()
        corner1.y = -self.bb_wid / 2000
        corner1.x = 0
        corner2.y = -self.bb_wid / 2000
        corner2.x = self.bb_len / 1000
        corner3.y = self.bb_wid / 2000
        corner3.x = self.bb_len / 1000
        corner4.y = self.bb_wid / 2000
        corner4.x = 0

        self.bbmarker.points.append(corner1)
        self.bbmarker.points.append(corner2)
        self.bbmarker.points.append(corner3)
        self.bbmarker.points.append(corner4)
        self.bbmarker.points.append(corner1)

        # for i in range(1,10000):
        #    self.publisher_bblinelist.publish(self.bbmarker)

        self.sub_topic_arr = '/{}/obst_detect/posearray'.format(robot_name)
        self.subscriber_arr = message_filters.Subscriber(self.sub_topic_arr, PoseArray)
        # we MUST subscribe to the array for sure!!
        if (self.show_image):
            self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
            self.subscriber = message_filters.Subscriber(self.sub_topic, CompressedImage)

        if (self.show_marker and not (self.show_image)):
            self.subscriber_arr.registerCallback(self.marker_only_callback)
        else:
            self.ts = message_filters.TimeSynchronizer([self.subscriber_arr, self.subscriber], 100)
            self.ts.registerCallback(self.callback)

    def callback(self, obst_list, image):
        self.publisher_bblinelist.publish(self.bbmarker)  # since a single publish is likely to get lost
        # print "CALLBACK HERE"
        if (self.show_marker):
            marker_list = self.visualizer.visualize_marker(obst_list)
            self.publisher_marker.publish(marker_list)

        # 4. EVENTUALLY DISPLAY IMAGE
        if (self.show_image):
            obst_image = CompressedImage()
            obst_image.header.stamp = image.header.stamp
            obst_image.format = "jpeg"
            obst_image.data = self.visualizer.visualize_image(rectify(rgb_from_ros(image), self.intrinsics), obst_list)
            self.publisher_img.publish(obst_image.data)

    def marker_only_callback(self, obst_list):
        # print "CALLBACK HERE"
        marker_list = self.visualizer.visualize_marker(obst_list)
        self.publisher_marker.publish(marker_list)

    def onShutdown(self):
        rospy.loginfo('Shutting down Visualisation of Obstacle Detection')


# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
    rospy.init_node('obst_detection_node_visual', anonymous=False)
    obst_detection_node = ObstDetectNodeVisual()
    rospy.on_shutdown(obst_detection_node.onShutdown)
    rospy.spin()
