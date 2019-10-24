#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped


class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")

        offset_mode = 0
        offset_mode = int(os.environ['OFFSET'])
        if offset_mode == 0:
            self.offset = 0.0
        elif offset_mode == 1:
            self.offset = 0.2175    # lane distance
        elif offset_mode == 2:
            self.offset = 0.115     # middle of the road
        else:
            self.offset = 0.0

        # construct publisher
        self.pub = rospy.Publisher('lane_controller_node/doffset', Float64, queue_size=1)
        self.sub = rospy.Subscriber('/%s/vehicle_detection_node/detection' %self.veh_name,BoolStamped, self.cbOvertake, queue_size=1)

    def cbOvertake(self,msg):
        if msg.data:
            print "detected vehicle"
            self.offset =  0.2175/4*1 #write
            rospy.sleep(0.5)
            self.offset =  0.2175/4*2 #write
            rospy.sleep(0.5)
            self.offset =  0.2175/4*3 #write
            rospy.sleep(0.5)
            self.offset =  0.2175/4*4 #write
            rospy.sleep(7.)
            print "going back to the right lane"
            self.offset =  0.2175/4*3 #write
            rospy.sleep(0.5)
            self.offset =  0.2175/4*2 #write
            rospy.sleep(0.5)
            self.offset =  0.2175/4*1 #write
            rospy.sleep(0.5)
            self.offset =  0.0

    def run(self):
        # publish message every 0.1 second
        rate = rospy.Rate(50) # 1Hz
        while not rospy.is_shutdown():
            message = self.offset
            #rospy.loginfo("Offset published: '%f'" % message)
            self.pub.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
