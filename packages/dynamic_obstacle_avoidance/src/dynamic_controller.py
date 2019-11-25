#!/usr/bin/env python

import os
import rospy
import numpy as np
from duckietown import DTROS
from std_msgs.msg import String, Float64, Float32, Float64MultiArray
from geometry_msgs.msg import Point32, Point
from duckietown_msgs.msg import BoolStamped


class Dynamic_Controller(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Dynamic_Controller, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.stepsize = 4
        self.transition_time = 4.0 #sec
        self.lanewidth = 0.2175

        self.duckieDist = 100
        self.duckieSide = 100
        self.max_vel = 7 #in m/s?? todo

        self.duckieDetected=False
        self.head_state = False
        self.tail_state = False
        self.head_veh_pose = np.zeros((2,1))
        self.head_veh_vel = np.zeros((2,1))
        self.tail_veh_pose = np.zeros((2,1))
        self.tail_veh_vel = np.zeros((2,1))
        self.rel_vel = 0.0

        self.offset = 0.0

        # construct publisher
        self.pub_doffset = rospy.Publisher('lane_controller_node/doffset', Float64, queue_size=1)
        #self.sub_bumper = rospy.Subscriber('/%s/vehicle_detection_node/detection' %self.veh_name,BoolStamped, self.cbOvertake, queue_size=1)
        self.sub_vehicle_head_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head_state' %self.veh_name,BoolStamped, self.cbHead_state, queue_size=1)
        self.sub_vehicle_head = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head' %self.veh_name,Float64MultiArray, self.cbHead, queue_size=1)
        self.sub_vehicle_tail_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail_state' %self.veh_name,BoolStamped, self.cbTail_state, queue_size=1)
        self.sub_vehicle_tail = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail' %self.veh_name,Float64MultiArray, self.cbTail, queue_size=1)
        self.sub_duckie_detect = rospy.Subscriber('/%s/duckie_detection_node/detection' %self.veh_name,BoolStamped, self.cbDuckieDetected, queue_size=1)
        self.sub_duckie_point = rospy.Subscriber('/%s/duckie_detection_node/detected_duckie_point' %self.veh_name,Point, self.cbDuckie, queue_size=1)

    def cbHead(self,msg):
        self.head_veh_pose = msg.data[0]
        self.head_vel = msg.data[2]
	print ("headbot position: ", self.head_veh_pose)
        print ("headbot velocity: ", self.head_vel)

    def cbTail(self,msg):
        self.tail_veh_pose = msg.data[0] #only x vel needed?
        self.tail_vel = msg.data[2]
        print ("tailbot position: ", self.tail_veh_pose)
        print ("tailbot velocity: ", self.tail_vel)

    def cbHead_state(self,msg):
        self.head_state = msg.data

    def cbTail_state(self,msg):
        self.tail_state = msg.data
        if self.tail_state: #if we see car before us, go to check if overtaking is possible
            print "starting overwatch"
            self.overwatch()

    def cbDuckie(self,msg):
        self.duckieDist=msg.y
        self.duckieSide=msg.x

    def cbDuckieDetected(self,msg):
        self.duckieDetected=msg.data

    def overwatch(self):
        if self.tail_vel < 0.5 * self.max_vel:
            print "tailbot slower than 0.5 * max_vel "
            if self.tail_veh_pose > 0.15 and self.tail_veh_pose < 1: #and if on the street before me, look at self.tail[1]
                print "tailbot is with in overtaking range"
                if not self.head_state: #if no car on the left lane
                    self.rel_vel = 0.1  # todo!!!!!!!!!!!!!!!!
                    self.overtake()


    def overtake(self):
        print "overtaking now!"
        for i in range(0,self.stepsize):
            self.offset += self.lanewidth/float(self.stepsize) #write
            rospy.sleep(self.transition_time/float(self.stepsize))

        rospy.sleep(3.) #time on left lane, make dependend on self.rel_vel
        print "going back to the right lane"
        for i in range(0,self.stepsize):
            self.offset -= self.lanewidth/float(self.stepsize) #write
            rospy.sleep(self.transition_time/float(self.stepsize))
        self.offset = 0.0

    def run(self):
        # publish message every 0.1 second
        rate = rospy.Rate(50) # 1Hz
        while not rospy.is_shutdown():
            message = self.offset
            #rospy.loginfo("Offset published: '%f'" % message)
            self.pub_doffset.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = Dynamic_Controller(node_name='dynamic_controller_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
