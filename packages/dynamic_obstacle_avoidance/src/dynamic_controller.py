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
        self.lanewidth = 0.22 #0.2175


        self.max_vel = 7 #in m/s?? todo

        self.duckie_state=False
        self.head_state = False
        self.tail_state = False
        self.duckie_pose = np.zeros((2,1))
        self.head_veh_pose = np.zeros((2,1))
        self.head_veh_vel = np.zeros((2,1))
        self.tail_veh_pose = np.zeros((2,1))
        self.tail_veh_vel = np.zeros((2,1))
        self.rel_vel = 0.0

        self.d_offset = 0.0
        self.gain_calib = rospy.get_param("/%s/kinematics_node/gain" %self.veh_name)
        self.gain = self.gain_calib
        self.gain_overtaking = 1.3

        # construct publisher
        #self.sub_bumper = rospy.Subscriber('/%s/vehicle_detection_node/detection' %self.veh_name,BoolStamped, self.cbOvertake, queue_size=1)
        self.sub_vehicle_head_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head_state' %self.veh_name,BoolStamped, self.cbHead_state, queue_size=1)
        self.sub_vehicle_head = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head' %self.veh_name,Float64MultiArray, self.cbHead, queue_size=1)
        self.sub_vehicle_tail_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail_state' %self.veh_name,BoolStamped, self.cbTail_state, queue_size=1)
        self.sub_vehicle_tail = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail' %self.veh_name,Float64MultiArray, self.cbTail, queue_size=1)
        self.sub_duckie_state = rospy.Subscriber('/%s/duckie_detection_node/detected_duckie_state' %self.veh_name,BoolStamped, self.cbDuckie_state, queue_size=1)
        self.sub_duckie_location = rospy.Subscriber('/%s/duckie_detection_node/detected_duckie_location' %self.veh_name,Float64MultiArray, self.cbDuckie, queue_size=1)

    def cbHead(self,msg):
        self.head_veh_pose = msg.data[0]
        self.head_vel = msg.data[2]
        rospy.loginfo("[%s] headbot position: %f , headbot velocity: %f  " % (self.node_name, self.head_veh_pose, self.head_vel))

    def cbTail(self,msg):
        self.tail_veh_pose = msg.data[0] #only x vel needed?
        self.tail_vel = msg.data[2]
        rospy.loginfo("[%s] tailbot position: %f , tailbot velocity: %f  " % (self.node_name, self.tail_veh_pose, self.tail_vel))

    def cbHead_state(self,msg):
        self.head_state = msg.data

    def cbTail_state(self,msg):
        self.tail_state = msg.data
        if self.tail_state: #if we see car before us, go to check if overtaking is possible
            self.overwatch()

    def cbDuckie(self,msg):
        self.duckie_pose=msg.data
        print ("duckie position: ", self.duckie_pose)

    def cbDuckie_state(self,msg):
        self.duckie_state=msg.data

    def overwatch(self):
        rospy.loginfo("[%s] starting overwatch.." % self.node_name)
        if self.tail_vel < 0.5 * self.max_vel:
            rospy.loginfo("[%s] velocity requirement fullfilled!" % self.node_name)
            if self.tail_veh_pose > 0.15 and self.tail_veh_pose < 1: #and if on the street before me, look at self.tail[1]ge"
                rospy.loginfo("[%s] tailbot close enough for overtaking" % self.node_name)
                if not self.head_state: #if no car on the left lane
                    rospy.loginfo("[%s] no car on the left lane, start overtaking" % self.node_name)
                    self.rel_vel = 0.1  # todo!!!!!!!!!!!!!!!!
                    self.overtake()

    def overtake(self):
        rospy.loginfo("[%s] overtaking now, going to the left lane!" % self.node_name)
        for i in range(0,self.stepsize):
            self.d_offset += self.lanewidth/float(self.stepsize) #write
            rospy.sleep(self.transition_time/float(self.stepsize))

        self.gain = self.gain_overtaking    #accelerate
        rospy.sleep(3.)                     #time on left lane, make dependend on self.rel_vel
        self.gain = self.gain_calib         #decelerate
        rospy.loginfo("[%s] going back to the right lane" % self.node_name)

        for i in range(0,self.stepsize):
            self.d_offset -= self.lanewidth/float(self.stepsize) #write
            rospy.sleep(self.transition_time/float(self.stepsize))
        self.d_offset = 0.0
        rospy.loginfo("[%s] setting d_offset to zero!" % self.node_name)

    def run(self):
        # publish message every 0.1 second
        rate = rospy.Rate(50) # 1Hz
        while not rospy.is_shutdown():
            rospy.set_param("/%s/lane_controller_node/d_offset" %self.veh_name, self.d_offset)
            rospy.set_param("/%s/kinematics_node/gain" %self.veh_name, self.gain)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = Dynamic_Controller(node_name='dynamic_controller_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
