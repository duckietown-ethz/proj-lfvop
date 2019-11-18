#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64, Float32, Float64MultiArray
from geometry_msgs.msg import Point32, Point
from duckietown_msgs.msg import BoolStamped


class dynamic_controller(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(dynamic_controller, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.stepsize = 4
        self.transition_time = 2.0 #sec
        self.lanewidth = 0.2175
        self.vehDist = 100 #just for init
        self.duckieDist = 100
        self.duckieSide = 100
        self.max_vel = 1

        self.vehDetected=False
        self.duckieDetected=False
        self.head_state = False
        self.tail_state = False
        self.head_veh_pose = np.zeros(2,1)
        self.head_veh_vel = np.zeros(2,1)
        self.tail_veh_pose = np.zeros(2,1)
        self.tail_veh_vel = np.zeros(2,1)
        self.rel_vel = 0.0

        self.offset = 0.0

        # construct publisher
        self.pub_doffset = rospy.Publisher('lane_controller_node/doffset', Float64, queue_size=1)
        #self.sub_bumper = rospy.Subscriber('/%s/vehicle_detection_node/detection' %self.veh_name,BoolStamped, self.cbOvertake, queue_size=1)
        #self.sub_led_detect = rospy.Subscriber('/%s/led_detection_node/detection' %self.veh_name,BoolStamped, self.cbLedDetected, queue_size=1)
        self.sub_vehicle_head_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head_state' %self.veh_name,BoolStamped, self.cbHead_state, queue_size=1)
        self.sub_vehicle_head = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head' %self.veh_name,Float64MultiArray, self.cbHead, queue_size=1)
        self.sub_vehicle_tail_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail_state' %self.veh_name,BoolStamped, self.cbTail_state, queue_size=1)
        self.sub_vehicle_tail = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail' %self.veh_name,Float64MultiArray, self.cbTail, queue_size=1)
        self.sub_duckie_detect = rospy.Subscriber('/%s/duckie_detection_node/detection' %self.veh_name,BoolStamped, self.cbDuckieDetected, queue_size=1)
        self.sub_duckie_point = rospy.Subscriber('/%s/duckie_detection_node/detected_duckie_point' %self.veh_name,Point, self.cbDuckie, queue_size=1)

    def cbHead(self,msg):
        self.head_veh_pose = msg.data[0:2]
        self.head_vel = msg.data[2:4]

    def cbTail(self,msg):
        self.tail_veh_pose = msg.data[0:1]
        self.tail_vel = msg.data[2:4]

    def cbHead_state(self,msg):
        self.head_state = msg.data

    def cbTail_state(self,msg):
        self.tail_state = msg.data
        if self.tail_state:
            self.overwatch()

    def cbVehicle(self,msg):
        self.vehDist=msg.data
        self.overtake()

    def cbDuckie(self,msg):
        self.duckieDist=msg.y
        self.duckieSide=msg.x
        self.overtake()

    def cbLedDetected(self,msg):
        self.vehDetected=msg.data

    def cbDuckieDetected(self,msg):
        self.duckieDetected=msg.data


    def overwatch(self):
        if self.tail_vel < 0.5 * self.max_vel:
            if self.tail[0] < 0.15 and self.tail[0] > 0.7:
                if self.head_state:
                    self.rel_vel = 0.1  # todo!!!!!!!!!!!!!!!!
                    ovetake()


    def overtake(self):
        print "overtaking now!"
        for i in range(0,self.stepsize):
            self.offset += self.lanewidth/float(self.stepsize) #write
            rospy.sleep(self.transition_time/float(self.stepsize))

        rospy.sleep(7.)
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
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
