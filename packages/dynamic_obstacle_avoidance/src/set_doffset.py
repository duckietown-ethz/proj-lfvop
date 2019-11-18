#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64, Float32
from geometry_msgs.msg import Point32, Point
from duckietown_msgs.msg import BoolStamped


class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.stepsize = 4
        self.transition_time = 2.0 #sec
        self.lanewidth = 0.2175
        self.vehDist = 100 #just for init
        self.duckieDist = 100
        self.duckieSide = 100
        self.vehDetected=False
        self.duckieDetected=False

        offset_mode = 0
        if 'OFFSET' in os.environ:
            offset_mode = int(os.environ['OFFSET'])
        if offset_mode == 0:
            self.offset = 0.0
        elif offset_mode == 1:
            self.offset = self.lanewidth    # lane distance
        elif offset_mode == 2:
            self.offset = self.lanewidth/2     # middle of the road
        else:
            self.offset = 0.0

        # construct publisher
        self.pub_doffset = rospy.Publisher('lane_controller_node/doffset', Float64, queue_size=1)
        #self.sub_bumper = rospy.Subscriber('/%s/vehicle_detection_node/detection' %self.veh_name,BoolStamped, self.cbOvertake, queue_size=1)
        self.sub_led_detect = rospy.Subscriber('/%s/led_detection_node/detection' %self.veh_name,BoolStamped, self.cbLedDetected, queue_size=1)
        self.sub_vehicle_distance = rospy.Subscriber('/%s/led_detection_node/detected_vehicle_distance' %self.veh_name,Float32, self.cbVehicle, queue_size=1)
        self.sub_duckie_detect = rospy.Subscriber('/%s/duckie_detection_node/detection' %self.veh_name,BoolStamped, self.cbDuckieDetected, queue_size=1)
        self.sub_duckie_point = rospy.Subscriber('/%s/duckie_detection_node/detected_duckie_point' %self.veh_name,Point, self.cbDuckie, queue_size=1)

    def cbVehicle(self,msg):
        self.vehDist=msg.data
        self.Overtake()

    def cbDuckie(self,msg):
        self.duckieDist=msg.y
        self.duckieSide=msg.x
        self.Overtake()

    def cbLedDetected(self,msg):
        self.vehDetected=msg.data

    def cbDuckieDetected(self,msg):
        self.duckieDetected=msg.data

    def Overtake(self):
        if (self.vehDetected and self.vehDist<0.7 and self.vehDist>0.1):
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
