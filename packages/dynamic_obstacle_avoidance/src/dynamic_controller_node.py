#!/usr/bin/env python
import os
import rospy
import numpy as np
import time
import math
from duckietown import DTROS
from std_msgs.msg import String, Float64, Float32, Float64MultiArray
from geometry_msgs.msg import Point32, Point
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, LEDPattern, FSMState
from duckietown_msgs.srv import SetCustomLEDPattern
from dynamic_obstacle_avoidance.msg import dynamic_obstacle


class Dynamic_Controller(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Dynamic_Controller, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # changing LED to two white in the front and two red in the back
        self.set_led_pattern = rospy.ServiceProxy('/%s/led_emitter_node/set_custom_pattern' % self.veh_name, SetCustomLEDPattern)
        pattern = LEDPattern()
        pattern.color_list = ["white","red","switchedoff","red","white"]
        pattern.frequency = 0.0
        self.set_led_pattern(pattern)

        self.nr_steps = 4 #nr of steps to split the d_offset during transition from one lane to another
        self.transition_time = 4.0 #sec, time for transition from one lane to another
        self.lanewidth = 0.22 #m, width of one lane
        self.leftlane_time = 3;#sec, time spent on left lane during overtaking

        self.max_vel = 7 #in m/s?? todo
        self.stop = False
        self.stop_prev = False
        self.overtaking = False

        self.head_state = False
        self.back_state = False
        self.head_veh_pose = 0
        self.head_veh_vel = 0
        self.back_veh_pose = 0
        self.back_veh_vel = 0
        self.rel_vel = 0.0

        self.duckie_left_state = False
        self.duckie_right_state = False
        self.duckie_left_pose = 4
        self.duckie_right_pose = 4

        self.d_offset = 0.0
        self.gain_calib = 1.05 #rospy.get_param("/%s/kinematics_node/gain" %self.veh_name)
        self.gain = self.gain_calib
        self.gain_overtaking = self.gain #1.3
        self.fsm_state =  "NORMAL_JOYSTICK_CONTROL"

        # construct publisher
        #self.sub_bumper = rospy.Subscriber('/%s/vehicle_detection_node/detection' %self.veh_name,BoolStamped, self.cbOvertake, queue_size=1)
        self.sub_vehicle_head_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head_state' %self.veh_name,BoolStamped, self.cbHead_state, queue_size=1)
        self.sub_vehicle_head = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_head' %self.veh_name,Float64MultiArray, self.cbHead, queue_size=1)
        self.sub_vehicle_back_state = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail_state' %self.veh_name,BoolStamped, self.cbBack_state, queue_size=1)
        self.sub_vehicle_back = rospy.Subscriber('/%s/led_detection_node/detected_duckiebot_tail' %self.veh_name,Float64MultiArray, self.cbBack, queue_size=1)
        self.sub_duckie = rospy.Subscriber('/%s/duckie_detection_node/detected_duckie' %self.veh_name, dynamic_obstacle, self.cbDuckie, queue_size=1)
        self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
        self.car_cmd_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size = 1)
        # rospy.set_param("/%s/lane_detector_node/segment_max_threshold" %self.veh_name, 15)
        # rospy.set_param("/%s/lane_filter_node/matrix_mesh_size" %self.veh_name, 0.4)

    def cbMode(self,fsm_state_msg):
        self.fsm_state = fsm_state_msg.state    # String of current FSM state
        print "fsm_state changed in dynamic_controller_node to: " , self.fsm_state

    def cbHead(self,msg):
        self.head_veh_pose = msg.data[0]
        self.head_veh_vel = msg.data[2]
        rospy.loginfo("[%s] headbot position: %f , headbot velocity: %f  " % (self.node_name, self.head_veh_pose, self.head_veh_vel))

    def cbBack(self,msg):
        self.back_veh_pose = msg.data[0] #only x vel needed?
        self.back_veh_vel = msg.data[2]
        rospy.loginfo("[%s] backbot position: %f , backbot velocity: %f  " % (self.node_name, self.back_veh_pose, self.back_veh_vel))

    def cbHead_state(self,msg):
        self.head_state = msg.data

    def cbBack_state(self,msg):
        self.back_state = msg.data

    def cbDuckie(self,msg):
        state = np.array(msg.state)
        self.duckie_right_state = any(state==1)
        self.duckie_left_state = any(state==2)

        if self.duckie_right_state:
            self.duckie_right_pose = msg.pos[2*np.argwhere(state==1)[0][0]] #only x, only take first detected duckie
            print ("duckie right pose: ", self.duckie_right_pose)
        else:
            self.duckie_right_pose = 0

        if self.duckie_left_state:
            self.duckie_left_pose = msg.pos[2*np.argwhere(state==2)[0][0]] #only x, only take first detected duckie
        else:
            self.duckie_left_pose = 0


    def cbCarCmd(self, car_cmd_msg):
        # print "hi, i'm here!!"
        car_cmd_msg_current = Twist2DStamped()
        car_cmd_msg_current = car_cmd_msg
        car_cmd_msg_current.header.stamp = rospy.Time.now()
        if self.stop or self.stop_prev:
            car_cmd_msg_current.omega = 0
            car_cmd_msg_current.v = 0
        self.stop_prev = self.stop
        self.car_cmd_pub.publish(car_cmd_msg_current)

    def overwatch(self):
        if self.fsm_state == "LANE_FOLLOWING" and not self.overtaking:
            if self.back_state or self.duckie_right_state: #if we see car before us, go to check if overtaking is possible
                rospy.loginfo("[%s] checking logic" % self.node_name)
                if not (self.head_state or self.duckie_left_state): #if no car on the left lane
                    rospy.loginfo("[%s] no car or duckie on the left lane" % self.node_name)
                    if (self.back_veh_vel < 0.5 * self.max_vel) or self.duckie_right_state:
                        rospy.loginfo("[%s] backbot slow enough or duckie_right!" % self.node_name)
                        if (self.back_veh_pose > 0.15 and self.back_veh_pose < 0.7) or (self.duckie_right_pose > 0.15 and self.duckie_right_pose < 1): #and if on the street before me, look at self.back[1]ge"
                            rospy.loginfo("[%s] backbot or duckie_right close enough for overtaking" % self.node_name)
                            self.rel_vel = 0.1  # todo!!!!!!!!!!!!!!!!
                            if self.back_state: #stay longer on the left lane if overtaking a duckiebot (which we assume is moving)
                                self.leftlane_time = 4;
                            elif self.duckie_right_state:
                                self.leftlane_time = 2;
                            self.overtake()

                if (self.back_veh_pose < 0.15 and self.back_state) or (self.duckie_right_pose < 0.15 and self.duckie_right_state):
                    rospy.logerr("[%s] Emergency stop!" % self.node_name)
                    self.stop = True
                else:
                    self.stop = False

    def overtake(self):
        self.overtaking = True
        rospy.loginfo("[%s] overtaking now, going to the left lane!" % self.node_name)

        for i in range(1,self.nr_steps+1):
            self.d_offset = math.sin(i*math.pi/(self.nr_steps*2)) * self.lanewidth
            #self.d_offset += self.lanewidth/float(self.nr_steps) #write
            rospy.set_param("/%s/lane_controller_node/d_offset" %self.veh_name, self.d_offset)
            rospy.sleep(self.transition_time/float(self.nr_steps))

        self.gain = self.gain_overtaking    #accelerate
        t_start = rospy.get_rostime().secs
        while (t_start + self.leftlane_time) > rospy.get_rostime().secs:
            if self.head_state or self.duckie_left_state: #stop if vehicle or duckie are facing us on left lane
                rospy.logerr("[%s] Emergency stop while overtaking!" % self.node_name)
                self.stop_prev = True
                self.stop = True
                while self.stop_prev or self.stop:
                    if self.head_state or self.duckie_left_state:
                        self.stop = True
                    else:
                        self.stop = False
                    rospy.sleep(0.5)

             #as soon as left lane is free again, we exit stop and continue driving on left lane
            rospy.sleep(0.1)                     #time on left lane, make dependend on self.rel_vel
        self.gain = self.gain_calib         #decelerate
        rospy.loginfo("[%s] going back to the right lane" % self.node_name)

        for i in range(1,self.nr_steps+1):
            self.d_offset = (1 - math.sin(i*math.pi/(self.nr_steps*2)) )* self.lanewidth
            #self.d_offset -= self.lanewidth/float(self.nr_steps) #write
            rospy.set_param("/%s/lane_controller_node/d_offset" %self.veh_name, self.d_offset)
            rospy.sleep(self.transition_time/float(self.nr_steps))
        self.d_offset = 0.0

        rospy.loginfo("[%s] setting d_offset to zero!" % self.node_name)
        self.overtaking = False

    def run(self):
        # publish message every 0.1 second
        rate = rospy.Rate(50) # 1Hz
        while not rospy.is_shutdown():
            self.overwatch()
            # rospy.set_param("/%s/lane_controller_node/d_offset" %self.veh_name, self.d_offset)
            # rospy.set_param("/%s/kinematics_node/gain" %self.veh_name, self.gain)

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = Dynamic_Controller(node_name='dynamic_controller_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
