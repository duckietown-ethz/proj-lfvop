#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import WheelsCmdStamped, VehiclePose, FSMState, Twist2DStamped, BoolStamped

class VehicleAvoidanceControlNodeTest:

    def __init__(self):
        self.node_name = "Vehicle Avoidance Control Node Test"

        self.wheels_cmd_sub = rospy.Subscriber("vehicle_avoidance_control_node/car_cmd", Twist2DStamped, self.cbCmd, queue_size = 1)
        self.vehicle_detected_sub = rospy.Subscriber("~vehicle_detected",BoolStamped, self.cbDetected, queue_size=1)
        self.in_lane_pub = rospy.Publisher("~in_lane",Bool, queue_size=1)
        self.fsm_mode_sub = rospy.Subscriber("~mode",FSMState,self.cbMode,queue_size=1)
        self.wheel_cmd_switch_sub = rospy.Subscriber("~switch_commands",WheelsCmdStamped,self.cbWheelSwitch, queue_size=1)
        self.pose_pub = rospy.Publisher("~vehicle_pose",VehiclePose, queue_size = 1)
        self.car_cmd_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size = 1)

        self.rho = 0.1

        rospy.loginfo("Initialization of [%s] completed" % (self.node_name))

        rospy.Timer(rospy.Duration.from_sec(1.0), self.pubPose)
        rospy.Timer(rospy.Duration.from_sec(1.0), self.pubInLane)
        rospy.Timer(rospy.Duration.from_sec(1.0), self.pubCarCmd)

    def pubPose(self,args=None):
        pose_msg_out = VehiclePose()

        pose_msg_out.header.stamp = rospy.Time.now()
        pose_msg_out.rho.data = self.rho
        pose_msg_out.theta.data = 0.0
        pose_msg_out.psi.data = 0.0

        self.rho = self.rho + 0.1

        self.pose_pub.publish(pose_msg_out)

    def pubInLane(self,args=None):
        self.in_lane_pub.publish(True)
        
    def pubCarCmd(self,args=None):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = 0.5
        car_cmd_msg.omega = 0.0
        self.car_cmd_pub.publish(car_cmd_msg)

    def cbWheelSwitch(self,switch_msg):
        rospy.loginfo('SWITCH OUTPUT : (left = %.2f, right = %.2f)' % 
            (switch_msg.vel_left, switch_msg.vel_right))

    def cbCmd(self, cmd_msg):
        rospy.loginfo('Command received : (v = %.2f, omega = %.2f)' %
                    (cmd_msg.v, cmd_msg.omega))

    def cbDetected(self,detected_msg):
        rospy.loginfo('Vehicle detected? : %r' %
                    (detected_msg.data))

    def cbMode(self, mode_msg):
        rospy.loginfo('Mode : %d' % (mode_msg.state))

if __name__ == '__main__':
    rospy.init_node('vehicle_avoidance_control_node_test', anonymous=False)
    vehicle_avoidance_control_node_test = VehicleAvoidanceControlNodeTest()
    rospy.spin()