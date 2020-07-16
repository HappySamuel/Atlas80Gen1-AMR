#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  Doing Left/Right 90' Turn
'''

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class Turning90():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.last_msg = ""
        self.do_turn = 0     # 0 - no turn | 1 - left turn | -1 - right turn
        self.yaw_tolerance = 8    # [deg]
        self.yaw = 0              # [deg]
        self.end_yaw = 90         # [deg]

        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.finish_90_turn_pub = rospy.Publisher(rospy.get_param("~finish_90_turn_topic"), String, queue_size=1)

        # Subscribers
        self.init_90_turn_sub = rospy.Subscriber(rospy.get_param("~init_90_turn_topic"), String, self.init_90_turnCB, queue_size=1)
        self.odom_sub = rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self.odomCB, queue_size=1)

    # Checking When to Initiate 90' Turning Left (+) or Right (-)
    def init_90_turnCB(self, msg):
        if(msg.data != self.last_msg):
            if(msg.data == "left"):
                self.do_turn = 1
                self.end_yaw = self.yaw + 90
            elif(msg.data == "right"):
                self.do_turn = -1
                self.end_yaw = self.yaw - 90
            elif(msg.data == "cancel"):
                self.do_turn = 0
            if(self.end_yaw >= 180):
                self.end_yaw -= 360
            elif(self.end_yaw <= -180):
                self.end_yaw += 360
        print self.last_msg
        self.last_msg = msg.data

    # Checking Current Vehicle's yaw angle
    def odomCB(self, msg):
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        # Left Turn
        if(self.do_turn == 1):
            if(self.yaw_checker(self.end_yaw, self.yaw)):
                self.apply_control(0.5)
                self.finish_90_turn_pub.publish("1")    # On Going
            else:
                self.finish_90_turn_pub.publish("2")    # Done 90' Turning
                self.do_turn = 0
                self.last_msg = ""
        # Right Turn
        elif(self.do_turn == -1):
            if(self.yaw_checker(self.end_yaw, self.yaw)):
                self.apply_control(-0.5)
                self.finish_90_turn_pub.publish("1")    # On Going
            else:
                self.finish_90_turn_pub.publish("2")    # Done 90' Turning
                self.do_turn = 0
                self.last_msg = ""
        # No Turn
        else:
            self.finish_90_turn_pub.publish("2")    # Do Nothing

    # Yaw Checker [deg]
    def yaw_checker(self, a, b):
        yaw_difference = a - b
        print "end_yaw :  " + str(a)
        print "current_yaw :  " + str(b)
        print "yaw_difference :  " + str(yaw_difference)
        print "-----------------------------------------"
        if(abs(yaw_difference) > self.yaw_tolerance):
            return True    # Continue
        else:
            return False   # Done

    # Convert Quaternion to Yaw [deg]
    def quaternion_to_yaw(self, q):
	roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion((q.x, q.y, q.z, q.w)))
	return yaw

    # Constant rotating at +/- 0.4 [rad/s]
    def apply_control(self, left_right):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, left_right)    # Turning Left (+) or Right (-)
        self.drive_pub.publish(drive_msg)



if __name__=="__main__":
    rospy.init_node("turning_90")
    Turning90()
    rospy.spin()
