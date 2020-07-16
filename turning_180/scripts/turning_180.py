#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  Doing 180' Turn
'''

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class Turning180():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.last_msg = False
        self.do_turn = False
        self.yaw_tolerance = 8    # [deg]
        self.yaw = 0    # [deg]
        self.end_yaw = 180    # [deg]

        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.finish_180_turn_pub = rospy.Publisher(rospy.get_param("~finish_180_turn_topic"), String, queue_size=1)

        # Subscribers
        self.init_180_turn_sub = rospy.Subscriber(rospy.get_param("~init_180_turn_topic"), Bool, self.init_180_turnCB, queue_size=1)
        self.odom_sub = rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self.odomCB, queue_size=1)

    # Checking When to Initiate 180' Turning
    def init_180_turnCB(self, msg):
        if(self.last_msg != msg.data and msg.data == True):
            self.do_turn = True
            self.end_yaw = self.yaw + 170
            if(self.end_yaw >= 180):
                self.end_yaw -= 360
            elif(self.end_yaw <= -180):
                self.end_yaw += 360
        self.last_msg = msg.data

    # Checking Current Vehicle's yaw angle
    def odomCB(self, msg):
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if(self.do_turn == True):
            if(self.yaw_checker(self.end_yaw, self.yaw)):
                self.apply_control(-0.5)
                self.finish_180_turn_pub.publish("1")    # On Going
            else:
                self.apply_control(0.0)
                self.finish_180_turn_pub.publish("2")    # Done 180' Turning
                self.do_turn = False
                self.last_msg = False
                rospy.sleep(2) # pause for 2 sec
        else:
            self.finish_180_turn_pub.publish("0")    # Do Nothing

    # Yaw Checker [deg]
    def yaw_checker(self, a, b):
        yaw_difference = a - b
        print "end_yaw :  " + str(a)
        print "current_yaw :  " + str(b)
        print "yaw_difference :  " + str(yaw_difference)
        print "-----------------------------------------"
        if(yaw_difference >= 180):
            yaw_difference -= 360
        elif(yaw_difference <= -180):
            yaw_difference += 360
        if(abs(yaw_difference) > self.yaw_tolerance):
            return True    # Continue
        else:
            return False   # Done

    # Convert Quaternion to Yaw [deg]
    def quaternion_to_yaw(self, q):
	x, y, z, w = q.x, q.y, q.z, q.w
	roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion((x, y, z, w)))
	return yaw

    # Constant rotating at designated rotation speed [rad/s]
    def apply_control(self, rot_speed):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0,0,0)
        drive_msg.angular = Vector3(0,0,rot_speed)    # "+" <--> CCW | "-" <--> CW
        self.drive_pub.publish(drive_msg)


if __name__=="__main__":
    rospy.init_node("turning_180")
    Turning180()
    rospy.spin()
