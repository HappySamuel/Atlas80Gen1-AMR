#!/usr/bin/env python

'''
Author :  Samuel Chieng Kien Ho
Function :  Turning to desired heading -180' ~ +180' [deg]
'''

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class HeadingTurner():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.do_turn = False
        self.desired_yaw = None    # [deg]
        self.yaw_tolerance = 10    # [deg]

        # Subscribers
        self.init_turn_sub = rospy.Subscriber(rospy.get_param("~init_turn_topic"), String, self.init_turnCB, queue_size=1)
        self.odom_sub = rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self.odomCB, queue_size=1)

        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.finish_turn_pub = rospy.Publisher(rospy.get_param("~finish_turn_topic"), String, queue_size=1)

    # Checking When to Initiate Turning and What Heading is desired
    def init_turnCB(self, msg):
        if(msg.data != ""):
            self.do_turn = True
            self.desired_yaw = float(msg.data)
            if(self.desired_yaw >= 180):
                self.desired_yaw = self.desired_yaw - 2.0*180
            elif(self.desired_yaw <= -180):
                self.desired_yaw = self.desired_yaw + 2.0*180

    # Checking Current Vehicle's yaw angle
    def odomCB(self, msg):
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if(self.do_turn == True and self.desired_yaw != None):
            # Turning Left (+)
            if(self.desired_yaw > yaw):
                if(self.yaw_checker(self.desired_yaw, yaw)):
                    self.apply_control(0.5)
                    self.finish_turn_pub.publish("1")    # On Going
                else:
                    self.finish_turn_pub.publish("2")    # Done Turning
                    self.do_turn = False
                    self.desired_yaw = None
                    rospy.sleep(2)    # pause for 2 sec
            # Turning Right (-)
            elif(self.desired_yaw < yaw):
                if(self.yaw_checker(self.desired_yaw, yaw)):
                    self.apply_control(-0.5)
                    self.finish_turn_pub.publish("1")    # On Going
                else:
                    self.finish_turn_pub.publish("2")    # Done Turning
                    self.do_turn = False
                    self.desired_yaw = None
                    rospy.sleep(2)    # pause for 2 sec
        else:
            self.finish_turn_pub.publish("0")    # Do Nothing    

    # Yaw Checker [deg]
    def yaw_checker(self, a, b):
        yaw_difference = a - b
        print "desired_yaw :  " + str(a)
        print "current_yaw :  " + str(b)
        print "yaw_difference :  " + str(yaw_difference)
        print "-----------------------------------------"
        if(abs(yaw_difference) > self.yaw_tolerance):
            return True    # Continue
        else:
            return False   # Done

    # Convert Quaternion to Yaw [deg]
    def quaternion_to_yaw(self, q):
	x, y, z, w = q.x, q.y, q.z, q.w
	roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion((x, y, z, w)))
	return yaw

    # Constant rotating at +/- 0.4 [rad/s]
    def apply_control(self, left_right):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, left_right)    # Turning Left (+) or Right (-)
        self.drive_pub.publish(drive_msg)



if __name__=="__main__":
    rospy.init_node("heading_turner")
    HeadingTurner()
    rospy.spin()
