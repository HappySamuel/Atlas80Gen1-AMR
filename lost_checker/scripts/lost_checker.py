#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tftr
import tf

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class LostChecker():
    def __init__(self):
        # Define Adjustable Parameters
        self.dist_threshold = rospy.get_param("~dist_threshold", 2) 
        self.yaw_threshold = rospy.get_param("~yaw_threshold", 30)

        # Internal USE Variables - Do not modify without consultation
        self.checker_rate = rospy.Rate(1)   # 0.5 Hz  <-->  2 sec
        self.last_1_msg = 0
        self.last_2_msg = 0
        self.last_1_pos = None
        self.last_2_pos = None
        self.dist_1_travelled = None
        self.dist_2_travelled = None
        self.lost = False

        # Subscribers
        self.odom_1_sub = rospy.Subscriber(rospy.get_param("~odom_1_topic", "pf/pose/odom"), Odometry, self.odom_1CB, queue_size=1)
        self.odom_2_sub = rospy.Subscriber(rospy.get_param("~odom_2_topic", "odometry/filtered_on_map"), Odometry, self.odom_2CB, queue_size=1)
        self.recover_sub = rospy.Subscriber(rospy.get_param("~recover_topic", "atlas80/recovered"), Bool, self.recoverCB, queue_size=1)

        # Publishers
        self.error_pub = rospy.Publisher("lost_checker/error", String, queue_size=1)
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic","low_level/twist_cmd_mux/input/lost"), Twist, queue_size=1)

        # Main Loop
        self.checker_loop()

    # 1st Odometry CallBack
    def odom_1CB(self, msg):
        odom_1_pos = msg.pose.pose.position
        odom_1_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if((msg.header.stamp.sec - self.last_1_msg) >= 2):
            self.dist_1_travelled = self.dist_accumulator(odom_1_pos, self.last_1_pos)
            self.last_1_msg = msg.header.stamp.sec
            self.last_1_pos = msg.pose.pose.position

    # 2nd Odometry CallBack
    def odom_2CB(self, msg):
        odom_2_pos = msg.pose.pose.position
        odom_2_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        if((msg.header.stamp.sec - self.last_2_msg) >= 2):
            self.dist_2_travelled = self.dist_accumulator(odom_2_pos, self.last_2_pos)
            self.last_2_msg = msg.header.stamp.sec
            self.last_2_pos = msg.pose.pose.position

    # Recover CallBack
    def recoverCB(self, msg):
        if(msg.data == True):
            self.reset()

    # Checker Loop
    def checker_loop(self):
        while not rospy.is_shutdown():
            print "testing..."
            if(self.dist_1_travelled != None and self.dist_2_travelled != None):
                self.check()
                self.reset()
            self.checker_rate.sleep()

    # Checking Status
    def check(self):
        if(abs(self.dist_1_travelled - self.dist_2_travelled) > self.dist_threshold):
            self.lost = True
        if self.lost:
            self.error_pub.publish("lose_track")
            self.stopping()
        else:
            self.error_pub.publish("ok")

    # Reset Variables
    def reset(self):
        self.last_1_msg = 0
        self.last_2_msg = 0
        self.last_1_pos = None
        self.last_2_pos = None
        self.dist_1_travelled = None
        self.dist_2_travelled = None
        self.lost = False

    # Distance Travelled [m] for fixed period
    def dist_accumulator(self, now_p, last_p):
        if not isinstance(last_p, None):
            total_dist = self.distance(now_p, last_p)
            return total_dist
        else:
            return 0

    # Distance Calculator [m]
    def distance(self, a, b):
        dist = np.sqrt((a.x-b.x)**2 +(a.y-b.y)**2)
        return dist

    # Convert Quaternion to Yaw [deg]
    def quaternion_to_yaw(self, q):
        yaw = np.rad2deg(tftr.euler_from_quaternion((q.x, q.y, q.z, q.w)))
        return yaw

    # Stop the Vehicle
    def stopping(self):
        cmd = Twist()
        cmd.linear = Vector3(0,0,0)
        cmd.angular = Vector3(0,0,0)
        self.drive_pub.publish(cmd)



if __name__=="__main__":
    rospy.init_node("lost_checker")
    LostChecker()
    rospy.spin()

