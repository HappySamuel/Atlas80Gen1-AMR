#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho
   Function :  Stopping Vehicle when obstacles are detected until they are cleared

       Mode 1                                       Mode 2

    ------------                              ------------------
   ' /   /   /  '                            ' /   /   /   /   /'
   '/   /   /   '   Obstacles outside        '/   / ------    / '   Obstacles within
   '   /   /   /'   Outer Region             '   / '      '  /  '   Inner Region
   '  /   /   / '   are not included         '  /  '------' /   '   are not included
   ' /   /   /  '                            ' /   /   /   /   /'
   '------------'                            '------------------'

"""
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import String

class ObstacleStopV4():
    def __init__(self):
        # Define Topics for publishing and subscribing messages
        self.drive_topic = rospy.get_param("~drive_topic")
        self.scan_topic = rospy.get_param("~scan_topic")
        self.obstacle_mode_topic = rospy.get_param("~obstacle_mode_topic")

        # Define Adjustable Parameters
        # - Scan Region (inner rectangular vs outer rectangular)
        self.inner_x_min = float(rospy.get_param("~inner_x_min"))
        self.inner_x_max = float(rospy.get_param("~inner_x_max"))
        self.inner_y_min = float(rospy.get_param("~inner_y_min"))
        self.inner_y_max = float(rospy.get_param("~inner_y_max"))
        self.outer_x_min = float(rospy.get_param("~outer_x_min"))
        self.outer_x_max = float(rospy.get_param("~outer_x_max"))
        self.outer_y_min = float(rospy.get_param("~outer_y_min"))
        self.outer_y_max = float(rospy.get_param("~outer_y_max"))

        # Internal Use Variables - Do not modify without consultation
        self.mode = 1

        # Subscribers
        self.obstacle_mode_sub = rospy.Subscriber(self.obstacle_mode_topic, String, self.obstacle_modeCB, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scanCB, queue_size=1)

        # Publisher
        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)

    # Checking on which mode being using
    def obstacle_modeCB(self, msg):
        if(int(msg.data) == 0):
            self.mode = 0
        elif(int(msg.data) == 1):
            self.mode = 1
        elif(int(msg.data) == 2):
            self.mode = 2

    # Checking LaserScan whether any obstacle lied within the scan region
    def scanCB(self, msg):
        overall_pts = []     # Obstacles detected within outer region
        inner_pts = []       # Obstacles detected within inner region
        remaining_pts = []
        for i in xrange(len(msg.ranges)):
            pt = Point()
            pt.x = msg.ranges[i]*np.cos(msg.angle_min + msg.angle_increment*i)
            pt.y = msg.ranges[i]*np.sin(msg.angle_min + msg.angle_increment*i)
            pt.z = 0.0
            if(self.outer_x_min <= pt.x <= self.outer_x_max and self.outer_y_min <= pt.y <= self.outer_y_max):
                overall_pts.append(pt)
            if(self.inner_x_min <= pt.x <= self.inner_x_max and self.inner_y_min <= pt.y <= self.inner_y_max):
                inner_pts.append(pt)
        if(self.mode == 0):
            print "Mode 0 ---> Obstacle Stop Function Closed"
        elif(self.mode == 1):
            print "Mode 1 ---> Without Table on Top"
            if(overall_pts != []):
                self.stopping()
        elif(self.mode == 2):
            print "Mode 2 ---> Carry Table on Top"
            remaining_pts = list(set(overall_pts) - set(inner_pts))
            if(remaining_pts != []):
                self.stopping()

    # Stopping the vehicle
    def stopping(self):
        print "Stopping............................."
        brake_cmd = Twist()
        brake_cmd.linear = Vector3(0, 0, 0)
        brake_cmd.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(brake_cmd)



if __name__=="__main__":
    rospy.init_node("obstacle_stop_v4")
    ObstacleStopV4()
    rospy.spin()
