#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho
   Functions :  (1) Stopping Vehicle when obstacles are detected until they are cleared
                (2) Increase detecting region (front and rear) when moving faster than certain speed
                (3) Stop Vehicle if lidar is detected failed (timestamp not changing for 2 Hz)
                (4) Filtering the noise of the receiving data

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


class ObstacleStopV7():
    def __init__(self):
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
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        # Internal Use Variables - Do not modify without consultation
        self.mode = 1
        self.addon_x = 0.0   # [m]
        self.new_timestamp = 0
        self.last_timestamp = 1
        self.checker_rate = rospy.Rate(10)   # [Hz]
        self.scan_msg = LaserScan()
        self.cluster_tolerance = 0.05   # [m]
        self.counter = 0

        # Subscribers
        self.obstacle_mode_sub = rospy.Subscriber(rospy.get_param("~obstacle_mode_topic"), String, self.obstacle_modeCB, queue_size=1)
        self.speed_sub = rospy.Subscriber(rospy.get_param("~speed_topic"), Twist, self.speedCB, queue_size=1)
        self.scan_sub = rospy.Subscriber(rospy.get_param("~scan_topic"), LaserScan, self.scanCB, queue_size=1)

        # Publisher
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)

        # Main Loop
        self.lidar_failure_checker()

    # Checking on which mode being using
    def obstacle_modeCB(self, msg):
        if(int(msg.data) == 0):
            self.mode = 0
        elif(int(msg.data) == 1):
            self.mode = 1
        elif(int(msg.data) == 2):
            self.mode = 2

    # Checking on vehicle speed command
    def speedCB(self, msg):
        if(msg.linear.x > self.speed_limit or msg.linear.x < -self.speed_limit):
            self.addon_x = 0.5
        else:
            self.addon_x = 0.0

    # Checking LaserScan whether any obstacle lied within the scan region
    def scanCB(self, msg):
        self.scan_msg = msg
        self.new_timestamp = msg.header.stamp

    def lidar_failure_checker(self):
        while not rospy.is_shutdown():
            overall_pts = []     # Obstacles detected within outer region
            inner_pts = []       # Obstacles detected within inner region
            remaining_pts = []
            overall_clusters = []
            remaining_clusters = []
            for i in xrange(len(self.scan_msg.ranges)):
                pt = Point()
                pt.x = self.scan_msg.ranges[i]*np.cos(self.scan_msg.angle_min + self.scan_msg.angle_increment*i)
                pt.y = self.scan_msg.ranges[i]*np.sin(self.scan_msg.angle_min + self.scan_msg.angle_increment*i)
                pt.z = 0.0
                if((self.outer_x_min - self.addon_x) <= pt.x <= (self.outer_x_max + self.addon_x) and self.outer_y_min <= pt.y <= self.outer_y_max):
                    overall_pts.append(pt)
                if(self.inner_x_min <= pt.x <= self.inner_x_max and self.inner_y_min <= pt.y <= self.inner_y_max):
                    inner_pts.append(pt)
            if(self.mode == 0):
                print "Mode 0 ---> Obstacle Stop Function Closed"
            elif(self.mode == 1):
#                print "Mode 1 ---> Without Table on Top"
                if(overall_pts != []):
                    overall_clusters = self.cluster_check(overall_pts)
                    if(overall_clusters != []):
                        self.stopping()
            elif(self.mode == 2):
#                print "Mode 2 ---> Carry Table on Top"
                remaining_pts = list(set(overall_pts) - set(inner_pts))
                if(remaining_pts != []):
                    remaining_cluster = self.cluster_check(remaining_pts)
                    if(remaining_cluster != []):
                        self.stopping()
#            print "new_timestamp :  " + str(self.new_timestamp)
#            print "last_timestamp :  " + str(self.last_timestamp)
            print "--------------------------------------------------"
            if(self.new_timestamp == self.last_timestamp):
                if(self.counter >= 5):
                    self.stopping()
                    print "Lidars malfunction......."
                self.counter = self.counter + 1
            else:
                self.counter = 0
            self.last_timestamp = self.scan_msg.header.stamp
            self.checker_rate.sleep()

    # Clustering the detected points and filter out the noise (only 1~2 pts exists)
    def cluster_check(self, pts):
        cluster = []
        for i in xrange(len(pts)):
            if(self.dist_checker(pts[i-1], pts[i], self.cluster_tolerance)):
                cluster.append(pts[i-1])
        print "cluster :  " + str(len(cluster))
        if(len(cluster) <= 1):
            cluster = []
        return cluster

    # Checking distance [m] between two points, whether within tolerance or not
    def dist_checker(self, a, b, tolerance):
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        if(distance <= tolerance):
            return True
        else:
            return False

    # Stopping the vehicle
    def stopping(self):
        print "Stopping............................."
        brake_cmd = Twist()
        brake_cmd.linear = Vector3(0, 0, 0)
        brake_cmd.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(brake_cmd)



if __name__=="__main__":
    rospy.init_node("obstacle_stop_v7")
    ObstacleStopV7()
    rospy.spin()

