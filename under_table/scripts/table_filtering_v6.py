#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho
   Function :  (1) Identify the Table within a detected Region (Rectangular)
               (2) Plan a route to go under the Table and publish the route
"""

import rospy
import numpy as np
import tf
import utils

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PointStamped, Quaternion, PolygonStamped, Vector3, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker
from utils import LineTrajectory
from tf import TransformListener


class TableFilteringV6():
    def __init__(self):
        # Define Adjustable Parameters
        # - Scan Region (Rectangular)
        self.x_min = float(rospy.get_param("~x_min"))    # [m]
        self.x_max = float(rospy.get_param("~x_max"))    # [m]
        self.y_min = float(rospy.get_param("~y_min"))    # [m]
        self.y_max = float(rospy.get_param("~y_max"))    # [m]
        # - Table Size
        self.table_width = float(rospy.get_param("~table_width"))           # [m]
        self.table_length = float(rospy.get_param("~table_length"))         # [m]
        self.table_diagonal = float(rospy.get_param("~table_diagonal"))     # [m]
        self.table_tolerance = float(rospy.get_param("~table_tolerance"))   # [m]
        # - Route Trim
        self.route_trim_x = rospy.get_param("~route_trim_x")   # [m]
        self.route_trim_y = rospy.get_param("~route_trim_y")   # [m]

        # Internal Use Variables - Do not modify without consultation
        self.init = False      # Need to be "False" when integrated with others
        self.cluster_tolerance = 0.1    # [m]
        self.table = False
        self.trajectory = LineTrajectory("/table_trajectory")
        self.tfTL = TransformListener()
        self.pathpoint_1 = None
        self.pathpoint_2 = None
        self.refresh_rate = rospy.Rate(10)
        self.stop_counter = 0
        self.centerpoint = None

        # Subscribers
        self.scan_sub = rospy.Subscriber(rospy.get_param("~scan_topic"), LaserScan, self.scanCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(rospy.get_param("~pose_topic"), Odometry, self.poseCB, queue_size=1)
        self.init_table_sub = rospy.Subscriber(rospy.get_param("~init_table_topic"), Bool, self.init_tableCB, queue_size=1)

        # Publishers
        self.route_pub = rospy.Publisher(rospy.get_param("~route_topic"), PolygonStamped, queue_size=1)
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.finish_table_pub = rospy.Publisher(rospy.get_param("~finish_table_topic"), Bool, queue_size=1)
        self.viz_table_pub = rospy.Publisher(rospy.get_param("~viz_table_topic"), Marker, queue_size=1)
        self.viz_points_pub = rospy.Publisher(rospy.get_param("~viz_points_topic"), Marker, queue_size=1)

        # Publish Route for Navigating Under Table
        self.route_publisher()

    # Check when to start the Table-Filtering
    def init_tableCB(self, msg):
        self.init = msg.data
        if(msg.data != True):
            self.table = False

    # Receive every LaserScan and do Filtering to find out Table
    def scanCB(self, msg):
        if(self.init == True):
            maybe_pts = []
            clusters = []
            mean_pts = []
            precheck_pts = []
            checked_pts = []
            table_pts = []
            # Finding Points within selected Scan_Region (Rectangular)
            for i in xrange(len(msg.ranges)):
                pt = Point()
                pt.x = msg.ranges[i]*np.cos(msg.angle_min + msg.angle_increment*i)
                pt.y = msg.ranges[i]*np.sin(msg.angle_min + msg.angle_increment*i)
                pt.z = 0.0
                if(self.x_min <= pt.x <= self.x_max and self.y_min <= pt.y <= self.y_max):
                    maybe_pts.append(pt)
            # Filter the maybe_pts into clusters of points
            for j in range(0,10):
                cluster_counter = 0
                temp_cluster = []
                if(len(maybe_pts) == 0):
                    break
                for i in xrange(len(maybe_pts)):
                    if(self.dist(maybe_pts[i-1], maybe_pts[i]) <= self.cluster_tolerance):
                        temp_cluster.append(maybe_pts[i-1])
                    if(self.dist(maybe_pts[i-1], maybe_pts[i]) > self.cluster_tolerance):
                        cluster_counter = cluster_counter + 1
                    if(cluster_counter == 2):
                        temp_cluster.append(maybe_pts[i-1])
                        break
                clusters.append(temp_cluster)    # Save each clusters that meet the requirement
                for i in xrange(len(temp_cluster)):
                    del maybe_pts[0]
            # Filter and Find the Mean Point of survived clusters (possible as table legs)
            for i in xrange(len(clusters)):
                if(len(clusters[i]) > 5):
                    mean_pts.append(self.mean_point(clusters[i]))
            # Visualize the detected Clusters
            for i in xrange(len(mean_pts)):
                self.viz_point(i, mean_pts[i], ColorRGBA(1,0,0,1))
            # Filter the Possibilities and Find out which Sets of Clusters is equivalent as Table
            for j in xrange(len(mean_pts)):
                correct_counter = 0
                for i in xrange(len(mean_pts)):
                    if(self.check_length_width_diagonal(mean_pts[j], mean_pts[i])):
                        correct_counter = correct_counter + 1
                    if(correct_counter == 3):
                        precheck_pts.append(mean_pts[j])
                        break
            checked_pts = self.remove_duplicates(precheck_pts)
            # Visualize the Table
            print len(checked_pts)
            if(len(checked_pts) == 4):
                for i in xrange(len(checked_pts)):
                    table_pts.append(checked_pts[i])
                table_pts.append(checked_pts[0])
                self.viz_table(table_pts)
                self.table = True
                self.pathpoint_1 = self.middle_point(checked_pts[0], checked_pts[3])
                self.pathpoint_2 = self.middle_point(checked_pts[1], checked_pts[2])
            else:
                self.pathpoint_1 = self.pathpoint_1
                self.pathpoint_2 = self.pathpoint_2

    # Checking Position (Map Frame) for when to stop
    def poseCB(self, msg):
        if(self.init == True):
            current_pose = msg.pose.pose.position
            current_yaw = utils.quaternion_to_angle(msg.pose.pose.orientation)
            if(self.centerpoint != None):
#                print "current_pose:  " + str(current_pose)
#                print "centerpoint :  " + str(self.centerpoint)
                print self.dist(current_pose, self.centerpoint)
                if(self.dist(current_pose, self.centerpoint) <= 0.17):   # 0.141 / 0.145 / 0.17 / 0.28 / 0.31
                    self.stopping()
                    self.stop_counter = self.stop_counter + 1
                if(self.stop_counter == 30):
                    self.finish_table_pub.publish(True)
                    self.stop_counter = 0
                    print "----------------------------- reporting finish --------------------------------------"         # Testing USE - find hidden bug

    # Keep on Publishing the Latest Route for the Table-Navi
    def route_publisher(self):
        while not rospy.is_shutdown():
            if(self.tfTL.frameExists("base_link")):
                if(self.init == True and self.pathpoint_1 != None and self.pathpoint_2 != None and self.table == True):
                    self.trajectory.clear()
                    # Transform the table mid-pts (base_link) into (map)
                    traj_pt_1 = self.transforming_point(self.pathpoint_1, "/map")
                    traj_pt_1.x += self.route_trim_x
                    traj_pt_1.y += self.route_trim_y
                    traj_pt_2 = self.transforming_point(self.pathpoint_2, "/map")
                    traj_pt_2.x += self.route_trim_x
                    traj_pt_2.y += self.route_trim_y
                    # Line Extension(2m before the table)
                    traj_pt_0 = Point()
                    traj_pt_0.x = traj_pt_1.x + (2.0*(traj_pt_1.x - traj_pt_2.x)/self.dist(traj_pt_1, traj_pt_2)) + self.route_trim_x
                    traj_pt_0.y = traj_pt_1.y + (2.0*(traj_pt_1.y - traj_pt_2.y)/self.dist(traj_pt_1, traj_pt_2)) + self.route_trim_y
                    traj_pt_0.z = 0.0
                    # Line Extension (0.3m after the table)
                    traj_pt_3 = Point()
                    traj_pt_3.x = traj_pt_2.x - (0.3*(traj_pt_1.x - traj_pt_2.x)/self.dist(traj_pt_1, traj_pt_2)) + self.route_trim_x
                    traj_pt_3.y = traj_pt_2.y - (0.3*(traj_pt_1.y - traj_pt_2.y)/self.dist(traj_pt_1, traj_pt_2)) + self.route_trim_y
                    traj_pt_3.z = 0.0
                    self.trajectory.addPoint(traj_pt_0)
                    self.trajectory.addPoint(traj_pt_1)
                    self.trajectory.addPoint(traj_pt_2)
                    self.trajectory.addPoint(traj_pt_3)
                    self.route_pub.publish(self.trajectory.toPolygon())
                    self.centerpoint = self.middle_point(traj_pt_1, traj_pt_2)
                    # Visualize the Path Points
                    self.viz_point(100, traj_pt_0, ColorRGBA(0,0,1,1))
                    self.viz_point(101, traj_pt_1, ColorRGBA(0,0,1,1))
                    self.viz_point(102, traj_pt_2, ColorRGBA(0,0,1,1))
                    self.viz_point(103, traj_pt_3, ColorRGBA(0,0,1,1))
                else:
                    self.centerpoint = self.centerpoint
            self.refresh_rate.sleep()

    # Distance Calculator
    def dist(self, a, b):
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        return distance

    # Mean Point Calculator for each Cluster
    def mean_point(self, point_set):
        mean_pt = Point()
        for i in xrange(len(point_set)):
            mean_pt.x = mean_pt.x + point_set[i].x    # x-position
            mean_pt.y = mean_pt.y + point_set[i].y    # y-position
        mean_pt.x = mean_pt.x/len(point_set)
        mean_pt.y = mean_pt.y/len(point_set)
        mean_pt.z = 0.0
        return mean_pt

    # a & b Middle Point Calculator
    def middle_point(self, a, b):
        middle_point = Point((a.x + b.x)/2.0, (a.y + b.y)/2.0, 0.0)
        return middle_point

    # Table Length or Width Checker
    def check_length_width_diagonal(self, a, b):
        if(0.0001<abs(self.dist(a,b) - self.table_length)<=self.table_tolerance or 0.0001<abs(self.dist(a,b) - self.table_width)<=self.table_tolerance or 0.0001<abs(self.dist(a,b) - self.table_diagonal)<self.table_tolerance):
            return True
        else:
            return False

    # Duplicates Remover
    def remove_duplicates(self, values):
        output = []
        seen = set()
        for value in values:
            if value not in seen:
                output.append(value)
                seen.add(value)
        return output

    # Transform Point from coordinate frame (base_link) to (map)
    def transforming_point(self, pre_pos, frame):
        pre_transform_pt = PointStamped()
        pre_transform_pt.header.frame_id = "base_link"
        pre_transform_pt.header.stamp = rospy.Time(0)
        pre_transform_pt.point = pre_pos
        transformed_pt = self.tfTL.transformPoint(frame, pre_transform_pt)
        after_transform_pt = Point(transformed_pt.point.x, transformed_pt.point.y, 0.0)
        return after_transform_pt

    # Use for Visualizing the Points
    def viz_point(self, sid, pt_position, color):
        point = Marker()
        point.header.frame_id = "base_link"
        point.header.stamp = rospy.Time.now()
        point.ns = "points"
        point.id = sid
        point.type = 1    # CUBE
        point.action = 0    # add/modify
        point.lifetime = rospy.Duration(0.1)
        point.scale = Vector3(0.1,0.1,0.1)
        point.color = color
        point.pose.orientation = Quaternion(0,0,0,1)
        point.pose.position = pt_position
        self.viz_points_pub.publish(point)

    # Use for Visualizing the Table
    def viz_table(self, table_points):
        table = Marker()
        table.header.frame_id = "base_link"
        table.header.stamp = rospy.Time.now()
        table.ns = "table"
        table.id = 0
        table.type = 4    # LINE STRIP
        table.action = 0    # add/modify
        table.lifetime = rospy.Duration(0.1)
        table.scale = Vector3(0.05,0,0)
        table.color = ColorRGBA(1,1,0,1)    # Yellow
        table.pose.orientation = Quaternion(0,0,0,1)
        for p in table_points:
            t_point = Point(p.x, p.y, 0.0)
            table.points.append(t_point)
        self.viz_table_pub.publish(table)

    # Stopping the vehicle
    def stopping(self):
        stop_cmd = Twist()
        stop_cmd.linear = Vector3(0, 0, 0)
        stop_cmd.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(stop_cmd)



if __name__=="__main__":
    rospy.init_node("table_filtering_v6")
    TableFilteringV6()
    rospy.spin()
