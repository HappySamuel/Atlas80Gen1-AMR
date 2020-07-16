#!/usr/bin/env python

'''
Author :  Samuel Chieng Kien Ho
Functions :  (1) Stopping Vehicle when obstacles are detected until they are cleared
             (2) Increase detecting region (front and rear) when moving faster than certain speed
             (3) Stop Vehicle if lidar is detected failed (timestamp not changing for 2 Hz)
             (4) Filtering the noise of the receiving data
Feature :  Self-defining the detection area (polygon), no longer using rectangular type

      Mode 0                 Mode 1                                       Mode 2

                          ------------                              ------------------
                         ' /   /   /  '                            ' /   /   /   /   /'
 Obstacles Detection     '/   /   /   '   Obstacles outside        '/   / ------    / '   Obstacles within
 is Closed Down          '   /   /   /'   Outer Region             '   / '      '  /  '   Inner Region
                         '  /   /   / '   are not included         '  /  '------' /   '   are not included
                         ' /   /   /  '                            ' /   /   /   /   /'
                         '------------'                            '------------------'
'''
import rospy
import numpy as np
import yaml

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Point, PolygonStamped
from std_msgs.msg import String
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon


class ObstacleStopV9():
    def __init__(self):
        # Define Adjustable Parameters
        self.file_location = rospy.get_param("~file_location")
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        # Internal USE Variables - Do not modify without consultation
        self.mode = 1
        self.front_increment = 0.0    # [m]
        self.new_timestamp = 0
        self.last_timestamp = 1
        self.region_file = self.file_location +"/detection_region.yaml"
        self.outer_zone = ShapelyPolygon()
        self.inner_zone = ShapelyPolygon()
        self.checker_rate = rospy.Rate(5)
        self.cluster_tolerance = 0.05   # [m]
        self.counter = 0
        self.speed_type = 0

        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.viz_inner_region_pub = rospy.Publisher(rospy.get_param("~viz_inner_region_topic"), PolygonStamped, queue_size=1)
        self.viz_outer_region_pub = rospy.Publisher(rospy.get_param("~viz_outer_region_topic"), PolygonStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("obstacle_stop/debug", String, queue_size=1)

        # Subscribers
        self.obstacle_mode_sub = rospy.Subscriber(rospy.get_param("~obstacle_mode_topic"), String, self.obstacle_modeCB, queue_size=1)
        self.speed_sub = rospy.Subscriber(rospy.get_param("~speed_topic"), Twist, self.speedCB, queue_size=1)
        self.scan_sub = rospy.Subscriber(rospy.get_param("~scan_topic"), LaserScan, self.scanCB, queue_size=1)

        # Lidar Health Checker Loop
        self.lidar_health_checker()

    # Checking on which mode should be used
    def obstacle_modeCB(self, msg):
        if(int(msg.data) == 0):
            self.mode = 0
        elif(int(msg.data) == 1):
            self.mode = 1
        elif(int(msg.data) == 2):
            self.mode = 2

    # Checking on vehicle's speed reading
    def speedCB(self, msg):
        if(msg.linear.x > self.speed_limit or msg.linear.x < -self.speed_limit):
            self.speed_type = 1    # fast
        else:
            self.speed_type = 0    # slow

    # Checking LaserScan whether any obstacle lied within the predefined region
    def scanCB(self, msg):
        overall_pts = []
        inner_pts = []
        remaining_pts = []
        overall_clusters = []
        remaining_clusters = []
        self.new_timestamp = msg.header.stamp
        for i in xrange(len(msg.ranges)):
            pt = Point()
            pt.x = msg.ranges[i]*np.cos(msg.angle_min + i*msg.angle_increment)
            pt.y = msg.ranges[i]*np.sin(msg.angle_min + i*msg.angle_increment)
            shapely_point = ShapelyPoint(pt.x, pt.y)
            if(self.outer_zone.contains(shapely_point)):
                overall_pts.append(pt)
            if(self.inner_zone.contains(shapely_point)):
                inner_pts.append(pt)
        if(self.mode == 0):
            print "Mode 0 ---> Obstacle Stop Function Closed"
            self.debug_pub.publish("Mode 0 ---> Obstacle Stop Function Closed")
        elif(self.mode == 1):
            #print "Mode 1 ---> Without Table on Top"
            if(overall_pts != []):
                overall_clusters = self.cluster_check(overall_pts)
                if(overall_clusters != []):
                    self.stopping()
                    print "Mode 1 ---> Obstacle Detected and Stopping"
                    self.debug_pub.publish("Mode 1 ---> Obstacle Detected and Stopping")
                else:
                    self.debug_pub.publish("Mode 1 ---> No Obstacles")
        elif(self.mode == 2):
            #print "Mode 2 ---> Carry Table on Top"
            remaining_pts = list(set(overall_pts) - set(inner_pts))
            if(remaining_pts != []):
                remaining_clusters = self.cluster_check(remaining_pts)
                if(remaining_clusters != []):
                    self.stopping()
                    print "Mode 2 ---> Obstacle Detected and Stopping"
                    self.debug_pub.publish("Mode 2 ---> Obstacle Detected and Stopping")
                else:
                    self.debug_pub.publish("Mode 2 ---> No Obstacles")
        self.load_params_from_yaml(self.region_file)

    # Checking on Lidar Health
    def lidar_health_checker(self):
        while not rospy.is_shutdown():
            if(self.new_timestamp == self.last_timestamp):
                if(self.counter >= 3):
                    self.stopping()
                    print "Lidars malfunction......."
                    self.debug_pub.publish("Lidars malfunction.......")
                self.counter += 1
            else:
                self.counter = 0
            self.last_timestamp = self.new_timestamp
            self.checker_rate.sleep()

    # Filterning Noise by clustering detected points
    def cluster_check(self, pts):
        cluster = []
        for i in xrange(len(pts)):
            if(self.dist_checker(pts[i-1], pts[i], self.cluster_tolerance)):
                cluster.append(pts[i-1])
        if(len(cluster) <= 1):
            cluster = []
        return cluster

    # Checking distance between 2 points, whether within tolerance or not
    def dist_checker(self, a, b, tolerance):
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        if(distance <= tolerance):
            return True
        else:
            return False

    # Loading parameters from yaml file
    def load_params_from_yaml(self, filepath):
        inner_region = []
        fast_outer_region = []
        slow_outer_region = []
        outer_region = []
        with open(filepath, 'r') as infile:
            data = yaml.load(infile)
            for p in data["outer_region"]["fast"]["points"]:
                fast_outer_region.append((p["x"], p["y"]))
            for p in data["outer_region"]["slow"]["points"]:
                slow_outer_region.append((p["x"], p["y"]))
            if(self.speed_type == 1):
                self.outer_zone = ShapelyPolygon(fast_outer_region)
                outer_region = fast_outer_region
            else:
                self.outer_zone = ShapelyPolygon(slow_outer_region)
                outer_region = slow_outer_region
            for p in data["inner_region"]["points"]:
                inner_region.append((p["x"], p["y"]))
            self.inner_zone = ShapelyPolygon(inner_region)
            self.visualize_region(outer_region, inner_region)

    # Visualize the Predefined Detection Region
    def visualize_region(self, pts_A, pts_B):
        poly_A = PolygonStamped()
        poly_A.header.frame_id = "/base_link"
        poly_A.header.stamp = rospy.Time.now()
        for i in xrange(len(pts_A)):
            pt = Point()
            pt.x = pts_A[i][0]
            pt.y = pts_A[i][1]
            pt.z = 0.0
            poly_A.polygon.points.append(pt)
        poly_B = PolygonStamped()
        poly_B.header.frame_id = "/base_link"
        poly_B.header.stamp = rospy.Time.now()
        for i in xrange(len(pts_B)):
            pt = Point()
            pt.x = pts_B[i][0]
            pt.y = pts_B[i][1]
            pt.z = 0.0
            poly_B.polygon.points.append(pt)
        if(self.mode == 0):
            poly_A.polygon.points = []
            poly_B.polygon.points = []
        elif(self.mode == 1):
            poly_B.polygon.points = []
        self.viz_outer_region_pub.publish(poly_A)
        self.viz_inner_region_pub.publish(poly_B)

    # Stopping the vehicle
    def stopping(self):
        brake_cmd = Twist()
        brake_cmd.linear = Vector3(0, 0, 0)
        brake_cmd.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(brake_cmd)



if __name__=="__main__":
    rospy.init_node("obstacle_stop_v9")
    ObstacleStopV9()
    rospy.spin()

