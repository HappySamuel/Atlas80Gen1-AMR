#!/usr/bin/env python

'''
Author: Samuel Chieng Kien Ho
Reference: Corey Walsh (MIT)

Main Purpose:
    - Generate a speed profile for a given trajectory
    - Visualize the generated speed profile with colorful display

Limitations:
    - The speed profile generated are sub-optimal solutions
'''
import rospy
import numpy as np
import time
import utils

from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PolygonStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from utils import LineTrajectory


class TrajectoryWithSpeedLoader():
    def __init__(self):
        # Define Adjustaable Parameters
        self.min_speed         = rospy.get_param("~min_speed")
        self.max_speed         = rospy.get_param("~max_speed")
        self.max_acceleration  = rospy.get_param("~max_acceleration")
        self.max_decceleration = rospy.get_param("~max_decceleration")

        # Internal USE Variables - Do not modify without consultation
        self.dynamic_model = utils.ApproximateDynamicsModel()
        self.ackermann_model = utils.AckermannModel(0.36)
        self.trajectory = LineTrajectory("/speed_track")
        self.counts = 0

        # Publishers
        self.viz_track_pub = rospy.Publisher("/speed_track/viz", MarkerArray, queue_size=1)
        self.traj_pub = rospy.Publisher(rospy.get_param("~pub_topic"), PolygonStamped, queue_size=1)

        # Subscriber
        self.traj_sub = rospy.Subscriber(rospy.get_param("~sub_topic"), String, self.trajCB, queue_size=1)

        print "Initialized. Waiting on messages..."
        # need to wait a short period of time before publishing  the first message
        time.sleep(0.5)

    # Callback Function for receiving path
    def trajCB(self, msg):
        path_id = msg.data
        if(self.counts == 1):
            self.generate_speed_profile(path_id)
            self.counts = 0
        self.counts += 1

    # Generating Speed Profile
    def generate_speed_profile(self, path):
        self.trajectory.clear()
        self.trajectory.load(path)
        self.trajectory.make_np_array()
        points = self.trajectory.np_points
        local_polar_points = utils.piecewise_linear_local_waypoints_polar(points)
        speeds = np.zeros(points.shape[0])
        for i in xrange(local_polar_points.shape[0]):
            max_speed = self.dynamic_model.max_speed_dist(local_polar_points[i,0])
            speeds[i] = max_speed
        speeds[0] = 0	# speed of the beginning of the track
        speeds[-1] = 0	# speed of the last of the track
        speeds = np.clip(speeds, self.min_speed, self.max_speed)
        # ensure that acceleration bounds are respected
        for i in xrange(speeds.shape[0]-1):
            x = local_polar_points[i,0]
            v_i = speeds[i]	# initial speed of a straight line
            v_f = speeds[i+1]	# final speed of a straight line
            a = (v_f**2 - v_i**2)/(2.0*x)	# vf^2 - vi^2 = 2ax
            if a > self.max_acceleration:
                a = self.max_acceleration
                v_f = np.sqrt(v_i**2 + 2.0*a*x)
                speeds[i+1] = v_f
        # ensure that decceleration bounds are respected
        for i in xrange(speeds.shape[0]-2, -1, -1):
            x = local_polar_points[i,0]
            v_i = speeds[i]
            v_f = speeds[i+1]
            a = (v_f**2 - v_i**2)/(2.0*x)	# vf^2 - vi^2 = 2ax
            if a < self.max_decceleration:
                a = self.max_decceleration
                v_i = np.sqrt(v_f**2 - 2.0*a*x)
                speeds[i] = v_i
        # counting estimated travesal time
        t_traversal = 0.0
        for i in xrange(speeds.shape[0]-1):
            v, x = speeds[i], local_polar_points[i,0]
            t = x/v
            t_traversal += t
        print "Estimated traversal time:    ", t_traversal
        print speeds
        # Publish the generated Speed Profile
        self.trajectory.speed_profile = speeds.tolist()
        self.viz_track_pub.publish(viz_speed_track(self.trajectory.speed_profile, self.trajectory.np_points))
        self.traj_pub.publish(self.trajectory.toPolygon())

# Take current point and next point to form a line strip to be visualized
def track_marker(point1, point2, index=0, color=ColorRGBA(0, 0, 0, 0.5)):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "speed_track"
    marker.id = index
    marker.type = 4		# line strip
    marker.lifetime = rospy.Duration.from_sec(1)
    marker.action = 0
    marker.scale.x = 0.3
    marker.color = color
    pt1 = Point(point1[0], point1[1], 0.0)
    pt2 = Point(point2[0], point2[1], 0.0)
    marker.points.append(pt1)
    marker.points.append(pt2)
    return marker

# Visualize the generated speed track with color
def viz_speed_track(traj_speed_profile, pts):
    markers = []
    viz_speed = np.array(traj_speed_profile)
    for i in range(0,viz_speed.shape[0]-1):
        speed = viz_speed[i]
        if 0.5 <= speed < 0.6:
            color = ColorRGBA(0, 1, 0, 0.6)     #green
        elif 0.6 <= speed < 0.7:
            color = ColorRGBA(1, 1, 0, 0.6)     #yellow
        elif 0.7 <= speed < 0.8:	
            color = ColorRGBA(1, 0.5, 0, 0.6)   #orange
        elif 0.8 <= speed < 0.9:
            color = ColorRGBA(1, 0, 0, 0.6)     #red
        elif 0.9 <= speed < 1.0:
            color = ColorRGBA(0.5, 0, 1, 0.6)   #purple
        elif speed >= 1.0:
            color = ColorRGBA(0, 0.5, 1.0, 0.6)	#sea-blue
        else:	# speed < 0.5
            color = ColorRGBA(1, 1, 1, 0.6)     #white
        # Call "track_marker" when every 2 points are given and save into "markers" to become array
        markers.append(track_marker(pts[i,:], pts[i+1,:], index=i, color=color))
        track_array = MarkerArray(markers=markers)
    return track_array



if __name__=="__main__":
    rospy.init_node("trajectory_with_speed_loader")
    TrajectoryWithSpeedLoader()
    rospy.spin()

