#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PolygonStamped, Twist, Point, Vector3, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class ChargingNaviV2():
    def __init__(self):
        # Define Adjustable Parameters
        self.lookahead = float(rospy.get_param("~lookahead"))
        self.max_reacquire = float(rospy.get_param("~max_reacquire"))
        self.KV = float(rospy.get_param("~KV"))
        self.KW = float(rospy.get_param("~KW"))

        # Internal Use Variables - Do not modify without consultation
        self.trajectory = utils.LineTrajectory("/charging_followed")
        self.do_viz = True
        self.d_t = 0.1
        self.nearest_pt = None
        self.lookahead_pt = None
        self.init = False
        self.stop_charging = False

        # Subscribers
        self.init_charging_sub = rospy.Subscriber(rospy.get_param("~init_charging_topic"), Bool, self.init_chargingCB, queue_size=1)
        self.stop_charging_sub = rospy.Subscriber(rospy.get_param("~stop_charging_topic"), Bool, self.stop_chargingCB, queue_size=1)
        self.traj_sub = rospy.Subscriber("/charging_navi/trajectory", PolygonStamped, self.trajCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(rospy.get_param("~pose_topic"), Odometry, self.poseCB, queue_size=1)
        self.amcl_sub = rospy.Subscriber(rospy.get_param("~amcl_topic"), PoseWithCovarianceStamped, self.poseCB, queue_size=1)

        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.nearest_pt_pub = rospy.Publisher("/charging_navi/nearest_pt", Marker, queue_size=1)
        self.lookahead_pt_pub = rospy.Publisher("/charging_navi/lookahead_pt", Marker, queue_size=1)

    def visualize(self):
        if not self.do_viz:
            return
        if self.nearest_pt_pub.get_num_connections()>0 and isinstance(self.nearest_pt, np.ndarray):
            self.nearest_pt_pub.publish(utils.make_circle_marker(self.nearest_pt, 0.5, [0, 0, 1], "/map", "/charging_navi", 0, 3))
        if self.lookahead_pt_pub.get_num_connections()>0 and isinstance(self.lookahead_pt, np.ndarray):
            self.lookahead_pt_pub.publish(utils.make_circle_marker(self.lookahead_pt, 0.5, [1, 1, 1], "/map", "/charging_navi", 1, 3))

    def init_chargingCB(self, msg):
        self.init = msg.data

    def stop_chargingCB(self, msg):
        self.stop_charging = msg.data

    def trajCB(self, msg):
        self.trajectory.clear()
        self.trajectory.fromPolygon(msg.polygon)
        self.trajectory.publish_viz(duration=0.0)

    def poseCB(self, msg):
        if isinstance(msg, Odometry):
            pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, utils.quaternion_to_angle(msg.pose.pose.orientation)])
        elif isinstance(msg, PoseWithCovarianceStamped):
            pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, utils.quaternion_to_angle(msg.pose.pose.orientation)])
        if(self.init == True and self.stop_charging == False):
            self.pure_pursuit(pose)

    def pure_pursuit(self, pose):
        # Stop if no trajectory has been received
        if(self.trajectory.empty()):
            return self.stop()
        # Convert trajectory's information (list of waypoints) into numpy matrix
        if(self.trajectory.dirty()):
            self.trajectory.make_np_array()
        # Step 1
        self.nearest_pt, nearest_dist, t, i = utils.nearest_point_on_trajectory(pose[:2], self.trajectory.np_points)
        if(nearest_dist < self.lookahead):
            # Step 2
            self.lookahead_pt, i2, t2 = utils.first_point_on_trajectory_intersecting_circle(pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=0)
            if(i2 == None):
                self.lookahead_pt = None
        elif(nearest_dist < self.max_reacquire):
            self.lookahead_pt = self.nearest_pt
        else:
            self.lookahead_pt = None
        # Stop vehicle if there is no navigation target
        if not isinstance(self.lookahead_pt, np.ndarray):
            self.stop()
        # Compute driving command based on current pose and lookahead point
        else:
            xspeed, rotspeed = self.determine_speeds(pose, self.lookahead_pt)
            self.apply_control(xspeed, rotspeed)
        self.visualize()

    def determine_speeds(self, pose, lookahead_point):
	'''Given a robot pose, and a lookahead point, determine the open loop control 
	   necessary to navigate to that lookahead point. Use Differential Robot steering
	   geometry. '''
	# get the lookahead point in the coordinate frame of the car
	delta_x = lookahead_point[0] - pose[0]
	delta_y = lookahead_point[1] - pose[1]
	w_v = 2*(delta_y*np.cos(pose[2]) - delta_x*np.sin(pose[2]))/(delta_x*delta_x+delta_y*delta_y)
	theta_d = np.arctan2((w_v*delta_x + np.sin(pose[2])), (-w_v*delta_y + np.cos(pose[2])))
	delta_theta = theta_d - pose[2]
	# use the differential robot model
	if delta_theta >= np.pi:
	    delta_theta = delta_theta - 2*np.pi
	elif delta_theta <= -np.pi:
	    delta_theta = delta_theta + 2*np.pi
	else:
	    delta_theta = delta_theta
	rot_speed = delta_theta/self.d_t
	a = np.sin(theta_d) - np.sin(pose[2])
	b = np.cos(pose[2]) - np.cos(theta_d)
	x_speed = delta_theta*(delta_x*a/self.d_t + delta_y*b/self.d_t)/(a*a+b*b)
	rot_speed = self.KW*rot_speed
	x_speed = self.KV*x_speed
	return x_speed, rot_speed

    # Publishing the driving command according to the computed value
    def apply_control(self, linear_spd, rotation_spd):
        drive_msg = Twist()
        drive_msg.linear = Vector3(linear_spd, 0, 0)
        drive_msg.angular = Vector3(0, 0, rotation_spd)
        self.drive_pub.publish(drive_msg)

    # Stopping
    def stop(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(drive_msg)


if __name__=="__main__":
    rospy.init_node("charging_navi_v2")
    ChargingNaviV2()
    rospy.spin()


