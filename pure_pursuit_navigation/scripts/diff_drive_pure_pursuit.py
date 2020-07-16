#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf2_ros

from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from atlas80evo_msgs.msg import FSMState


class DiffDrivePurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.

	Set point determined with the method described here: 
	http://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2006_1/howard_thomas_2006_1.pdf
        Relies on localization for ground truth vehicle position. """
    def __init__(self):
        # Adjustable Parameters
	self.traj_topic = rospy.get_param("~trajectory_topic")
	self.odom_topic       = rospy.get_param("~odom_topic")
        self.state_topic      = rospy.get_param("~state_topic")
	self.drive_topic      = rospy.get_param("~drive_topic")

	self.lookahead        = rospy.get_param("~lookahead")
	self.max_reacquire    = rospy.get_param("~max_reacquire")
	self.controller_freq  = float(rospy.get_param("~controller_freq", "10.0"))
	self.wrap             = bool(rospy.get_param("~wrap"))
	wheelbase_length      = float(rospy.get_param("~wheelbase"))
        self.KV               = float(rospy.get_param("~KV"))
        self.KW               = float(rospy.get_param("~KW"))

        # Internal USE Variables - Modify with consultation
	self.trajectory  = utils.LineTrajectory("/followed_trajectory")
	self.do_viz      = True
	self.odom_timer  = utils.Timer(10)
	self.iters       = 0
        self.tf2Buffer   = tf2_ros.Buffer()
        self.listener    = tf2_ros.TransformListener(self.tf2Buffer)
	self.d_t         = float(1/self.controller_freq)
        self.rate        = rospy.Rate(self.controller_freq)
        self.allow_drive = False

	self.nearest_point   = None
	self.lookahead_point = None

	# set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
        # Publishers
	self.viz_name = "/pure_pursuit"
	self.nearest_point_pub = rospy.Publisher(self.viz_name + "/nearest_point", Marker, queue_size=1)
	self.lookahead_point_pub = rospy.Publisher(self.viz_name + "/lookahead_point", Marker, queue_size=1)
	# - topic to send drive commands to
	self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)

        # Subscribers
	# - topic to listen for trajectories
	self.traj_sub = rospy.Subscriber(self.traj_topic, PolygonStamped, self.trajectoryCB, queue_size=1)
        self.state_sub = rospy.Subscriber(self.state_topic, FSMState, self.stateCB, queue_size=1)

	# topic to listen for odometry messages, either from particle filter or robot localization
	self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odomCB, queue_size=1)

	print "Initialized. Waiting on messages..."

    def visualize(self):
	''' Publishes visualization topics:
	       - Circle to indicate the nearest point along the trajectory
	       - Circle to indicate the chosen lookahead point '''
	if not self.do_viz:
	    return
	# visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
	if self.nearest_point_pub.get_num_connections() > 0 and isinstance(self.nearest_point, np.ndarray):
	    self.nearest_point_pub.publish(utils.make_circle_marker(
	    self.nearest_point, 0.5, [0.0,0.0,1.0], "/map", self.viz_name, 0, 3))

	if self.lookahead_point_pub.get_num_connections() > 0 and isinstance(self.lookahead_point, np.ndarray):
	    self.lookahead_point_pub.publish(utils.make_circle_marker(
	    self.lookahead_point, 0.5, [1.0,1.0,1.0], "/map", self.viz_name, 1, 3))

    # Trajectory Callback
    def trajectoryCB(self, msg):
	''' Clears the currently followed trajectory, and loads the new one from the 
	    message '''
	print "Receiving new trajectory:", len(msg.polygon.points), "points" 
	self.trajectory.clear()
	self.trajectory.fromPolygon(msg.polygon)
	self.trajectory.publish_viz(duration=0.0)


    # Main Loop
    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                transform = self.tf2Buffer.lookup_transform(self.frame_global, self.frame_robot, rospy.Time())
                print transform
                self.pure_pursuit()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            self.rate.sleep()


    def odomCB(self, msg):
	''' Extracts robot state information from the message, and executes pure pursuit 
	    control. '''
	pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, utils.quaternion_to_angle(msg.pose.pose.orientation)])
	self.pure_pursuit(pose)

	# this is for timing info
	self.odom_timer.tick()
	self.iters += 1
	if self.iters % 20 == 0:
	    print "Control fps:", self.odom_timer.fps()

    def pure_pursuit(self, pose):
	''' Determines and applies Pure Pursuit control law

	    1. Find the nearest point on the trajectory
	    2. Traverse the trajectory looking for the nearest point that is the 
		 lookahead distance away from the car, and further along the path than 
		 the nearest point from step (1). This is the lookahead point.
	    3. Determine steering angle necessary to travel to the lookahead point from 
		 step (2)
	    4. Send the desired speed and steering angle commands to the robot

	    Special cases:
	        - If nearest_point is beyond the max path reacquisition distance, stop
	        - If nearest_point is between max reacquisition dist and lookahead dist, 
		    navigate to nearest_point
	        - If nearest_point is less than the lookahead distance, find the 
		    lookahead point as normal '''
	# stop if no trajectory has been received
	if self.trajectory.empty():
	    return self.stopping()

	# this instructs the trajectory to convert the list of waypoints into a numpy matrix
	if self.trajectory.dirty():
	    self.trajectory.make_np_array()

	# step 1
	nearest_point, nearest_dist, t, i = utils.nearest_point_on_trajectory(pose[:2], self.trajectory.np_points)
	self.nearest_point = nearest_point

	if nearest_dist < self.lookahead:
	    # step 2
	    lookahead_point, i2, t2 = \
		utils.first_point_on_trajectory_intersecting_circle(pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
	    if i2 == None:
		if self.iters % 5 == 0:
		    print "Could not find intersection, end of path?"
		self.lookahead_point = None
	    else:
		if self.iters % 5 == 0:
		    print "found lookahead point"
		self.lookahead_point = lookahead_point
	elif nearest_dist < self.max_reacquire:
	    if self.iters % 5 == 0:
		print "Reacquiring trajectory"
	    self.lookahead_point = self.nearest_point
	else:
	    self.lookahead_point = None

	# stop of there is no navigation target, otherwise use mobile robot geometry to navigate there
	if not isinstance(self.lookahead_point, np.ndarray):
	    self.stopping()
	else:
            if(self.allow_drive == True):
		xspeed, rotspeed = self.determine_speeds(pose, self.lookahead_point)
		# send the control commands
		self.apply_control(xspeed, rotspeed)
	    else:
                self.stopping()

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

    def apply_control(self, xspeed, rotspeed):
	cmd = Twist()
	cmd.linear.x = xspeed
	cmd.angular.z = rotspeed
	self.drive_pub.publish(cmd)

    def stopping(self):
	print "Stopping"
	cmd = Twist()
	cmd.linear.x = 0
	cmd.angular.z = 0
	self.drive_pub.publish(cmd)


if __name__=="__main__":
    rospy.init_node("diff_drive_pure_pursuit")
    pf = DiffDrivePurePursuit()
    rospy.spin()

