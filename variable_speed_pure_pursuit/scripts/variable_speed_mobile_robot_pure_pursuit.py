#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  (1)
               (2)
   Features :  
'''

import rospy
import numpy as np
import time
import utils

from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Twist, Vector3
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry


class VariableSpeedMobileRobotPurePursuit(object):
    def __init__(self):
        # Define Adjustable Parameters
        self.max_reacquire     = rospy.get_param("~max_reacquire")
        self.controller_freq   = float(rospy.get_param("~controller_freq", "10.0"))
        self.wrap              = bool(rospy.get_param("~wrap"))

        # Internal USE Variables - Do not modify without consultation
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.model       = utils.MobileRobotControlModel()
        self.do_viz      = True
        self.odom_timer  = utils.Timer(10)
        self.iters       = 0
        self.d_t         = float(1/self.controller_freq)
        self.check_drive = True
        self.nearest_point   = None
        self.lookahead_point = None
	self.current_pose = None
        self.chosen_lookahead = None
        self.chosen_KV = None
        self.chosen_KW = None

        # Publishers
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.nearest_pt_pub = rospy.Publisher("/pure_pursuit/nearest_point", Marker, queue_size=1)
        self.lookahead_pt_pub = rospy.Publisher("/pure_pursuit/lookahead_point", Marker, queue_size=1)
	self.lookahead_dist_pub = rospy.Publisher("/pure_pursuit/lookahead_dist", Marker, queue_size=1)

        # Subscribers
        self.traj_sub = rospy.Subscriber(rospy.get_param("~trajectory_topic"), PolygonStamped, self.trajectoryCB, queue_size=1)
        self.odom_sub = rospy.Subscriber(rospy.get_param("~odom_topic"),  Odometry, self.odomCB, queue_size=1)
        self.check_drive_sub = rospy.Subscriber(rospy.get_param("~check_drive_topic"), Bool, self.check_driveCB, queue_size=1)
	print "Initialized. Waiting on messages..."

    def visualize(self):
        ''' Publishes visualization topics:
            - Circle to indicate the nearest point along the trajectory
            - Circle to indicate the chosen lookahead point '''
        if not self.do_viz:
            return
	# visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
	if self.nearest_pt_pub.get_num_connections() > 0 and isinstance(self.nearest_point, np.ndarray):
	    self.nearest_pt_pub.publish(utils.make_circle_marker(
	    self.nearest_point, 0.5, [0.0,0.0,1.0], "/map", "/pure_pursuit", 0, 3))

	if self.lookahead_pt_pub.get_num_connections() > 0 and isinstance(self.lookahead_point, np.ndarray):
	    self.lookahead_pt_pub.publish(utils.make_circle_marker(
	    self.lookahead_point, 0.5, [1.0,1.0,1.0], "/map", "/pure_pursuit", 1, 3))

	if self.lookahead_pt_pub.get_num_connections() > 0 and isinstance(self.lookahead_point, np.ndarray) and isinstance(self.current_pose, np.ndarray):
            self.lookahead_dist_pub.publish(utils.make_line_marker(
            self.lookahead_point, self.current_pose, 0.3, [1.0,1.0,0.0], "/map", "/pure_pursuit", 0, 3))

    def check_driveCB(self, msg):
        if(msg.data == True):
            self.check_drive = True
        elif(msg.data == False):
            self.check_drive = False

    def trajectoryCB(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the 
            message '''
#        print "Receiving new trajectory:", len(msg.polygon.points), "points" 
        self.trajectory.clear()
        self.trajectory.fromPolygon(msg.polygon)
        self.trajectory.publish_viz(duration=0.0)

    def odomCB(self, msg):
        ''' Extracts robot state information from the message, and executes pure pursuit 
            control. '''
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, utils.quaternion_to_angle(msg.pose.pose.orientation)])
        self.current_pose = pose
        self.pure_pursuit(pose)
        # this is for timing info
        self.odom_timer.tick()
        self.iters += 1
#        if self.iters % 20 == 0:
#            print "Control fps:", self.odom_timer.fps()

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
            return self.stop()

        # this instructs the trajectory to convert the list of waypoints into a numpy matrix
        if self.trajectory.dirty():
            self.trajectory.make_np_array()

        # step 1
        nearest_point, nearest_dist, t, i = utils.nearest_point_on_trajectory(pose[:2], self.trajectory.np_points)
        self.nearest_point = nearest_point

        desired_speed = self.trajectory.speed_at_t(t+i) # t+i
        self.chosen_lookahead = self.model.speed_to_lookahead(desired_speed)
        self.chosen_KV = self.model.speed_to_kv(desired_speed)
        self.chosen_KW = self.model.speed_to_kw(desired_speed)
        print self.chosen_lookahead
        print self.chosen_KV
        print self.chosen_KW
        print "------------------------------------------------------"

        if nearest_dist < self.chosen_lookahead:
            # step 2
            lookahead_point, i2, t2 = \
                utils.first_point_on_trajectory_intersecting_circle(pose[:2], self.chosen_lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
            if i2 == None:
#                if self.iters % 5 == 0:
#                    print "Could not find intersection, end of path?"
                self.lookahead_point = None
            else:
#                if self.iters % 5 == 0:
#                    print "found lookahead point"
                self.lookahead_point = lookahead_point
        elif nearest_dist < self.max_reacquire:
#            if self.iters % 5 == 0:
#                print "Reacquiring trajectory"
            self.lookahead_point = self.nearest_point
        else:
            self.lookahead_point = None

        # stop if there is no navigation target, otherwise use mobile robot geometry to navigate there
        if not isinstance(self.lookahead_point, np.ndarray):
            self.stop()
        else:
            if(self.check_drive == True):
                xspeed, rotspeed = self.determine_speeds(pose, self.lookahead_point)
                # send the control commands
                self.apply_control(xspeed, rotspeed)
            else:
                self.stop()
        self.visualize()

    # Calculate the desired speed using mobile robot geometry
    def determine_speeds(self, pose, lookahead_point):
        # Transform the lookahead point into the coordinate frame of the car
        delta_x = lookahead_point[0] - pose[0]
        delta_y = lookahead_point[1] - pose[1]
        w_v = 2*(delta_y*np.cos(pose[2]) - delta_x*np.sin(pose[2]))/(delta_x**2+delta_y**2)
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
        x_speed = delta_theta*(delta_x*a/self.d_t + delta_y*b/self.d_t)/(a**2+b**2)
        rot_speed = self.chosen_KW*rot_speed
        x_speed = self.chosen_KV*x_speed
        return x_speed, rot_speed

    # Driving the Vehicle according to the given linear speed & rotation speed
    def apply_control(self, linear_speed, rotation_speed):
        drive_msg = Twist()
        drive_msg.linear = Vector3(linear_speed,0,0)
        drive_msg.angular = Vector3(0,0,rotation_speed)
        self.drive_pub.publish(drive_msg)

    # Stopping the Vehicle
    def stop(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0,0,0)
        drive_msg.angular = Vector3(0,0,0)
        self.drive_pub.publish(drive_msg)



if __name__=="__main__":
    rospy.init_node("variable_speed_mobile_robot_pure_pursuit")
    VariableSpeedMobileRobotPurePursuit()
    rospy.spin()

