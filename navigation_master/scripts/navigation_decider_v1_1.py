#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations

from pygame import mixer
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry

# "status" : "started / arrived / departed / suspended / resumed / cancelled / completed"
# "lifter" : "up / down"

class NavigationDeciderV1_1():
    def __init__(self):
        # Define Topics for publishing and subscribing messages
        self.battery_topic = rospy.get_param("~battery_topic")
        self.pose_topic = rospy.get_param("~pose_topic")
        self.lifter_topic = rospy.get_param("~lifter_topic")
        self.moving_topic = rospy.get_param("~moving_topic")
        self.traj_topic = rospy.get_param("~traj_topic")
        self.mission_topic = rospy.get_param("~mission_topic")
        self.path_topic = rospy.get_param("~path_topic")
        self.all_topic = rospy.get_param("~all_topic")
        self.suspend_topic = rospy.get_param("~suspend_topic")
        self.check_topic = rospy.get_param("~check_topic")
#        self.init_turn_topic = rospy.get_param("~init_turn_topic")  #"turning/init"
#        self.done_turn_topic = rospy.get_param("~done_turn_topic")  #"turning/status"

        # Define Adjustable Parameters
        self.path_directory = rospy.get_param("~path_directory")
        self.wait_time = rospy.get_param("~wait_time")              # [sec]

        # Define Paths' File Name
        self.path_1 = self.path_directory + "/home_A.traj"
        self.path_2 = self.path_directory + "/home_B.traj"
        self.path_3 = self.path_directory + "/A_1.traj"
        self.path_4 = self.path_directory + "/A_2.traj"
        self.path_5 = self.path_directory + "/A_3.traj"
        self.path_6 = self.path_directory + "/B_1.traj"
        self.path_7 = self.path_directory + "/B_2.traj"
        self.path_8 = self.path_directory + "/B_3.traj"
        self.path_9 = self.path_directory + "/1_home.traj"
        self.path_10 = self.path_directory + "/2_home.traj"
        self.path_11 = self.path_directory + "/3_home.traj"
        self.sound_file = "/home/atlas80-b/Desktop/countdown_10_seconds.ogg"

        # Internal Use Variables - Do not modify without consultation
        self.pub_rate = rospy.Rate(1)    # 0.5 [Hz] <---> 2 [sec]
        self.voltage_percent = "100.0"
        self.c_pose = np.array( [100.0, 100.0] )    # [m, m]
        self.yaw = 0.0    # [Deg]
        self.lifter = "0"
        self.moving = "0"
        self.location = "home"
        self.start_pt = np.array( [0.0, 0.0] )
        self.end_pt = np.array( [0.0, 0.0] )
        self.last_end_pt = np.array( [1000.0, 1000.0] )
        self.route = []
        self.action = []
        self.last_msg = ""
        self.counter = 0       #   1 - 1st_Route | 2 - 2nd_Route | 3 - 3rd_Route | 4 - Finish_Job
        self.safe = True       # battery level safe for doing delivery or not
        self.finish_wait = False    # finish waiting or not
        self.dist_tolerance = 1.0    # [m]
        self.time_counter = 0
        self.timer = self.wait_time * 1    # [count = sec * Hz]
        self.finish_action = True    # finish appointed action or not
        self.finish_turn = 0
        self.volume = 1.0
        self.play_counter = 1
        self.delivery_status = ""
        self.start_check = False    # when to start on checking suspend / resume status
        self.start_counter = 0
        self.cmd = False

        # Subscribers
        self.battery_sub = rospy.Subscriber(self.battery_topic, String, self.batteryCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.poseCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber(self.lifter_topic, Bool, self.lifterCB, queue_size=1)
        self.moving_sub = rospy.Subscriber(self.moving_topic, Twist, self.movingCB, queue_size=1)
        self.traj_sub = rospy.Subscriber(self.traj_topic, PolygonStamped, self.trajCB, queue_size=1)
        self.mission_sub = rospy.Subscriber(self.mission_topic, String, self.missionCB, queue_size=1)
        self.check_sub = rospy.Subscriber(self.check_topic, Bool, self.checkCB, queue_size=1)
#        self.done_turn_sub = rospy.Subscriber(self.done_turn_topic, Int32, self.done_turnCB, queue_size=1)

        # Publishers
        self.path_pub = rospy.Publisher(self.path_topic, String, queue_size=1)
        self.all_pub = rospy.Publisher(self.all_topic, String, queue_size=1)
#        self.turn_pub = rospy.Publisher(self.init_turn_topic, Bool, queue_size=1)
        self.suspend_pub = rospy.Publisher(self.suspend_topic, Bool, queue_size=1)

        mixer.init()
        self.sound = mixer.Sound(self.sound_file)

        # Main Loop
        self.adhoc_path_publisher()

    # Checking Battery Voltage, whether need to activate self-charging mode
    def batteryCB(self, msg):
        self.voltage_percent = msg.data

    # Checking Current Pose (x-pos, y-pos, yaw-angle)
    def poseCB(self, msg):
        self.c_pose = np.array( [msg.pose.pose.position.x, msg.pose.pose.position.y] )
        self.yaw = self.quaternion_to_angle(msg.pose.pose.orientation)

    # Checking Lifter Status (Up, Down)
    def lifterCB(self, msg):
        if(msg == True):
            self.lifter = "1"    # Up
        else:
            self.lifter = "0"    # Down

    # Checking Moving Status (Moving, Stop)
    def movingCB(self, msg):
        if(msg.linear.x != 0.0 or msg.angular.z != 0.0):
            self.moving = "1"    # Moving
        else:
            self.moving = "0"    # Stop

    # Checking finish waiting (Resume being toggled)
    def checkCB(self, msg):
        if(self.start_check == True):
            # If Resume is being toggled, finish_waiting = True
            if(msg.data == False):
                self.finish_wait = True

    # Checking the start-point and end-point of a selected path
    def trajCB(self, msg):
        self.start_pt = np.array( [msg.polygon.points[0].x, msg.polygon.points[0].y] )
        self.end_pt = np.array( [msg.polygon.points[-1].x, msg.polygon.points[-1].y] )

    # Receiving Mission from WebServer
    def missionCB(self, msg):
        if(self.last_msg != msg.data) and (msg.data != ""):
            route_action = []
            self.route = []
            self.action = []
            mission = msg.data.split(",")
            for i in xrange(len(mission)):
                route_action.append(mission[i].split("-"))
                self.route.append(route_action[i][0])
                self.action.append(int(route_action[i][1]))
            self.counter = 1
            self.delivery_status = "started"
        self.last_msg = msg.data

    # Checking Status of 180' Turning
    def done_turnCB(self, msg):
        if(msg.data == 2):
            self.finish_turn = 2
        elif(msg.data == 1):
            self.finish_turn = 1
        else:
            self.finish_turn = 0

    # 1st - x_pos | 2nd - y_pos | 3rd - yaw | 4th - lifter | 5th - moving
    # 6th - voltage | 7th - delivery_status | 8th - location
    # All Robot Status Publisher
    def status_publisher(self):
        all_msg = str(self.c_pose[0]) +","+ str(self.c_pose[1]) +","+ str(self.yaw) +","+ self.lifter +","+ self.moving +","+ self.voltage_percent +","+ self.delivery_status +","+ self.location
        self.all_pub.publish(all_msg)

    # Adhoc Path Chooser
    def adhoc_path_publisher(self):
        while not rospy.is_shutdown():
            print self.delivery_status
            # Check Mission Accomplished Status
            self.mission_done_checker()
            # Check Action Accomplished Status
            self.action_done_checker()
            print "finish_action : " + str(self.finish_action)
            if(self.finish_action == False):
                continue
            # Check Battery Status
            self.battery_safe_checker()
            if(self.safe == False):
                continue
            # Depart from Home
            if(self.counter == 1 and len(self.route) >= 2):
                self.location = self.route[0]
                if(self.route[0] == "Shado_Home" and self.route[1] == "Shado_A"):
                    chosen_path = self.path_1
                elif(self.route[0] == "Shado_Home" and self.route[1] == "Shado_B"):
                    chosen_path = self.path_2
            # Heading to destination
            elif(self.counter == 2 and len(self.route) >= 2):
                if(self.route[0] == "Shado_A" and self.route[1] == "Shado_1"):
                    chosen_path = self.path_3
                elif(self.route[0] == "Shado_A" and self.route[1] == "Shado_2"):
                    chosen_path = self.path_4
                elif(self.route[0] == "Shado_A" and self.route[1] == "Shado_3"):
                    chosen_path = self.path_5
                elif(self.route[0] == "Shado_B" and self.route[1] == "Shado_1"):
                    chosen_path = self.path_6
                elif(self.route[0] == "Shado_B" and self.route[1] == "Shado_2"):
                    chosen_path = self.path_7
                elif(self.route[0] == "Shado_B" and self.route[1] == "Shado_3"):
                    chosen_path = self.path_8
            # Returning Home
            elif(self.counter == 3 and len(self.route) == 1):
                if(self.route[0] == "Shado_1"):
                    chosen_path = self.path_9
                elif(self.route[0] == "Shado_2"):
                    chosen_path = self.path_10
                elif(self.route[0] == "Shado_3"):
                    chosen_path = self.path_11
            else:
                chosen_path = None
            if(chosen_path != None):
                self.path_pub.publish(chosen_path)
            print chosen_path
            # Publish vehicle status to the Web-Server-Handler
            self.status_publisher()
            self.pub_rate.sleep()

    # Check the Mission Accomplished Status
    def mission_done_checker(self):
        if(self.route != []):
            print self.dist_checker(self.c_pose, self.end_pt)
            print self.c_pose
            print self.end_pt
            print "-----------"
            # 1st completeness (arrived 1st location)
            if(self.dist_checker(self.c_pose, self.end_pt) and self.counter == 1):
                self.counter = 2
                del self.route[0]
                self.delivery_status = "arrived"
                self.location = self.route[0]
                self.last_end_pt = self.end_pt
            # 2nd completeness (arrived 2nd location)
            elif(self.dist_checker(self.c_pose, self.end_pt) and self.counter == 2 and self.last_end_pt[0] != self.end_pt[0] and self.last_end_pt[1] != self.end_pt[1]):
                self.counter = 3
                del self.route[0]
                self.delivery_status = "arrived"
                self.location = self.route[0]
                self.last_end_pt = self.end_pt
            # 3rd completeness (back to home)
            elif(self.dist_checker(self.c_pose, self.end_pt) and self.counter == 3 and self.last_end_pt[0] != self.end_pt[0] and self.last_end_pt[1] != self.end_pt[1]):
                self.counter = 4 # Finish Job
                self.last_end_pt = np.array( [0.0, 0.0] )
                del self.route[0]
                self.location = "Shado_Home"
            else:
                self.counter = self.counter
        elif(self.route == [] and self.counter == 4):
            self.delivery_status = "completed"

    # Check the Action Accomplished Status
    def action_done_checker(self):
        if(len(self.action) >=1):
            # Stop & Wait
            if(self.action[0] == 3 and self.delivery_status == "arrived"):
                print "stop & wait"
                print "finish_wait : " + str(self.finish_wait)
                # Suspend the vehicle upon arrival
                if(self.finish_wait == False):
                    if(self.start_counter == 0):
                        self.toggle_suspend_resume()
                    self.finish_action = False
                    # Use for delaying the checkCB()
                    if(self.start_counter == 3):
                        self.start_check = True
                    self.start_counter = self.start_counter + 1
                # Resume the vehicle after someone toggle "Resume"
                elif(self.finish_wait == True):
                    del self.action[0]
                    self.finish_wait = False
                    self.finish_action = True
                    self.delivery_status = "departed"
                    self.start_check = False
                    self.start_counter = 0
            # Stop & Go
            elif(self.action[0] == 4 and self.delivery_status == "arrived"):
                print "stop & go"
                remaining_time = int((self.timer - self.time_counter)/1)
                # Suspend the vehicle during countdown
                if(self.time_counter < self.timer):
                    if(self.time_counter == 0):
                        self.toggle_suspend_resume()
                    self.finish_action = False
                    if(remaining_time == 11 and self.play_counter == 1):
                        self.sound.set_volume(self.volume)
                        self.sound.play()
                        self.play_counter = 0
                    self.time_counter = self.time_counter + 1
                # Resume the vehicle after finish countdown
                else:
                    self.time_counter = 0
                    if(self.time_counter == 0):
                        self.toggle_suspend_resume()
                    del self.action[0]
                    self.finish_action = True
                    self.delivery_status = "departed"
                    self.play_counter = 1
            # No Action
            elif(self.action[0] == 0):
                print "no action"
                self.finish_action = True
                del self.action[0]
        self.pub_rate.sleep()

    # Check battery percentage, vehicle is not allowed to do the delivery when drop below certain %
    def battery_safe_checker(self):
        if(self.voltage_percent > 40.0):
            self.safe = True
        else:
            self.safe = False
        self.pub_rate.sleep()

    # Distance Checker, Checking whether distance between 2 points is within distance_tolerance or not
    def dist_checker(self, a, b):
        distance = np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        if(distance <= self.dist_tolerance):
            return True
        else:
            return False

    # Trigger Suspend/Resume like a toggle button
    def toggle_suspend_resume(self):
        self.cmd = not self.cmd
        self.suspend_pub.publish(self.cmd)

    # Convert Quaternion to Angle
    def quaternion_to_angle(self, q):
	"""Convert a quaternion _message_ into an angle in radians.
	The angle represents the yaw.
	This is not just the z component of the quaternion."""
	x, y, z, w = q.x, q.y, q.z, q.w
	roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
	return yaw



if __name__=="__main__":
    rospy.init_node("navigation_decider_v1_1")
    NavigationDeciderV1_1()
    rospy.spin()
