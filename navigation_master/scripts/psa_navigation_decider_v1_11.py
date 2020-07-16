#!/usr/bin/env python

"""
Changes :  (1) "Psa_Home" is assigned as default when receiving a new mission
           (2) "No_Action" for "Psa_Home" is needless and removed
"""

import rospy
import numpy as np
import tf.transformations

from pygame import mixer
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry

# "status" : "started / arrived / departed / suspended / resumed / cancelled / completed"
# "lifter" : "up / down"

class PsaNavigationDeciderV1_11():
    def __init__(self):
        # Define Topics for publishing and subscribing messages
        self.battery_topic = rospy.get_param("~battery_topic")
        self.pose_topic = rospy.get_param("~pose_topic")
        self.lifter_topic = rospy.get_param("~lifter_topic")
        self.finish_180_turn_topic = rospy.get_param("~finish_180_turn_topic")
        self.finish_turn_topic = rospy.get_param("~finish_turn_topic")
        self.finish_table_topic = rospy.get_param("~finish_table_topic")
        self.finish_charging_topic = rospy.get_param("~finish_charging_topic")
        self.moving_topic = rospy.get_param("~moving_topic")
        self.traj_topic = rospy.get_param("~traj_topic")
        self.mission_topic = rospy.get_param("~mission_topic")
        self.path_topic = rospy.get_param("~path_topic")
        self.all_topic = rospy.get_param("~all_topic")
        self.suspend_topic = rospy.get_param("~suspend_topic")
        self.check_resume_topic = rospy.get_param("~check_resume_topic")
        self.init_180_turn_topic = rospy.get_param("~init_180_turn_topic")
        self.init_turn_topic = rospy.get_param("~init_turn_topic")
        self.init_table_topic = rospy.get_param("~init_table_topic")
        self.init_lift_topic = rospy.get_param("~init_lift_topic")
        self.init_charging_topic = rospy.get_param("~init_charging_topic")
        self.check_drive_topic = rospy.get_param("~check_drive_topic")
        self.obstacle_mode_topic = rospy.get_param("~obstacle_mode_topic")
        self.init_mag_topic = rospy.get_param("~init_mag_topic")
        self.finish_mag_topic = rospy.get_param("~finish_mag_topic")

        # Define Adjustable Parameters
        self.path_directory = rospy.get_param("~path_directory")
        self.wait_time = int(rospy.get_param("~wait_time"))    # [sec]
        self.battery_percent_safe = float(rospy.get_param("~battery_percent_safe"))    # [%]
        self.check_point = np.array( [float(rospy.get_param("~check_point_x")), float(rospy.get_param("~check_point_y"))] )
        self.check_heading_min = float(rospy.get_param("~check_heading_min"))
        self.check_heading_max = float(rospy.get_param("~check_heading_max"))

        # Define Paths' File Name
        self.path_1 = self.path_directory + "/psa_home_A.traj"
        self.path_2 = self.path_directory + "/psa_home_B.traj"
        self.path_3 = self.path_directory + "/psa_home_C.traj"
        self.path_4 = self.path_directory + "/psa_home_1.traj"
        self.path_5 = self.path_directory + "/psa_home_2.traj"
        self.path_6 = self.path_directory + "/psa_home_3.traj"
        self.path_7 = self.path_directory + "/psa_home_4.traj"
        self.path_8 = self.path_directory + "/psa_home_5.traj"
        self.path_9 = self.path_directory + "/psa_A_1.traj"
        self.path_10 = self.path_directory + "/psa_A_2.traj"
        self.path_11 = self.path_directory + "/psa_A_3.traj"
        self.path_12 = self.path_directory + "/psa_A_4.traj"
        self.path_13 = self.path_directory + "/psa_A_5.traj"
        self.path_14 = self.path_directory + "/psa_B_1.traj"
        self.path_15 = self.path_directory + "/psa_B_2.traj"
        self.path_16 = self.path_directory + "/psa_B_3.traj"
        self.path_17 = self.path_directory + "/psa_B_4.traj"
        self.path_18 = self.path_directory + "/psa_B_5.traj"
        self.path_19 = self.path_directory + "/psa_C_1.traj"
        self.path_20 = self.path_directory + "/psa_C_2.traj"
        self.path_21 = self.path_directory + "/psa_C_3.traj"
        self.path_22 = self.path_directory + "/psa_C_4.traj"
        self.path_23 = self.path_directory + "/psa_C_5.traj"
        self.path_24 = self.path_directory + "/psa_1_A.traj"
        self.path_25 = self.path_directory + "/psa_1_B.traj"
        self.path_26 = self.path_directory + "/psa_1_C.traj"
        self.path_27 = self.path_directory + "/psa_2_A.traj"
        self.path_28 = self.path_directory + "/psa_2_B.traj"
        self.path_29 = self.path_directory + "/psa_2_C.traj"
        self.path_30 = self.path_directory + "/psa_3_A.traj"
        self.path_31 = self.path_directory + "/psa_3_B.traj"
        self.path_32 = self.path_directory + "/psa_3_C.traj"
        self.path_33 = self.path_directory + "/psa_4_A.traj"
        self.path_34 = self.path_directory + "/psa_4_B.traj"
        self.path_35 = self.path_directory + "/psa_4_C.traj"
        self.path_36 = self.path_directory + "/psa_5_A.traj"
        self.path_37 = self.path_directory + "/psa_5_B.traj"
        self.path_38 = self.path_directory + "/psa_5_C.traj"
        self.path_39 = self.path_directory + "/psa_A_home.traj"
        self.path_40 = self.path_directory + "/psa_B_home.traj"
        self.path_41 = self.path_directory + "/psa_C_home.traj"
        self.path_42 = self.path_directory + "/psa_1_home.traj"
        self.path_43 = self.path_directory + "/psa_2_home.traj"
        self.path_44 = self.path_directory + "/psa_3_home.traj"
        self.path_45 = self.path_directory + "/psa_4_home.traj"
        self.path_46 = self.path_directory + "/psa_5_home.traj"
        self.path_47 = self.path_directory + "/psa_completed.traj"
        self.sound_file = "/home/atlas80-b/catkin_ws/src/atlas80/sounds/countdown_10_seconds.ogg"
        self.bg_sound_file = "/home/atlas80-b/catkin_ws/src/atlas80/sounds/bb83.ogg"

        # Internal Use Variables - Do not modify without consultation
        self.pub_rate = rospy.Rate(4)    # 4 [Hz] <---> 0.25 [sec]
        self.voltage_percent = "100.0"
        self.c_pose = np.array( [100.0, 100.0] )    # [m, m]
        self.yaw = 0.0    # [deg]
        self.lifter = "0"
        self.moving = "0"
        self.location = "home"
        self.start_pt = np.array( [0.0, 0.0] )
        self.end_pt = np.array( [0.0, 0.0] )
        self.prev_end_pt = np.array( [0.0, 0.0] )
        self.last_end_pt = np.array( [1000.0, 1000.0] )
        self.route = []
        self.action = []
        self.last_msg = ""
        self.counter = 0  #1 - 1st_Route | 2 - 2nd_Route | 3 - 3rd_Route | 4 - Finish_Job
        self.battery_safe = False   # battery level safe for doing delivery or not
        self.battery_time_counter = 0
        self.finish_wait = False    # finish waiting or not
        self.waypt_tolerance = 1.0   # [m]
        self.time_counter = 0
        self.timer = self.wait_time * 4    # [count = sec * Hz]
        self.finish_action = True   # finish appointed action or not
        self.volume = 1.0
        self.play_counter = 1
        self.delivery_status = ""
        self.start_check = False    # when to start on checking
        self.start_counter = 0
        self.cmd = False
        self.finish_180_turn = False    # finish 180' turning or not
        self.finish_turn = 0       # finish heading turner or not
        self.finish_lift = False    # finish lifting / lowering or not
        self.finish_table = False   # finish table navi or not
        self.step_counter = 1       # Steps to do 1~3
        self.drop_counter = 0       # used to monitoring Command Once
        self.lift_counter = 0       # used to monitoring Command Once
        self.finish_go = False
        self.obstacle_mode = 1
        self.finish_charging = False
        self.first_heading = True
        self.heading_check = True
        self.heading = ""
        self.heading_counter = 1
        self.finish_heading = True
        self.mag_goal = ""
        self.mag_once = True
        self.finish_mag = True    # finish magnetic navi or not

        # Subscribers
        self.battery_sub = rospy.Subscriber(self.battery_topic, String, self.batteryCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.poseCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber(self.lifter_topic, String, self.lifterCB, queue_size=1)
        self.finish_180_turn_sub = rospy.Subscriber(self.finish_180_turn_topic, String, self.finish_180_turnCB, queue_size=1)
        self.finish_turn_sub = rospy.Subscriber(self.finish_turn_topic, String, self.finish_turnCB, queue_size=1)
        self.finish_table_sub = rospy.Subscriber(self.finish_table_topic, Bool, self.finish_tableCB, queue_size=1)
        self.finish_charging_sub = rospy.Subscriber(self.finish_charging_topic, Bool, self.finish_chargingCB, queue_size=1)
        self.finish_mag_sub = rospy.Subscriber(self.finish_mag_topic, String, self.finish_magCB, queue_size=1)
        self.moving_sub = rospy.Subscriber(self.moving_topic, Twist, self.movingCB, queue_size=1)
        self.traj_sub = rospy.Subscriber(self.traj_topic, PolygonStamped, self.trajCB, queue_size=1)
        self.mission_sub = rospy.Subscriber(self.mission_topic, String, self.missionCB, queue_size=1)
        self.check_resume_sub = rospy.Subscriber(self.check_resume_topic, Bool, self.check_resumeCB, queue_size=1)

        # Publishers
        self.path_pub = rospy.Publisher(self.path_topic, String, queue_size=1)
        self.all_pub = rospy.Publisher(self.all_topic, String, queue_size=1)
        self.suspend_pub = rospy.Publisher(self.suspend_topic, Bool, queue_size=1)
        self.init_180_turn_pub = rospy.Publisher(self.init_180_turn_topic, Bool, queue_size=1)
        self.init_turn_pub = rospy.Publisher(self.init_turn_topic, String, queue_size=1)
        self.init_table_pub = rospy.Publisher(self.init_table_topic, Bool, queue_size=1)
        self.init_lift_pub = rospy.Publisher(self.init_lift_topic, String, queue_size=1)
        self.init_charging_pub = rospy.Publisher(self.init_charging_topic, Bool, queue_size=1)
        self.check_drive_pub = rospy.Publisher(self.check_drive_topic, Bool, queue_size=1)
        self.obstacle_mode_pub = rospy.Publisher(self.obstacle_mode_topic, String, queue_size=1)
        self.init_mag_pub = rospy.Publisher(self.init_mag_topic, String, queue_size=1)

        # Initialize Sound Control
        mixer.init()
        self.sound = mixer.Sound(self.sound_file)
        self.bg_sound = mixer.Sound(self.bg_sound_file)
        self.bg_sound.set_volume(0.7)
        self.bg_sound.play(-1)

        # Main Loop
        self.adhoc_path_publisher()

    # Checking Battery Voltage (%), whether need to activate self-charging mode
    def batteryCB(self, msg):
        self.voltage_percent = msg.data

    # Checking Current Pose (x-pos, y-pos, yaw-angle)
    def poseCB(self, msg):
        self.c_pose = np.array( [msg.pose.pose.position.x, msg.pose.pose.position.y] )
        self.yaw = np.rad2deg(self.quaternion_to_angle(msg.pose.pose.orientation))

    # Checking Lifter Status (Top, Bottom)
    def lifterCB(self, msg):
        lifter_status = int(msg.data)
        if(lifter_status == 1):
            self.lifter = "1"    # Top
            if(self.start_check == True):
                self.finish_lift = True        # Done Lifting
            else:
                self.finish_lift = False       # On Going
        elif(lifter_status == 0):
            self.lifter = "0"    # Bottom
            if(self.start_check == True):
                self.finish_lift = True        # Done Dropping
            else:
                self.finish_lift = False       # On Going
        else:
            self.finish_lift = False

    # Checking Status of 180' Turning
    def finish_180_turnCB(self, msg):
        if(int(msg.data) == 2):
            self.finish_180_turn = True     # Done 180' Turning
        elif(int(msg.data) == 1):
            self.finish_180_turn = False    # On Going

    # Checking Status of Heading Turner
    def finish_turnCB(self, msg):
        if(int(msg.data) == 2):
            self.finish_turn = True    # Done Turning
        elif(int(msg.data) == 1):
            self.finish_turn = False   # On Going

    # Checking Status of Table Navi
    def finish_tableCB(self, msg):
        if(msg.data == True):
            self.finish_table = True    # Done Table Navi
        elif(msg.data == False):
            self.finish_table = False   # On Going

    # Checking Status of Self-Charging
    def finish_chargingCB(self, msg):
        if(msg.data == True):
            self.finish_charging = True    # Done Self-Charging
        elif(msg.data == False):
            self.finish_charging = False   # Not Doing

    # Checking Status of Magnetic Navi
    def finish_magCB(self, msg):
        if(int(msg.data) == 1):
            self.finish_mag = True    # Done Magnetic Navi
        elif(int(msg.data) == 2):
            self.finish_mag = False   # On Going

    # Checking Moving Status (Moving, Stop)
    def movingCB(self, msg):
        if(msg.linear.x != 0.0 or msg.angular.z != 0.0):
            self.moving = "1"    # Moving
        else:
            self.moving = "0"    # Stop

    # Checking finish waiting (Resume being toggled)
    def check_resumeCB(self, msg):
        if(self.start_check == True):
            # If Resume is being toggled, finish_waiting = True
            if(msg.data == False):
                self.finish_wait = True

    # Checking the start-point and end-point of a selected path
    def trajCB(self, msg):
        self.start_pt = np.array( [msg.polygon.points[0].x, msg.polygon.points[0].y] )
        self.end_pt = np.array( [msg.polygon.points[-1].x, msg.polygon.points[-1].y] )
        self.prev_end_pt = np.array( [msg.polygon.points[-2].x, msg.polygon.points[-2].y] )

    # Receiving Mission from WebServer
    def missionCB(self, msg):
        if(self.last_msg != msg.data and msg.data != "" and self.battery_safe == True and (self.delivery_status == "completed" or self.delivery_status == "")):
            route_action = []
            self.route = ["Psa_Home"]
            self.action = []
            id_and_mission = msg.data.split("=")
            mission_id = id_and_mission[0]
            mission = id_and_mission[1].split(",")
            for i in xrange(len(mission)):
                route_action.append(mission[i].split("-"))
                self.route.append(route_action[i][0])
                self.action.append(int(route_action[i][1]))
            self.counter = 1
            self.location = "Psa_Home"
            self.delivery_status = "started"
            self.first_heading = True
            self.mag_once = True
            self.mag_goal = ""
        self.last_msg = msg.data

    # 1st - x_pos | 2nd - y_pos | 3rd - yaw | 4th - lifter | 5th - moving
    # 6th - voltage | 7th - delivery_status | 8th - location
    # All Robot Status Publisher
    def status_publisher(self):
        all_msg = str(self.c_pose[0]) +","+ str(self.c_pose[1]) +","+ str(self.yaw) +","+ self.lifter +","+ self.moving +","+ self.voltage_percent +","+ self.delivery_status +","+ self.location
        self.all_pub.publish(all_msg)

    # Adhoc Path Chooser
    def adhoc_path_publisher(self):
        while not rospy.is_shutdown():
            print "delivery_status :  " + str(self.delivery_status)
            print "finish_table :  " + str(self.finish_table)
            print "finish_lift :  " + str(self.finish_lift)
            print "finish_180_turn :  " + str(self.finish_180_turn)
            print "finish_turn :  " + str(self.finish_turn)
            print "finish_heading :  " + str(self.finish_heading)
            print "finish_mag :  " + str(self.finish_mag)
            print "finish_action : " + str(self.finish_action)
            print "route :  " + str(self.route)
            print "action :  " + str(self.action)
            print "yaw :  " +str(self.yaw)
            print "battery_safe :  " + str(self.battery_safe) + " | battery_percentage :  " + self.voltage_percent
            # Publish vehicle status to the Web-Server-Handler
            self.status_publisher()
            # Selecting Obstacle Detecting Mode
            self.obstacle_detector()
            # Check Vehicle's Heading Upon Receiving New Mission
            self.mission_start_checker()
            if(self.heading_check == False):  # Skip below if heading is not correct
                continue
            # Heading Turner Checker before Arriving Destination
            self.heading_turner_checker()
            if(self.finish_heading == False):
                continue
            # Check Mission Accomplished Status
            self.mission_done_checker()
            # Check Action Accomplished Status
            self.action_done_checker()
            if(self.finish_action == False):  # Skip below if action is not yet finished
                continue
            # Check Battery Status
            self.battery_safe_checker()       # Skip below if battery level is not enough for safe operating
            if(self.battery_safe == False):
                continue
            # Depart from Home
            if(self.counter == 1 and len(self.route) >= 2):
                self.location = self.route[0]
                if(self.route[0] == "Psa_Home" and self.route[1] == "Psa_A"):
                    chosen_path = self.path_1
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_B"):
                    chosen_path = self.path_2
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_C"):
                    chosen_path = self.path_3
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_1"):
                    chosen_path = self.path_4
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_2"):
                    chosen_path = self.path_5
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_3"):
                    chosen_path = self.path_6
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_4"):
                    chosen_path = self.path_7
                    self.heading = "-90"
                    self.mag_goal = "4"
                elif(self.route[0] == "Psa_Home" and self.route[1] == "Psa_5"):
                    chosen_path = self.path_8
                    self.heading = "-90"
                    self.mag_goal = "5"
            # Heading to destination
            elif(self.counter == 2 and len(self.route) >= 2):
                if(self.route[0] == "Psa_A" and self.route[1] == "Psa_1"):
                    chosen_path = self.path_9
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_A" and self.route[1] == "Psa_2"):
                    chosen_path = self.path_10
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_A" and self.route[1] == "Psa_3"):
                    chosen_path = self.path_11
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_A" and self.route[1] == "Psa_4"):
                    chosen_path = self.path_12
                    self.heading = "-90"
                    self.mag_goal = "4"
                elif(self.route[0] == "Psa_A" and self.route[1] == "Psa_5"):
                    chosen_path = self.path_13
                    self.heading = "-90"
                    self.mag_goal = "5"
                elif(self.route[0] == "Psa_B" and self.route[1] == "Psa_1"):
                    chosen_path = self.path_14
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_B" and self.route[1] == "Psa_2"):
                    chosen_path = self.path_15
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_B" and self.route[1] == "Psa_3"):
                    chosen_path = self.path_16
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_B" and self.route[1] == "Psa_4"):
                    chosen_path = self.path_17
                    self.heading = "-90"
                    self.mag_goal = "4"
                elif(self.route[0] == "Psa_B" and self.route[1] == "Psa_5"):
                    chosen_path = self.path_18
                    self.heading = "-90"
                    self.mag_goal = "5"
                elif(self.route[0] == "Psa_C" and self.route[1] == "Psa_1"):
                    chosen_path = self.path_19
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_C" and self.route[1] == "Psa_2"):
                    chosen_path = self.path_20
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_C" and self.route[1] == "Psa_3"):
                    chosen_path = self.path_21
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_C" and self.route[1] == "Psa_4"):
                    chosen_path = self.path_22
                    self.heading = "-90"
                    self.mag_goal = "4"
                elif(self.route[0] == "Psa_C" and self.route[1] == "Psa_5"):
                    chosen_path = self.path_23
                    self.heading = "-90"
                    self.mag_goal = "5"
                elif(self.route[0] == "Psa_1" and self.route[1] == "Psa_A"):
                    chosen_path = self.path_24
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_1" and self.route[1] == "Psa_B"):
                    chosen_path = self.path_25
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_1" and self.route[1] == "Psa_C"):
                    chosen_path = self.path_26
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_2" and self.route[1] == "Psa_A"):
                    chosen_path = self.path_27
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_2" and self.route[1] == "Psa_B"):
                    chosen_path = self.path_28
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_2" and self.route[1] == "Psa_C"):
                    chosen_path = self.path_29
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_3" and self.route[1] == "Psa_A"):
                    chosen_path = self.path_30
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_3" and self.route[1] == "Psa_B"):
                    chosen_path = self.path_31
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_3" and self.route[1] == "Psa_C"):
                    chosen_path = self.path_32
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_4" and self.route[1] == "Psa_A"):
                    chosen_path = self.path_33
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_4" and self.route[1] == "Psa_B"):
                    chosen_path = self.path_34
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_4" and self.route[1] == "Psa_C"):
                    chosen_path = self.path_35
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_5" and self.route[1] == "Psa_A"):
                    chosen_path = self.path_36
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_5" and self.route[1] == "Psa_B"):
                    chosen_path = self.path_37
                    self.heading = "-90"
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_5" and self.route[1] == "Psa_C"):
                    chosen_path = self.path_38
                    self.heading = "-90"
                    self.mag_goal = "0"
            # Returning Home
            elif(self.counter == 3 and len(self.route) == 1):
                if(self.route[0] == "Psa_A"):
                    chosen_path = self.path_39
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_B"):
                    chosen_path = self.path_40
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_C"):
                    chosen_path = self.path_41
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_1"):
                    chosen_path = self.path_42
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_2"):
                    chosen_path = self.path_43
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_3"):
                    chosen_path = self.path_44
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_4"):
                    chosen_path = self.path_45
                    self.heading = ""
                    self.mag_goal = "0"
                elif(self.route[0] == "Psa_5"):
                    chosen_path = self.path_46
                    self.heading = ""
                    self.mag_goal = "0"
            elif(self.counter == 4 and len(self.route) == 0):
                chosen_path = self.path_47
                self.heading = ""
                self.mag_goal = "0"
            else:
                chosen_path = None
                self.heading = ""
                self.mag_goal = "0"
            # Keep on updating path for vehicle - Waypoint Navi
            if(chosen_path != None):
                self.path_pub.publish(chosen_path)
            # Initiate the magnetic navi
            if(self.mag_goal != "" and self.mag_once == True):
                self.init_mag_pub.publish(self.mag_goal)
            # Magnetic Navi Checker
            if(self.finish_mag == False):
                self.check_drive_pub.publish(False)
                self.pub_rate.sleep()
                self.mag_once = False
                continue
            print chosen_path
            # Activate Waypoint Navi
            self.check_drive_pub.publish(True)
            # Reset Counters after every complete iteration
            self.finish_table = False   # Reset the finish_table
            self.finish_lift = False   # Reset the finish_lift
            self.finish_180_turn = False  # Reset the finish_180_turn
            self.finish_turn = False   # Reset the finish_turn
            self.finish_mag = True   # Reset the finish_mag
            print "-------------------------------------------------"
            self.pub_rate.sleep()

    # Check the Vehicle's Heading after receiving a New Mission
    def mission_start_checker(self):
        if(self.delivery_status == "started" and self.dist_checker(self.c_pose, self.check_point, 0.5)):
            if(self.check_heading_min <= self.yaw <= self.check_heading_max):
                self.init_180_turn_pub.publish(False)
                self.check_drive_pub.publish(True)
                self.heading_check = True
                self.finish_180_turn = False
            else:
                self.heading_check = False
                self.check_drive_pub.publish(False)
                self.init_180_turn_pub.publish(True)
            self.pub_rate.sleep()

    # Check the Vehicle whether doing the Heading Turner before arriving destination is needed
    def heading_turner_checker(self):
        if(self.heading != "" and self.dist_checker(self.c_pose, self.prev_end_pt, 0.3) == True and self.heading_counter == 1):
            self.init_turn_pub.publish(self.heading)
            self.heading_counter = 0
            self.check_drive_pub.publish(False)
            self.finish_heading = False
            self.pub_rate.sleep()
        else:
            self.finish_heading = True
#            self.check_drive_pub.publish(True)

    # Check the Mission Accomplished Status
    def mission_done_checker(self):
        if(self.route != []):
            # 1st completeness (arrived 1st location)
            if(self.dist_checker(self.c_pose, self.end_pt, self.waypt_tolerance) and self.counter == 1 and (self.last_end_pt[0] != self.end_pt[0] or self.last_end_pt[1] != self.end_pt[1])):
                self.counter = 2
                del self.route[0]
                self.delivery_status = "arrived"
                self.location = self.route[0]
                self.last_end_pt = self.end_pt
                self.heading_counter = 1
            # 2nd completeness (arrived 2nd location)
            elif(self.dist_checker(self.c_pose, self.end_pt, self.waypt_tolerance) and self.counter == 2 and (self.last_end_pt[0] != self.end_pt[0] or self.last_end_pt[1] != self.end_pt[1])):
                self.counter = 3
                del self.route[0]
                self.delivery_status = "arrived"
                self.location = self.route[0]
                self.last_end_pt = self.end_pt
                self.heading_counter = 1
            # 3rd completeness (back to home)
            elif(self.dist_checker(self.c_pose, self.end_pt, self.waypt_tolerance) and self.counter == 3 and (self.last_end_pt[0] != self.end_pt[0] or self.last_end_pt[1] != self.end_pt[1])):
                self.counter = 4 # Finish Job
                del self.route[0]
                self.last_end_pt = self.end_pt
                self.location = "Psa_Home"
                self.heading_counter = 1
            else:
                self.counter = self.counter
        elif(self.route == [] and self.counter == 4):
            self.delivery_status = "completed"
            self.obstacle_mode = 1
            self.last_end_pt = self.end_pt

    # Check the Action Accomplished Status
    def action_done_checker(self):
        if(len(self.action) >=1):
            # Stop & Wait ----- (1) Turning 180', (2) Stop & Wait
            if(self.action[0] == 3 and self.delivery_status == "arrived"):
                self.check_drive_pub.publish(False)
                print "Stop & Wait"
                if(self.finish_180_turn == False or self.finish_wait == False):
                    self.finish_action = False
                    # Initiating - (1) Turning 180'
                    if(self.step_counter == 1):
                        self.init_180_turn_pub.publish(True)
                        self.step_counter = 2
                    # Initiating - (2) Stop & Wait
                    elif(self.step_counter == 2 and self.finish_180_turn == True):
                        self.init_180_turn_pub.publish(False)
                        if(self.start_counter == 0):
                            self.toggle_suspend_resume()
                        # Use for delaying the checkCB()
                        if(self.start_counter == 6):
                            self.start_check = True
                        self.start_counter = self.start_counter + 1
                # Resume the vehicle after someone toggle "Resume"
                elif(self.finish_180_turn == True and self.finish_wait == True):
                    del self.action[0]
                    self.finish_180_turn = False
                    self.finish_wait = False
                    self.finish_action = True
                    self.delivery_status = "departed"
                    self.start_check = False
                    self.start_counter = 0
                    self.step_counter = 1
                    self.init_180_turn_pub.publish(False)
            # Stop & Go ----- (1) Turning 180', (2) Stop & Go
            elif(self.action[0] == 4 and self.delivery_status == "arrived"):
                self.check_drive_pub.publish(False)
                print "Stop & Go"
                if(self.finish_180_turn == False or self.finish_go == False):
                    self.finish_action = False
                    # Initiating - (1) Turning 180'
                    if(self.step_counter == 1):
                        self.init_180_turn_pub.publish(True)
                        self.step_counter = 2
                    # Initiating - (2) Stop & Go
                    elif(self.step_counter == 2 and self.finish_180_turn == True):
                        self.init_180_turn_pub.publish(False)
                        remaining_time = int((self.timer - self.time_counter)/1)
                        # Suspend the vehicle during countdown
                        if(self.time_counter < self.timer):
                            if(self.time_counter == 0):
                                self.toggle_suspend_resume()
                            if(remaining_time == 44 and self.play_counter == 1):
                                self.sound.set_volume(self.volume)
                                self.sound.play()
                                self.play_counter = 0
                            self.time_counter = self.time_counter + 1
                        # Resume the vehicle after finish countdown
                        else:
                            self.time_counter = 0
                            if(self.time_counter == 0):
                                self.toggle_suspend_resume()
                                self.finish_go = True
                elif(self.finish_180_turn == True and self.finish_go == True):
                    del self.action[0]
                    self.finish_180_turn = False
                    self.finish_go = False
                    self.finish_action = True
                    self.delivery_status = "departed"
                    self.play_counter = 1
                    self.step_counter = 1
                    self.init_180_turn_pub.publish(False)
            # No Action
            elif(self.action[0] == 0):
                print "No Action"
                self.finish_action = True
                del self.action[0]
            # Pick Table ----- (1) Table Navi, (2) Lift Table, (3) Turning 180'
            elif(self.action[0] == 1 and self.delivery_status == "arrived"):
                self.check_drive_pub.publish(False)
                self.obstacle_mode = 0
                print "Pick Table"
                if(self.finish_table == False or self.finish_lift == False or self.finish_180_turn == False):
                    self.finish_action = False
                    # Initiating - (1) Table Navi
                    if(self.step_counter == 1):
                        self.init_table_pub.publish(True)
                        self.step_counter = 2
                    # Initiating - (2) Lift Table
                    elif(self.step_counter == 2 and self.finish_table == True):
                        self.init_table_pub.publish(False)
                        if(self.lift_counter < 12):  # 8 <--> 2 [sec]
                            self.init_lift_pub.publish("1")    # lifting up
                        # Use for delaying the checking on lifterCB()
                        if(self.lift_counter >= 12):  # 8 <--> 2 [sec]
                            self.start_check = True
                            self.step_counter = 3
                        self.lift_counter = self.lift_counter + 1
                    # Initiating - (3) Turning 180'
                    elif(self.step_counter == 3 and self.finish_table == True and self.finish_lift == True):
                        self.init_table_pub.publish(False)
                        self.init_180_turn_pub.publish(True)
                # Finish All Steps
                elif(self.finish_table == True and self.finish_lift == True and self.finish_180_turn == True):
                    print "All Steps Finished"
                    del self.action[0]
                    self.finish_action = True
                    self.delivery_status = "departed"
                    self.step_counter = 1
                    self.start_check = False
                    self.lift_counter = 0
                    self.init_table_pub.publish(False)
                    self.init_180_turn_pub.publish(False)
                    self.obstacle_mode = 2
            # Drop Table ----- (1) Turning 180', (2) Drop Table
            elif(self.action[0] == 2 and self.delivery_status == "arrived"):
                self.check_drive_pub.publish(False)
                self.obstacle_mode = 0
                print "Drop Table"
                if(self.finish_180_turn == False or self.finish_lift == False):
                    self.finish_action = False
                    # Initiating - (1) Turning 180'
                    if(self.step_counter == 1):
                        self.init_180_turn_pub.publish(True)
                        self.step_counter = 2
                    # Initiating - (2) Drop Table
                    elif(self.step_counter == 2 and self.finish_180_turn == True):
                        self.init_180_turn_pub.publish(False)
                        if(self.drop_counter < 12):  # 8 <--> 2 [sec]
                            self.init_lift_pub.publish("0")    # lowering down
                        # Use for delaying the checking on lifterCB()
                        if(self.drop_counter >= 12):  # 8 <--> 2 [sec]
                            self.start_check = True
                        self.drop_counter = self.drop_counter + 1
                # Finish All Steps
                elif(self.finish_180_turn == True and self.finish_lift == True):
                    del self.action[0]
                    self.finish_action = True
                    self.delivery_status = "departed"
                    self.step_counter = 1
                    self.start_check = False
                    self.drop_counter = 0
                    self.init_180_turn_pub.publish(False)
                    self.obstacle_mode = 1
        self.pub_rate.sleep()

    # Check battery percentage, vehicle is not allowed to do the delivery when drop below certain %
    def battery_safe_checker(self):
        if(float(self.voltage_percent) <= self.battery_percent_safe and (self.delivery_status == "completed" or self.delivery_status == "" or self.delivery_status == "charging")):
            if(self.battery_time_counter > 16):    # Wait for 4 sec
                self.init_charging_pub.publish(True)
                self.check_drive_pub.publish(False)
                self.delivery_status = "charging"
                self.battery_time_counter = 0
            self.battery_time_counter = self.battery_time_counter + 1
            self.battery_safe = False
        elif(float(self.voltage_percent) > self.battery_percent_safe and self.finish_charging == True):
            self.init_charging_pub.publish(False)
            self.check_drive_pub.publish(False)
            self.delivery_status = "completed"
            self.battery_safe = True
            self.finish_charging = False
            self.battery_time_counter = 0
        elif(float(self.voltage_percent) > self.battery_percent_safe and self.delivery_status != "charging"):
            self.battery_safe = True
#            self.check_drive_pub.publish(True)
        self.pub_rate.sleep()

    # Obstacle Detection Mode Selecter
    def obstacle_detector(self):
        if(int(self.lifter) == 1 and self.obstacle_mode == 2):    # With Table On Top
            self.obstacle_mode_pub.publish("2")
        elif(int(self.lifter) == 0 and self.obstacle_mode == 0):  # Doing Table Navi or Dropping Table
            self.obstacle_mode_pub.publish("0")
        elif(int(self.lifter) == 0 and self.obstacle_mode == 1 and self.dist_checker(self.c_pose, self.start_pt, 2.0) == False):  # Without Table On Top
            self.obstacle_mode_pub.publish("1")

    # Distance Checker, Checking whether distance between 2 points is within distance_tolerance or not
    def dist_checker(self, a, b, dist_tolerance):
        distance = np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        if(distance <= dist_tolerance):
            return True
        else:
            return False

    # Trigger Suspend/Resume like a toggle button
    def toggle_suspend_resume(self):
        self.cmd = not self.cmd
        self.suspend_pub.publish(self.cmd)

    # Convert Quaternion to Angle
    def quaternion_to_angle(self, q):
	"""Convert a quaternion into angle [rad].
	The angle represents the yaw.
	This is not just the z component of the quaternion."""
	x, y, z, w = q.x, q.y, q.z, q.w
	roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
	return yaw



if __name__=="__main__":
    rospy.init_node("psa_navigation_decider_v1_11")
    PsaNavigationDeciderV1_11()
    rospy.spin()
