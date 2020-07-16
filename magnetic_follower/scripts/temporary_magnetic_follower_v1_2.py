#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

'''
Section 1:                                         Section 2:

                --- (exit pt)                                (out pt)          (pt 4)           (pt 5)
      (Mag Navi) |                                  (Mag Navi)   |----------------|----------------|-
 ..........      |      ..........                                                |   (Mag Navi)   |
   (Wall) |      |      | (Wall)                                                 ---              ---
 ``````````      |      ``````````
                 |                                                               ... (Table Navi) ...
                ---                                                              | |              | |
                                                                                 ```              ```

Features :  (1) Enable "Go into" and "Come out" of Bay 4 to 5 with magnetic following
            (2) Detecting Horizontal Magnetic Line as the signal to begin magnetic following
            (3) Ability to decide whether to skip crossroad
            (4) Automatically Choosing route according to special sequence of commands
            (5) Able to do "Auto-Relocalizing" the Lidar Localization until correct
            (6) Able to trigger LEFT or RIGHT 90' turn
            (7) Use nearly 25[Hz] to run (based on poseCB)
            (8) Add-On exiting Store's Entrance with magnetic following

Notes:
-- Internal Use Message --
command = [0, 1, 2, 3, 4]
0 = go straight
1 = left turn
2 = right turn
3 = skip crossroad
4 = end point

-- Received Incoming Message --
mag_mission = "in-4-exit"    "in-5-exit"    "out-4-"    "out-5-"

'''

class TemporaryMagneticFollowerV1_2():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.init_mag_topic = rospy.get_param("~init_mag_topic")
        self.notify_topic = rospy.get_param("~notify_topic")
        self.mag_topic = rospy.get_param("~mag_topic")
        self.mag_drive_topic = rospy.get_param("~mag_drive_topic")
        self.init_90_turn_topic = rospy.get_param("~init_90_turn_topic")
        self.relocalize_topic = rospy.get_param("~relocalize_topic")
        self.finish_mag_topic = rospy.get_param("~finish_mag_topic")
        self.pose_topic = rospy.get_param("~pose_topic")

        # Define Adjustable Parameters
        self.pt_4 = Point(rospy.get_param("~pt_4_x"), rospy.get_param("~pt_4_y"), 0.0)
        self.pt_4_heading = rospy.get_param("~pt_4_heading")
        self.pt_5 = Point(rospy.get_param("~pt_5_x"), rospy.get_param("~pt_5_y"), 0.0)
        self.pt_5_heading = rospy.get_param("~pt_5_heading")
        self.out_pt = Point(rospy.get_param("~out_pt_x"), rospy.get_param("~out_pt_y"), 0.0)
        self.out_pt_heading = rospy.get_param("~out_pt_heading")
        self.exit_pt = Point(rospy.get_param("~exit_pt_x"), rospy.get_param("~exit_pt_y"), 0.0)
        self.exit_pt_heading = rospy.get_param("~exit_pt_heading")

        # Internal Use Variables - Do not modify without consultation
        self.mag_data = None
        self.k = 0
        self.command = []
        self.tmp_command = []
        self.mag_on = False   # use only between starting and ending
        self.end = False    # use to identify whether currently is starting / ending
        self.once = False    # use to make sure the crossroad detected repeatedly won't be counted
        self.last_msg = ""
        self.goal_pose = Point()
        self.tmp_pose = Point()
        self.turn_counter = 0
        self.turn = ""
        self.step_counter = 0  # use for check whether currently should be "1 - exit mode" or "2 - bay mode"
        self.last_mag = np.zeros(16)

        # Subscribers
        self.init_mag_sub = rospy.Subscriber(self.init_mag_topic, String, self.init_magCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.poseCB, queue_size=1)
        self.mag_sub = rospy.Subscriber(self.mag_topic, String, self.magCB, queue_size=1)

        # Publishers
        self.notify_pub = rospy.Publisher(self.notify_topic, String, queue_size=1)
        self.mag_drive_pub = rospy.Publisher(self.mag_drive_topic, Twist, queue_size=1)
        self.init_90_turn_pub = rospy.Publisher(self.init_90_turn_topic, String, queue_size=1)
        self.relocalize_pub = rospy.Publisher(self.relocalize_topic, PoseWithCovarianceStamped, queue_size=1)
        self.finish_mag_pub = rospy.Publisher(self.finish_mag_topic, String, queue_size=1)
        self.check_drive_pub = rospy.Publisher("/waypoint_navi/check_drive", Bool, queue_size=1)

    # Check on the Initiating Status for Magnetic Following
    def init_magCB(self, msg):
        if(msg.data!="0" and msg.data!=self.last_msg):
            print "--------------------------------"
            print "| mag_navi/init :  " + msg.data + " |"
            print "--------------------------------"
            mag_mission = msg.data.split("-")
            if(len(mag_mission)==3):
                direction = mag_mission[0]
                station = mag_mission[1]
                gate = mag_mission[2]
            # Going into the Bay 4-5
            if(direction=="in" and int(station)==4):
                self.tmp_command = [0,2,0,4]  # str-right-str-end
                self.tmp_pose = Point(self.pt_4.x, self.pt_4.y, self.pt_4_heading)
                self.once = True
            elif(direction=="in" and int(station)==5):
                self.tmp_command = [0,3,0,2,0,4]  # str-skip-str-right-str-end
                self.tmp_pose = Point(self.pt_5.x, self.pt_5.y, self.pt_5_heading)
                self.once = True
            # Coming out from Bay 4-5
            elif(direction=="out" and int(station)==4):
                self.command = [0,3,0,1,0,4]  # str-skip-str-left-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
                self.once = True
            elif(direction=="out" and int(station)==5):
                self.command = [0,3,0,1,0,3,0,4]  # str-skip-str-left-str-skip-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
                self.once = True
            # Exiting Store's Entrance
            if(gate=="exit"):
                self.command = [0,4]  # str-end
                self.goal_pose = Point(self.exit_pt.x, self.exit_pt.y, self.exit_pt_heading)
                self.once = True
                self.step_counter = 1
            elif(gate==""):
                self.step_counter = 2
        self.last_msg = msg.data

    # Receiving detected magnetic line data
    def magCB(self, msg):
        self.mag_data = map(int, msg.data.split("/"))

    # Check on the Position Feedback from Lidar Localization
    def poseCB(self, msg):
        now_mag = np.zeros(16)
        position = msg.pose.pose.position
        roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))
        if(self.mag_data!=None and self.command!=[]):
            for i in xrange(len(self.mag_data)):
                if(self.mag_data[i] > 120):
                   now_mag[i] = 1 
                else:
                   now_mag[i] = 0
            or_mag = self.or_array(now_mag, self.last_mag)
            print "or_mag :  " + str(or_mag)
            j = [i for i in or_mag if i==1]
            # Initialize Magnetic Following
            if(self.mag_on==False and len(j)>7 and self.end==False):
                print "-------------------------------------------"
                print "|      Start Magnetic Line Following      |"
                print "-------------------------------------------"
                print "j :  "+ str(j)
                self.mag_on = True
                self.end = False
                self.moving_forward()
                self.notify_pub.publish("0,1")   # Magnetic Following -- ON
                self.once = False
                self.finish_mag_pub.publish("2")    # On Going
                self.check_drive_pub.publish(False)   # Disable Waypoint_Navi
            # Between Initializing and Ending
            elif(self.mag_on==True and self.k<len(self.command) and self.end==False):
                print "step :  "+ str(self.k) +"  -->  "+ str(self.command[self.k])
                print "j :  "+ str(j)
                self.finish_mag_pub.publish("2")    # On Going
                # Moving Forward when Straight Line is Detected
                if(self.command[self.k]==0 and 1<=len(j)<=7):
                    self.notify_pub.publish("0,1")   # Magnetic Following -- ON
                    if(self.step_counter == 1):
                        self.slope_forward()
                    else:
                        self.moving_forward()
                    self.once = True
                    self.init_90_turn_pub.publish("cancel")  # Cancel the turning effect
                    print "----------------------------------------------"
                    print "|               Moving Forward               |"
                    print "----------------------------------------------"
                # Update Command Sequence when Detecting Crossroads / Perpendicular Line
                elif(self.command[self.k]==0 and len(j)>7):
                    if(self.once==True):
                        self.k = self.k + 1
                        self.once = False
                    print "--------------------------------------------------"
                    print "|               Crossroad Detected               |"
                    print "--------------------------------------------------"
                # Starting to Turn LEFT
                elif(self.command[self.k]==1):
                    self.notify_pub.publish("0,0")   # Magnetic Following -- OFF
                    print "--------------------------------------------"
                    print "|               Turning LEFT               |"
                    print "--------------------------------------------"
                    self.init_90_turn_pub.publish("left")
                    if(self.turn_counter>30):   # 25 = 1 sec
                        self.turn_counter = 0
                        self.k = self.k + 1
                    self.turn_counter = self.turn_counter + 1
                # Starting to Turn RIGHT
                elif(self.command[self.k]==2):
                    self.notify_pub.publish("0,0")   # Magnetic Following -- OFF
                    print "---------------------------------------------"
                    print "|               Turning RIGHT               |"
                    print "---------------------------------------------"
                    self.init_90_turn_pub.publish("right")
                    if(self.turn_counter>30):   # 25 = 1 sec
                        self.turn_counter = 0
                        self.k = self.k + 1
                    self.turn_counter = self.turn_counter + 1
                # Moving across Crossroad if yet to Reach Turning Point
                elif(self.command[self.k]==3):
                    self.moving_forward()
                    self.k = self.k + 1
                    print "----------------------------------------------"
                    print "|               Skip Crossroad               |"
                    print "----------------------------------------------"
                # Stop Vehicle after Reaching Final Destination
                elif(self.command[self.k]==4 and self.k==len(self.command)-1):
                    self.mag_on = False
                    self.end = True
                    self.notify_pub.publish("0,0")    # Magnetic Following -- OFF
                    self.stopping()
                    print "---------------------------------------------------"
                    print "|               Final Point Arrived               |"
                    print "---------------------------------------------------"
            # Start to Relocalize Vehicle Position until Success
            elif(self.mag_on==False and self.k==len(self.command)-1 and self.end==True):
                self.notify_pub.publish("0,0")   # Magnetic Following -- OFF
                self.stopping()
                if(self.dist_checker(position, self.goal_pose, 0.3)==False or self.heading_checker(yaw, self.goal_pose.z, 30)==False):
                    self.finish_mag_pub.publish("2")    # On Going
                    print "----------------------------------------"
                    print "|              Relocalize              |"
                    print "----------------------------------------"
                    self.relocalizing(self.goal_pose.x, self.goal_pose.y, self.goal_pose.z)
                else:
                    print "------------------------------------"
                    print "|               DONE               |"
                    print "------------------------------------"
                    if(self.step_counter == 1):
                        self.step_counter = 2
                        self.command = self.tmp_command
                        self.goal_pose = self.tmp_pose
                        self.finish_mag_pub.publish("2")    # On Going
                    elif(self.step_counter == 2):
                        self.finish_mag_pub.publish("1")    # Done
                        self.command = []
                    self.k = 0
                    self.once = False
                    self.mag_on = False
                    self.end = False
                    print self.command
                    self.check_drive_pub.publish(True) # Return Control to Waypoint_Navi
            self.last_mag = now_mag

    # Distance Checker [m]
    def dist_checker(self, a, b, tolerance):
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        if(distance <= tolerance):
            return True
        else:
            return False

    # Heading Checker [deg]
    def heading_checker(self, a, b, tolerance):
        yaw_diff = a - b
        if(yaw_diff >= 180):
            yaw_diff = abs(yaw_diff - 360)
        elif(yaw_diff <= -180):
            yaw_diff = abs(yaw_diff + 360)
        if(yaw_diff <= tolerance):
            return True
        else:
            return False

    # Force-Updating Lidar-Localization
    def relocalizing(self, x_pos, y_pos, heading):
        pt = PoseWithCovarianceStamped()
        pt.header.frame_id = "/map"
        pt.header.stamp = rospy.Time.now()
        pt.pose.pose.position = Point(x_pos, y_pos, 0.0)
        q = tftr.quaternion_from_euler(0.0, 0.0, np.deg2rad(heading))
        pt.pose.pose.orientation = Quaternion(0.0, 0.0, q[2], q[3])
        self.relocalize_pub.publish(pt)

    # Moving Vehicle Forward at 0.3 [m/s]
    def moving_forward(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0.3, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.mag_drive_pub.publish(drive_msg)

    # Moving Vehicle Forward at 0.4 [m/s]
    def slope_forward(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0.4, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.mag_drive_pub.publish(drive_msg)

    # Stopping Vehicle
    def stopping(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.mag_drive_pub.publish(drive_msg)

    # OR logic gate for array
    def or_array(self, a, b):
        c = np.zeros(16)
        if(len(a) == len(b)):
            for i in a:
                if(a[i] == 1 or b[i] == 1):
                    c[i] = 1
                else:
                    c[i] = 0
            return c
        else:
            print "Error.... \"a\" and \"b\" have different dimension"



if __name__=="__main__":
    rospy.init_node("temporary_magnetic_follower_v1_2")
    TemporaryMagneticFollowerV1_2()
    rospy.spin()
