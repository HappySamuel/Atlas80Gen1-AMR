#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

'''
                        (pt 4)           (pt 5)
 (Mag Navi)   |------------|----------------|-
                           |   (Mag Navi)   |
                          ---              ---

                          ... (Table Navi) ...
                          | |              | |
                          ```              ```

Features :  (1) Enable "Go into" and "Come out" of Bay 4 & 5 by using magnetic following
            (2) Detecting Horizontal Magnetic Line as the signal to begin magnetic following
            (3) Ability to decide whether to skip crossroad
            (4) Choosing route according to special sequence of commands
            (5) Able to do "Auto-Relocalizing" the Lidar Localization until correct
            (6) Able to trigger LEFT or RIGHT 90' turn
            (7) Use nearly 30[Hz] to run (based on poseCB)

Notes:
command = [0, 1, 2, 3]
0 = go straight
1 = turning point
2 = skip crossroad
3 = end point
'''


class MagneticFollowerV1_3():
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

        # Internal Use Variables - Do not modify without consultation
        self.mag_data = None
        self.k = 0
        self.command = []
        self.mag_on = False   # use only between starting and ending
        self.end = False    # use to identify whether currently is starting / ending
        self.once = False    # use to make sure the crossroad detected repeatedly won't be counted
        self.last_msg = ""
        self.goal_pose = Point()
        self.turn_counter = 0
#        self.turning = False
        self.turn = ""

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
                self.turn = mag_mission[2]
            if(direction=="in" and int(station)==4 and self.turn=="right"):
                self.command = [0,1,0,3]
                self.goal_pose = Point(self.pt_4.x, self.pt_4.y, self.pt_4_heading)
                self.once = True
            elif(direction=="in" and int(station)==5 and self.turn=="right"):
                self.command = [0,2,0,1,0,3]
                self.goal_pose = Point(self.pt_5.x, self.pt_5.y, self.pt_5_heading)
                self.once = True
            elif(direction=="out" and int(station)==4 and self.turn=="left"):
                self.command = [0,2,0,1,0,3]
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
                self.once = True
            elif(direction=="out" and int(station)==5 and self.turn=="left"):
                self.command = [0,2,0,1,0,2,0,3]
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
                self.once = True
        self.last_msg = msg.data

    # Receiving detected magnetic line data
    def magCB(self, msg):
        self.mag_data = map(int, msg.data.split("/"))

    # Check on the Position Feedback from Lidar Localization
    def poseCB(self, msg):
        position = msg.pose.pose.position
        roll, pitch, yaw = np.rad2deg(tftr.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))
        if(self.mag_data!=None and self.command!=[]):
            j = [i for i in self.mag_data if i>120]
            # Initialize Magnetic Following
            if(self.mag_on==False and len(j)>6 and self.end==False):
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
                # Starting to Turn
                elif(self.command[self.k]==1):
                    self.notify_pub.publish("0,0")   # Magnetic Following -- OFF
                    print "------------------------------------------------"
                    print "|               Starting to TURN               |"
                    print "------------------------------------------------"
                    if(self.turn=="left"):
                        self.init_90_turn_pub.publish("left")  # For PSA Use
                    elif(self.turn=="right"):
                        self.init_90_turn_pub.publish("right") # For PSA Use
                    if(self.turn_counter>50):   # 25 = 1 sec
#                        self.turning = False
                        self.turn_counter = 0
                        self.k = self.k + 1
                    self.turn_counter = self.turn_counter + 1
                # Moving across Crossroad if yet to Reach Turning Point
                elif(self.command[self.k]==2):
                    self.k = self.k + 1
                    self.moving_forward()
                    print "----------------------------------------------"
                    print "|               Skip Crossroad               |"
                    print "----------------------------------------------"
                # Stop Vehicle after Reaching Final Destination
                elif(self.command[self.k]==3 and self.k==len(self.command)-1):
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
                if(self.dist_checker(position, self.goal_pose, 0.3)==False or self.heading_checker(yaw, self.goal_pose.z, 20)==False):
                    self.finish_mag_pub.publish("2")    # On Going
                    print "----------------------------------------"
                    print "|              Relocalize              |"
                    print "----------------------------------------"
                    self.relocalizing(self.goal_pose.x, self.goal_pose.y, self.goal_pose.z)
                else:
                    print "------------------------------------"
                    print "|               DONE               |"
                    print "------------------------------------"
                    self.finish_mag_pub.publish("1")    # Done
                    self.command = []
                    self.k = 0
                    self.once = False
                    self.mag_on = False
                    self.end = False
                    self.check_drive_pub.publish(True) # Return Control to Waypoint_Navi

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

    # Moving Vehicle Forward at 0.2 [m/s]
    def moving_forward(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0.3, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.mag_drive_pub.publish(drive_msg)

    # Stopping Vehicle
    def stopping(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.mag_drive_pub.publish(drive_msg)



if __name__=="__main__":
    rospy.init_node("magnetic_follower_v1_3")
    MagneticFollowerV1_3()
    rospy.spin()
