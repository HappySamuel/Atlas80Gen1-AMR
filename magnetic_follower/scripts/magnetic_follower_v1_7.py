#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon

'''
Section 1:                                         Section 2:

                --- (exit pt)                                (out pt)          (pt 1)   ~~~     (pt 5)
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
            (9) Able to "Go into" to Bay 1 to 3 with lidar navigation while use magnetic following at store's entrance only

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

class MagneticFollowerV1_7():
    def __init__(self):
        # Define Adjustable Parameters
        self.pt_1 = Point(rospy.get_param("~pt_1_x"), rospy.get_param("~pt_1_y"), 0.0)
        self.pt_1_heading = rospy.get_param("~pt_1_heading")
        self.pt_2 = Point(rospy.get_param("~pt_2_x"), rospy.get_param("~pt_2_y"), 0.0)
        self.pt_2_heading = rospy.get_param("~pt_2_heading")
        self.pt_3 = Point(rospy.get_param("~pt_3_x"), rospy.get_param("~pt_3_y"), 0.0)
        self.pt_3_heading = rospy.get_param("~pt_3_heading")
        self.pt_4 = Point(rospy.get_param("~pt_4_x"), rospy.get_param("~pt_4_y"), 0.0)
        self.pt_4_heading = rospy.get_param("~pt_4_heading")
        self.pt_5 = Point(rospy.get_param("~pt_5_x"), rospy.get_param("~pt_5_y"), 0.0)
        self.pt_5_heading = rospy.get_param("~pt_5_heading")
        self.out_pt = Point(rospy.get_param("~out_pt_x"), rospy.get_param("~out_pt_y"), 0.0)
        self.out_pt_heading = rospy.get_param("~out_pt_heading")
        self.exit_pt = Point(rospy.get_param("~exit_pt_x"), rospy.get_param("~exit_pt_y"), 0.0)
        self.exit_pt_heading = rospy.get_param("~exit_pt_heading")
        self.bay_polygon = ShapelyPolygon([(rospy.get_param("~bay_1_x"), rospy.get_param("~bay_1_y")), (rospy.get_param("~bay_2_x"), rospy.get_param("~bay_2_y")), (rospy.get_param("~bay_3_x"), rospy.get_param("~bay_3_y")), (rospy.get_param("~bay_4_x"), rospy.get_param("~bay_4_y"))])    # Only available in 2D

        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(30)    # 30 [hz] <--> 0.033 [sec]
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
        self.position = Point()
        self.yaw = 0
        self.direction = ""
        self.iters = 0
        self.abort_iters = 0

        # Publishers
        self.notify_pub = rospy.Publisher(rospy.get_param("~notify_topic"), String, queue_size=1)
        self.drive_pub = rospy.Publisher(rospy.get_param("~drive_topic"), Twist, queue_size=1)
        self.init_90_turn_pub = rospy.Publisher(rospy.get_param("~init_90_turn_topic"), String, queue_size=1)
        self.relocalize_pub = rospy.Publisher(rospy.get_param("~relocalize_topic"), PoseWithCovarianceStamped, queue_size=1)
        self.finish_mag_pub = rospy.Publisher(rospy.get_param("~finish_mag_topic"), String, queue_size=1)
        self.check_drive_pub = rospy.Publisher(rospy.get_param("~check_drive_topic"), Bool, queue_size=1)
        self.debug_pub = rospy.Publisher("/mag_navi/debug", String, queue_size=1)

        # Subscribers
        self.init_mag_sub = rospy.Subscriber(rospy.get_param("~init_mag_topic"), String, self.init_magCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(rospy.get_param("~pose_topic"), Odometry, self.poseCB, queue_size=1)
        self.mag_sub = rospy.Subscriber(rospy.get_param("~mag_topic"), String, self.magCB, queue_size=1)

        self.mag_loop()

    # Check on the Initiating Status for Magnetic Following
    def init_magCB(self, msg):
        if(msg.data!="0" and msg.data!=self.last_msg):
            self.k = 0
            print "--------------------------------"
            print "| mag_navi/init :  " + msg.data + " |"
            print "--------------------------------"
            mag_mission = msg.data.split("-")
            if(len(mag_mission)==3):
                self.direction = mag_mission[0]
                station = mag_mission[1]
                gate = mag_mission[2]
            # Going into the Bay 1-5
            if(self.direction=="in" and int(station)==1):
                self.tmp_command = [0,3,0,3,0,3,0,4]  # [0,3,0,4] str-skip-str-end
                self.tmp_pose = Point(self.pt_1.x, self.pt_1.y, self.pt_1_heading)
            elif(self.direction=="in" and int(station)==2):
                self.tmp_command = [0,1,0,2,0,4]  # str-left-str-right-str-end
                self.tmp_pose = Point(self.pt_2.x, self.pt_2.y, self.pt_2_heading)
            elif(self.direction=="in" and int(station)==3):
                self.tmp_command = [0,1,0,3,0,2,0,4]  # str-left-str-skip-str-right-str-end
                self.tmp_pose = Point(self.pt_3.x, self.pt_3.y, self.pt_3_heading)
            elif(self.direction=="in" and int(station)==4):
                self.tmp_command = [0,1,0,3,0,3,0,2,0,4]  # str-left-str-skip-skip-str-right-str-end
                self.tmp_pose = Point(self.pt_4.x, self.pt_4.y, self.pt_4_heading)
            elif(self.direction=="in" and int(station)==5):
                self.tmp_command = [0,1,0,3,0,3,0,3,0,2,0,4]  # str-left-str-skip-str-skip-str-skip-str-right-str-end
                self.tmp_pose = Point(self.pt_5.x, self.pt_5.y, self.pt_5_heading)
            # Coming out from Bay 1-5
            elif(self.direction=="out" and int(station)==1):
                self.command = [0,3,0,3,0,4]  # str-skip-str-skip-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
            elif(self.direction=="out" and int(station)==2):
                self.command = [0,3,0,1,0,2,0,4]  # str-skip-str-left-str-right-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
            elif(self.direction=="out" and int(station)==3):
                self.command = [0,3,0,1,0,3,0,2,0,4]  # str-skip-str-left-str-skip-str-right-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
            elif(self.direction=="out" and int(station)==4):
                self.command = [0,3,0,1,0,3,0,3,0,2,0,4]  # str-skip-str-left-str-skip-str-skip-str-right-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
            elif(self.direction=="out" and int(station)==5):
                self.command = [0,3,0,1,0,3,0,3,0,3,0,2,0,4]  # str-skip-str-left-str-skip-str-skip-str-skip-str-right-str-end
                self.goal_pose = Point(self.out_pt.x, self.out_pt.y, self.out_pt_heading)
            # Exiting Store's Entrance
            if(gate=="exit"):
                self.command = [0,4]  # str-end
                self.goal_pose = Point(self.exit_pt.x, self.exit_pt.y, self.exit_pt_heading)
        elif(msg.data=="0"):
            self.command = []
            self.goal_pose = Point()
        elif(msg.data=="cancel"):
            self.command = []
            self.goal_pose = Point()
        self.last_msg = msg.data

    # Receiving detected magnetic line data
    def magCB(self, msg):
        self.mag_data = map(int, msg.data.split("/"))

    # Check on the Position Feedback from Lidar Localization
    def poseCB(self, msg):
        self.position = msg.pose.pose.position
        roll, pitch, self.yaw = np.rad2deg(tftr.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))

    # Main Loop - Running at 30 Hz
    def mag_loop(self):
        while not rospy.is_shutdown():
            self.mag_follower()
            if(self.iters%15==0):    # Publishing at 2 Hz
                self.debug_msg()
            self.iters += 1
            self.refresh_rate.sleep()

    # Magnetic Following Algorithm
    def mag_follower(self):
        current_position = ShapelyPoint(self.position.x, self.position.y)   # only availble in 2D
#        print "Inside Bay Area :  " + str(self.bay_polygon.contains(current_position))              # debug USE
        # Checking whether inside the Bay Area
        if(self.bay_polygon.contains(current_position) and self.direction=="in"): # Do not include the magnetic line tape area
            self.command = self.tmp_command
            self.goal_pose = self.tmp_pose
            self.k = 0
            print "Going into Bay Area"
        # Main Run
        if(self.mag_data!=None and self.command!=[]):
            j = [i for i in self.mag_data if i>=130]
            # Initialize Magnetic Following
            if(not self.mag_on and len(j)>7 and not self.end):
                print "-------------------------------------------"
                print "|      Start Magnetic Line Following      |"
                print "-------------------------------------------"
                print "j :  "+ str(j)
                print self.command
                self.mag_on = True
                self.end = False
                self.moving_forward()
                self.notify_pub.publish("0,1")   # Magnetic Following -- ON
                self.once = False
                self.finish_mag_pub.publish("2")    # On Going
                self.check_drive_pub.publish(False)   # Disable Waypoint_Navi
                self.abort_iters = 0
            # Between Initializing and Ending
            elif(self.mag_on and self.k<len(self.command) and not self.end):
                print "step :  "+ str(self.k) +"  -->  "+ str(self.command[self.k])
                print "j :  "+ str(j)
                self.finish_mag_pub.publish("2")    # On Going
                # Moving Forward when Straight Line is Detected
                if(self.command[self.k]==0 and 1<=len(j)<8):
                    self.notify_pub.publish("0,1")   # Magnetic Following -- ON
                    self.moving_forward()
                    self.once = True
                    self.init_90_turn_pub.publish("cancel")  # Cancel the turning effect
                    self.check_drive_pub.publish(False)   # Disable Waypoint_Navi
                    print "----------------------------------------------"
                    print "|               Moving Forward               |"
                    print "----------------------------------------------"
                # Update Command Sequence when Detecting Crossroads / Perpendicular Line
                elif(self.command[self.k]==0 and len(j)>=12):
                    if(self.once):
                        self.k += 1
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
                    if(self.turn_counter>60):   # 15 = 1 sec
                        self.turn_counter = 0
                        self.k += 1
                    self.turn_counter = self.turn_counter + 1
                # Starting to Turn RIGHT
                elif(self.command[self.k]==2):
                    self.notify_pub.publish("0,0")   # Magnetic Following -- OFF
                    print "---------------------------------------------"
                    print "|               Turning RIGHT               |"
                    print "---------------------------------------------"
                    self.init_90_turn_pub.publish("right")
                    if(self.turn_counter>60):   # 15 = 1 sec
                        self.turn_counter = 0
                        self.k += 1
                    self.turn_counter = self.turn_counter + 1
                # Moving across Crossroad if yet to Reach Turning Point
                elif(self.command[self.k]==3):
                    self.moving_forward()
                    self.k += 1
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
                # Shutdown magnetic following mode and revert back to lidar based waypt_navi
                elif(self.command[self.k]==0 and len(j)==0 and len(self.command)<=2):  # Only use on slope
                    if(self.abort_iters >= 60):
                        self.mag_on = False
                        self.end = False
                        self.command = []
                        self.notify_pub.publish("0,0")    # Magnetic Following -- OFF
                        self.check_drive_pub.publish(True)
                        print "------------------------------------------------------------------"
                        print "|     Magnetic Line Not Detected, Shutdown Magnetic Follower     |"
                        print "------------------------------------------------------------------"
                    self.abort_iters += 1
            # Start to Relocalize Vehicle Position until Success
            elif(not self.mag_on and self.k==len(self.command)-1 and self.end):
                self.notify_pub.publish("0,0")   # Magnetic Following -- OFF
                self.stopping()
                if(not self.dist_checker(self.position, self.goal_pose, 0.4) or not self.heading_checker(self.yaw, self.goal_pose.z, 30)):
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
        else:
            self.finish_mag_pub.publish("1")    # Done

    # Combining debug details and publishing out
    def debug_msg(self):
        if(self.command!=[]):
            mission_msg = "current_mission :  "+str(self.command)
            stage_msg = "current_stage :  "+str(self.k)
            total_msg = mission_msg +" , "+stage_msg
            self.debug_pub.publish(total_msg)

    # Distance Checker [m]
    def dist_checker(self, a, b, tolerance):
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        return True if(distance <= tolerance) else False

    # Heading Checker [deg]
    def heading_checker(self, a, b, tolerance):
        yaw_diff = a - b
        if(yaw_diff >= 180):
            yaw_diff = abs(yaw_diff - 360)
        elif(yaw_diff <= -180):
            yaw_diff = abs(yaw_diff + 360)
        return True if(yaw_diff <= tolerance) else False

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
        self.drive_pub.publish(drive_msg)

    # Stopping Vehicle
    def stopping(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(drive_msg)



if __name__=="__main__":
    rospy.init_node("magnetic_follower_v1_7")
    MagneticFollowerV1_7()
    rospy.spin()
