#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tftr

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

'''

                   (pt_4)           (pt_5)
 (Mag Navi) ----------------------------
                      |   (Mag Navi)   |
                     ---              ---
                      |                |
                      |  (Table Navi)  |
'''


class MagneticFollower():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.init_mag_topic = rospy.get_param("~init_mag_topic")
        self.notify_topic = rospy.get_param("~notify_topic")
        self.mag_topic = rospy.get_param("~mag_topic")
        self.mag_drive_topic = rospy.get_param("~mag_drive_topic")
        self.init_90_turn_topic = rospy.get_param("~init_90_turn_topic")
        self.finish_90_turn_topic = rospy.get_param("~finsih_90_turn_topic")
        self.relocalize_topic = rospy.get_param("~relocalize_topic")
        self.finish_mag_topic = rospy.get_param("~finish_mag_topic")
        self.pose_topic = rospy.get_param("~pose_topic")

        # Define Adjustable Parameters
        self.pt_4 = Point(rospy.get_param("~pt_4_x"), rospy.get_param("~pt_4_y"), 0.0)
        self.pt_4_heading = rospy.get_param("~pt_4_heading")
        self.pt_5 = Point(rospy.get_param("~pt_5_x"), rospy.get_param("~pt_5_y"), 0.0)
        self.pt_5_heading = rospy.get_param("~pt_5_heading")

        # Internal Use Variables - Do not modify without consultation
        self.running_rate = rospy.Rate(10)
        self.mag_data = None
        self.mag_on = False
        self.finish_90_turn = False
        self.do_once = False
        self.stop_times = []   # 0 <--> continue moving forward | 1 <--> do something after detecting parallel line
        self.goal_pose = Point()   # x = x-pos | y = y-pos | z = heading    (Special Case)
        self.position = Point()    # x = x-pos | y = y-pos | z = z_pos

        # Subscribers
        self.init_mag_sub = rospy.Subscriber(self.init_mag_topic, String, self.init_magCB, queue_size=1)
        self.finish_90_turn_sub = rospy.Subscriber(self.finish_90_turn_topic, String, self.finish_90_turnCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.poseCB, queue_size=1)
        self.mag_sub = rospy.Subscriber(self.mag_topic, String, self.magCB, queue_size=1)

        # Publishers
        self.notify_pub = rospy.Publisher(self.notify_topic, String, queue_size=1)
        self.mag_drive_pub = rospy.Publisher(self.mag_drive_topic, Twist, queue_size=1)
        self.init_90_turn_pub = rospy.Publisher(self.init_90_turn_topic, String, queue_size=1)
        self.relocalize_pub = rospy.Publisher(self.relocalize_topic, PoseWithCovarianceStamped, queue_size=1)
        self.finish_mag_pub = rospy.Publisher(self.finish_mag_topic, String, queue_size=1)

        # Initialize
        self.finish_mag_pub.publish("0")    # OFF
        self.notify_pub.publish("0,0")    # Magnetic Following -- OFF
        # Run mag_drive
        self.mag_drive()

    # Check on the Initiating Status for Magnetic Following
    def init_magCB(self, msg):
        self.stop_times = []
        if(int(msg.data) == 4):
            self.stop_times = [0,1,0]
            self.goal_pose = Point(self.pt_4.x, self.pt_4.y, self.pt_4_heading)
            self.mag_on = True
            self.do_once = True
        elif(int(msg.data) == 5):
            self.stop_times = [0,0,1,0]
            self.goal_pose = Point(self.pt_5.x, self.pt_5.y, self.pt_5_heading)
            self.mag_on = True
            self.do_once = True
        else:
            self.mag_on = False
            self.do_once = False
            self.stop_times = []

    # Check on the Turning 90' Status
    def finish_90_turnCB(self, msg):
        if(int(msg.data) == 2):
            self.finish_90_turn = True    # Done Turning
        elif(int(msg.data) == 1):
            self.finish_90_turn = False   # On Going

    def poseCB(self, msg):
        self.position = msg.pose.pose.position

    # Receiving detected magnetic line data
    def magCB(self, msg):
        self.mag_data = map(int, msg.data.split("/"))

    # Magnetic-Guided Navi
    def mag_drive(self):
        k = 0
        turning  = False
        turn_counter = 0
        while not rospy.is_shutdown():
            print "mag_on :  " + str(self.mag_on)
            print "finish_90_turn :  " + str(self.finish_90_turn)
            print "sequence :  " + str(self.stop_times)
            if(self.mag_on == True and k < len(self.stop_times)):
                j = [i for i in self.mag_data if i > 130]
                print "j :  " + str(j)
                print "step :  " + str(self.stop_times[k])
                # continue moving forward when straight line is detected
                if(self.stop_times[k] == 0 and 1 <= len(j) <= 8):
                    self.notify_pub.publish("0,1")    # Magnetic Following -- ON
                    self.moving_forward()
                    self.do_once = True
                    self.finish_mag_pub.publish("2")    # On Going
                # stop vehicle at the last point of line following
                elif(self.stop_times[k] == 1 and len(j) > 5 and self.finish_90_turn == False):
                    turning = True
                # after finish 90' turning, enter last stage of magnetic line following
                elif(self.stop_times[k] == 1 and self.finish_90_turn == True):
                    k = k + 1
                    self.finish_mag_pub.publish("2")    # On Going
                # continue moving at the crossroad if haven't reach the last point
                elif(self.stop_times[k] == 0 and len(j) > 8 and k != len(self.stop_times)-1):
                    if(self.do_once == True):
                        k = k + 1
                        k = np.minimum(k, len(self.stop_times)-1)
                        self.do_once = False
                        self.moving_forward()
                    self.finish_mag_pub.publish("2")    # On Going
                # stop vehicle after reached final destination
                elif(self.stop_times[k] == 0 and len(j) > 8 and k == len(self.stop_times)-1):
                    self.mag_on = False
                    self.finish_mag_pub.publish("2")    # On Going
            # Start to relocalize vehicle position until success
            elif(self.finish_90_turn == True and self.mag_on == False and k == len(self.stop_times)-1):
                self.notify_pub.publish("0,0")    # Magnetic Following -- OFF
                if(self.dist_checker(self.position, self.goal_pose, 0.3) == False):
                    print "--------------- relocalize ---------------"
                    self.relocalizing(self.goal_pose.x, self.goal_pose.y, self.goal_pose.z)
                    self.finish_mag_pub.publish("2")    # On Going
                else:
                    print "--------------- Done ---------------"
                    self.finish_mag_pub.publish("1")    # Done
                    self.finish_90_turn = False
                    self.stop_times = []
                    k = 0
                    self.mag_on = False
            # Special for turning
            print "turning :  " + str(turning)
            if(turning == True):
                self.notify_pub.publish("0,0")    # Magnetic Following OFF
                print "---------- starting to turn ----------"
                self.init_90_turn_pub.publish("right")   # For PSA Use
                if(turn_counter > 2):
                    turning  = False
                    turn_counter = 0
                turn_counter = turn_counter + 1
                self.finish_mag_pub.publish("2")    # On Going
            self.running_rate.sleep()

    # Distance Checker [m]
    def dist_checker(self, a, b, tolerance):
        print a
        print b
        distance = np.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        print "dist_checker :  " + str(distance)
        if(distance <= tolerance):
            return True
        else:
            return False

    # Force-Updating Lidar-Localization
    def relocalizing(self, x_pos, y_pos, heading):
        pt = PoseWithCovarianceStamped()
        pt.header.frame_id = "/map"
        pt.header.stamp = rospy.Time.now()
        pt.pose.pose.position = Point(x_pos, y_pos, 0.0)
        quat = tftr.quaternion_from_euler(0.0, 0.0, np.deg2rad(heading))
        pt.pose.pose.orientation = Quaternion(0.0, 0.0, quat[2], quat[3])
        self.relocalize_pub.publish(pt)

    # Moving Vehicle Forward at 0.2 [m/s]
    def moving_forward(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0.3, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.mag_drive_pub.publish(drive_msg)



if __name__=="__main__":
    rospy.init_node("magnetic_follower")
    MagneticFollower()
    rospy.spin()
