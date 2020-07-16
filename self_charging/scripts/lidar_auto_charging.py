#!/usr/bin/env python

import rospy
import numpy as np
import tf
import time

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point, PolygonStamped
from nav_msgs.msg import Odometry
from utils import LineTrajectory


class LidarAutoCharging():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.init_charging_topic = rospy.get_param("~init_charging_topic")
        self.battery_topic = rospy.get_param("~battery_topic")
        self.pose_topic = rospy.get_param("~pose_topic")
        self.drive_topic = rospy.get_param("~drive_topic")
        self.contactor_topic = rospy.get_param("~contactor_topic")
        self.stop_charging_topic = rospy.get_param("~stop_charging_topic")
        self.finish_charging_topic = rospy.get_param("~finish_charging_topic")
        self.suspend_topic = rospy.get_param("~suspend_topic")

        # Define Adjustable Parameters
        self.stop_pt = Point(float(rospy.get_param("~stop_pt_x")), float(rospy.get_param("~stop_pt_y")), 0)
        self.check_pt = Point(float(rospy.get_param("~check_pt_x")), float(rospy.get_param("~check_pt_y")), 0)
        self.finish_pt = Point(float(rospy.get_param("~finish_pt_x")), float(rospy.get_param("~finish_pt_y")), 0)
        self.heading_min = float(rospy.get_param("~heading_min"))
        self.heading_max = float(rospy.get_param("~heading_max"))
        self.charging_route = rospy.get_param("~charging_route")

        # Internal Use Variables - Do not modify without consultation
        self.trajectory = LineTrajectory("/charging_trajectory")
        self.init = False
        self.step_counter = 0
        self.first_time = True
        self.contactor_counter = 0
        self.finish_charging_counter = 0

        # Subscribers
        self.init_charging_sub = rospy.Subscriber(self.init_charging_topic, Bool, self.init_chargingCB, queue_size=1)
        self.battery_sub = rospy.Subscriber(self.battery_topic, String, self.batteryCB, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.poseCB, queue_size=1)

        # Publishers
        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)
        self.contactor_pub = rospy.Publisher(self.contactor_topic, Bool, queue_size=1)
        self.stop_charging_pub = rospy.Publisher(self.stop_charging_topic, Bool, queue_size=1)
        self.finish_charging_pub = rospy.Publisher(self.finish_charging_topic, Bool, queue_size=1)
        self.traj_pub = rospy.Publisher("/charging_navi/trajectory", PolygonStamped, queue_size=1)
        self.suspend_pub = rospy.Publisher(self.suspend_topic, Bool, queue_size=1)

    def init_chargingCB(self, msg):
        self.init = msg.data

    def batteryCB(self, msg):
        self.voltage_percent = float(msg.data)

    def poseCB(self, msg):
        print "step_counter :  " + str(self.step_counter)
        now_position = msg.pose.pose.position
        now_heading = self.quaternion_to_angle(msg.pose.pose.orientation)
        # Initiate self charging procedure
        if(self.init == True and self.first_time == True):
            self.step_counter = 1
            print "Step 0 - Load Trajectory"
            self.trajectory.load(self.charging_route)
            time.sleep(0.5)
            self.traj_pub.publish(self.trajectory.toPolygon())
            self.first_time = False
            self.stop_charging_pub.publish(False)
        # Step 1 - When arriving check point, disable the charging navi and start to tune heading
        if(self.step_counter == 1 and self.dist_checker(now_position, self.check_pt, 0.25) == True):
            print "Step 1 - Tune heading"
            print np.rad2deg(now_heading)
            self.init = False
            if(now_heading > np.deg2rad(self.heading_max) or now_heading < np.deg2rad(self.heading_min)):
                self.step_counter = 2
            else:
                self.tuning_heading(0.3)
                self.stop_charging_pub.publish(True)
            self.finish_charging_pub.publish(False)
        # Step 2 - Continue enable Charging Navi after the checking point
        elif(self.step_counter == 2 and self.dist_checker(now_position, self.stop_pt, 0.1) == False):
            print "Step 2 - Continue enable Charging Navi"
            self.stop_charging_pub.publish(False)
            self.step_counter = 3
        # Step 3 - Stop Vehicle at the Charging Point
        elif(self.step_counter == 3 and self.dist_checker(now_position, self.stop_pt, 0.1) == True):
            print "Step 3 - Stop Vehicle at Charging Point"
            self.stopping()
            self.stop_charging_pub.publish(True)
            self.step_counter = 4
        # Step 4 - Turn On the Contactor upon arrival of the Charging Point
        elif(self.step_counter == 4 and self.voltage_percent < 80.0):
            print "Step 4 - Turn ON Contactor"
            self.contactor_pub.publish(True)
#            self.suspend_pub.publish(True)
            if(self.contactor_counter >= 60):
                self.step_counter = 5
                self.contactor_counter = 0
            self.contactor_counter = self.contactor_counter + 1
        # Step 5 - Turn Off the Contactor after finish charging
        elif(self.step_counter == 5 and self.voltage_percent >= 80.0):
            print "Step 5 - Turn OFF Contactor"
            if(self.finish_charging_counter >= 3000):
                self.contactor_pub.publish(False)
                self.step_counter = 6
                self.finish_charging_counter = 0
            self.finish_charging_counter = self.finish_charging_counter + 1
        # Step 6 - Enable Charging Navi to navigate till Finish Location
        elif(self.step_counter == 6):
            print "Step 6 - Navi to Finish Location"
            self.stop_charging_pub.publish(False)
            self.step_counter = 7
        # Step 7 - Report finish self_charging procedure, Reset counter
        elif(self.step_counter == 7 and self.dist_checker(now_position, self.finish_pt, 1.0) == True):
            print "Step 7 - Finish self_charging"
            self.finish_charging_pub.publish(True)
            self.step_counter = 0
            self.first_time = True
            self.contactor_counter = 0
            self.finish_charging_counter = 0

    # Distance Checker, Checking whether 2 points are within distance tolerance or not
    def dist_checker(self, a, b, dist_tolerance):
        distance = np.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
        if(distance <= dist_tolerance):
            return True
        else:
            return False

    # Tuning the Heading of Vehicle
    def tuning_heading(self, rotspeed):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, rotspeed)
        self.drive_pub.publish(drive_msg)

    # Stopping Vehicle
    def stopping(self):
        drive_msg = Twist()
        drive_msg.linear = Vector3(0, 0, 0)
        drive_msg.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(drive_msg)

    # Convert Quaternion to Angle
    def quaternion_to_angle(self, q):
	x, y, z, w = q.x, q.y, q.z, q.w
	roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
	return yaw



if __name__=="__main__":
    rospy.init_node("lidar_auto_charging")
    LidarAutoCharging()
    rospy.spin()
