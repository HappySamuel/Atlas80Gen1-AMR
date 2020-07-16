#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  (1) Remotely restore the lidar localization at defined restore point, when vehicle is Lidar-LOST
               (2) Suspend Vehicle right after restore lidar localization, until operator resume it.

   Notes: 
          buttons[5] <--> deadman button (RB)
          buttons[0] <--> recover pt 1 (green A)
          buttons[1] <--> recover pt 2 (red B)
          buttons[2] <--> recover pt 3 (blue X)
'''

import rospy
import numpy as np
import tf.transformations as tftr
import yaml

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


class RemoteRestore():
    def __init__(self):
        # Define Adjustable Parameters
        self.restoration_pts_file_path = "/home/atlas80-b/catkin_ws/src/atlas80/config/restoration_point.yaml"

        # Internal USE Variables
        self.relocalize_pt_1 = Point()
        self.relocalize_pt_2 = Point()
        self.relocalize_pt_3 = Point()
        self.iters = 0

        # Publisher
        self.relocalize_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.kill_mag_mission_pub = rospy.Publisher("/atlas80/cancel_mag", String, queue_size=1)
        self.check_drive_pub = rospy.Publisher("/waypoint_navi/check_drive", Bool, queue_size=1)
        self.suspend_pub = rospy.Publisher("suspend/input/navi", Bool, queue_size=1)

        # Subcriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

    # Status Feedback of Joystick Controller
    def joyCB(self, msg):
        if(self.iters % 20 == 0):
            self.load_params_from_yaml(self.restoration_pts_file_path)
        if(msg.buttons[0] == 1 and msg.buttons[5] == 1):          # Green Button
            self.relocalizing(self.relocalize_pt_1.x, self.relocalize_pt_1.y, self.relocalize_pt_1.z)
            self.kill_mag_mission_pub.publish("restore")
            self.suspend_pub.publish(True)
        elif(msg.buttons[1] == 1 and msg.buttons[5] == 1):        # Red Button
            self.relocalizing(self.relocalize_pt_2.x, self.relocalize_pt_2.y, self.relocalize_pt_2.z)
            self.kill_mag_mission_pub.publish("restore")
            self.suspend_pub.publish(True)
        elif(msg.buttons[2] == 1 and msg.buttons[5] == 1):        # Blue Button
            self.relocalizing(self.relocalize_pt_3.x, self.relocalize_pt_3.y, self.relocalize_pt_3.z)
            self.kill_mag_mission_pub.publish("cancel")
            self.check_drive_pub.publish(True)
            self.suspend_pub.publish(True)
        self.iters += 1

    # Force-Updating Lidar-Localization
    def relocalizing(self, x_pos, y_pos, heading):
        pt = PoseWithCovarianceStamped()
        pt.header.frame_id = "/map"
        pt.header.stamp = rospy.Time.now()
        pt.pose.pose.position = Point(x_pos, y_pos, 0.0)
        q = tftr.quaternion_from_euler(0.0, 0.0, np.deg2rad(heading))
        pt.pose.pose.orientation = Quaternion(0.0, 0.0, q[2], q[3])
        self.relocalize_pub.publish(pt)

    # Loading parameters from yaml file
    def load_params_from_yaml(self, filepath):
        with open(filepath, 'r') as infile:
            data = yaml.load(infile)
            for param in data:
                self.relocalize_pt_1 = Point(data["relocalize_pt_1"]["x"], data["relocalize_pt_1"]["y"], data["relocalize_pt_1"]["heading"])
                self.relocalize_pt_2 = Point(data["relocalize_pt_2"]["x"], data["relocalize_pt_2"]["y"], data["relocalize_pt_2"]["heading"])
                self.relocalize_pt_3 = Point(data["relocalize_pt_3"]["x"], data["relocalize_pt_3"]["y"], data["relocalize_pt_3"]["heading"])



if __name__=="__main__":
    rospy.init_node("remote_restore")
    RemoteRestore()
    rospy.spin()

