#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, Point, Vector3
from nav_msgs.msg import Odometry

import tf.transformations
import tf
import numpy as np

"""
Author: Samuel Chieng Kien Ho

Combine two topics' informations into one topic informations to be used by MPC Tracking
Input Topic:	/odometry/filtered	---->	40Hz from Robot_Localization (Data Fusion)
Lookup Transfrom:	"map -> odom"

Output Topic:	/odometry/mpc_use	---->	40Hz for mpc use (x, y, theta, vx, vy, vz, ax, ay, az)

Input Format for each function of tf.transformations library:
	Quaternions xi + yj + zk + w represented as [x, y, z, w]
	quaternion_multiply([4,1,-2,3],[8,-5,6,7])
	quaternion_inverse([4,1,-2,3])
"""

class OdomForMpc():
    def __init__(self):
        # Internal Use Variable - Do not modify without consultation
        self.tfTL = tf.TransformListener()

        # Subscriber
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odomCB, queue_size=1)

        # Publisher
        self.map_to_base_pub = rospy.Publisher("/odometry/filtered_on_map", Odometry, queue_size =1 )

        # To print out ready
        rospy.loginfo("Odometry for MPC Tracking initialized")

    def odomCB(self, msg):
        # Get all the informations from "/odometry/filtered"
        pose_position = np.array( (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) )
        pose_orientation = np.array( (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) )
        twist_linear = np.array( (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z) )
        twist_angular = np.array( (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z) )
        self.coordinate_transformation(pose_position, pose_orientation, twist_linear, twist_angular)

    def coordinate_transformation(self, position, orientation, linear, angular):
        try:
            (trans, rot) = self.tfTL.lookupTransform("map", "odom", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        rot_inv = tf.transformations.quaternion_conjugate(rot)
        qp = np.array( (position[0], position[1], position[2], 0) )
        qp = tf.transformations.quaternion_multiply(qp, rot_inv)
        qp = tf.transformations.quaternion_multiply(rot, qp)
        new_position = Point(qp[0] + trans[0], qp[1] + trans[1], qp[2] + trans[2])
        new_orientation = tf.transformations.quaternion_multiply(orientation, rot)
        qv = np.array( (linear[0], linear[1], linear[2], 0) )
        qv = tf.transformations.quaternion_multiply(rot_inv, qv)
        qv = tf.transformations.quaternion_multiply(qv, rot)
        new_linear = Vector3(qv[0], qv[1], qv[2])
        qw = np.array( (angular[0], angular[1], angular[2], 0) )
        qw = tf.transformations.quaternion_multiply(rot_inv, qw)
        qw = tf.transformations.quaternion_multiply(qw, rot)
        new_angular = Vector3(qw[0], qw[1], qw[2])
        self.topic_msg_republish(new_position, new_orientation, new_linear, new_angular)

    def topic_msg_republish(self, position, orientation, linear, angular):
        odom_for_mpc = Odometry()
        odom_for_mpc.header.frame_id = "map"
        odom_for_mpc.header.stamp = rospy.Time.now()
        odom_for_mpc.pose.pose.position = position
        odom_for_mpc.pose.pose.orientation.x = orientation[0]
        odom_for_mpc.pose.pose.orientation.y = orientation[1]
        odom_for_mpc.pose.pose.orientation.z = orientation[2]
        odom_for_mpc.pose.pose.orientation.w = orientation[3]
        odom_for_mpc.twist.twist.linear = linear
        odom_for_mpc.twist.twist.angular = angular
        self.map_to_base_pub.publish(odom_for_mpc)



if __name__=="__main__":
    rospy.init_node("odom_for_mpc")
    OdomForMpc()
    rospy.spin()

