#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' ROS node that computes the car displacement from the encoders count. Publishes
the result as:
- an EncoderOdo custom msg
- an Odometry msg

Parameters:
- wheel_size: the circumference of the wheels
- left_correction_factor: because the 2 wheels may not be of exactly the same size,
  this correction factor is applied to the circumference of the left wheel.
- dist_btw_wheels: the distance between the wheels, used to compute the car rotation
  from the count difference.
- frame_id: the frame ID of the TF transform broadcasted (defaults to 'odom')
- x: the initial value of x (defaults to 0)
- y: the initial value of y (defaults to 0)
- th: the initial value of th (defaults to 0)

Subscribed topics:
- encoder_counts (type EncoderCounts)

Published topics:
- encoder_odo (type EncoderOdo)
- odom (type Odometry)

Tuning method:
- left_correction_factor: drive in a straight line and monitor the reported angular
  position of the car. If theta increases, then decrease the factor (to be verified).
- wheel_size: drive in a straight line, and adjust so that the physical travelled
  distance is equal to the reported travelled distance. See also
  encoders_calibration_node.
- dist_btw_wheels: drive in a circle and adjust so that the reported angular value
  matches.
'''

import roslib; roslib.load_manifest('phidget_encoders')
import rospy
import diagnostic_updater as DIAG
import tf.transformations
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from phidget_encoders.msg import *
from math import sin, cos, pi


class EncodersOdoNode:

    def __init__(self):
        self.distBtwWheels = rospy.get_param('~dist_btw_wheels',0.995)
        self.wheelSize = rospy.get_param('~wheel_size', 1.322)
        self.leftCorrectionFactor = rospy.get_param('~left_correction_factor', 1.011)

        self.frameID = rospy.get_param('~frame_id', 'encoder_odom') #TF frame ID
        rospy.loginfo('publishing tf on frame id ' + self.frameID)

        self.x = rospy.get_param('~x', 0.0)
        self.y = rospy.get_param('~y', 0.0)
        self.th = rospy.get_param('~th', 0.0)
        self._br = tf.TransformBroadcaster()           # define a transformbroadcaster

        self.pub = rospy.Publisher('encoder_odo', Encoders)
        self.odomPub = rospy.Publisher('odom', Odometry)
        self.sub = rospy.Subscriber('encoder_counts', EncoderCounts, self.callback)


    def callback(self, msg):
        dr = msg.d_right * self.wheelSize*1
        dl = msg.d_left * self.wheelSize *-1* self.leftCorrectionFactor

        omsg = Encoders()
        omsg.dt = msg.dt
        omsg.stamp = msg.stamp
        omsg.d_dist = (dl+dr)/2
        omsg.d_th = (dr-dl)/self.distBtwWheels
        if omsg.dt>0:
            omsg.v = omsg.d_dist/omsg.dt
            omsg.w = omsg.d_th/omsg.dt
        self.pub.publish(omsg)


        d_x = cos(omsg.d_th) * omsg.d_dist
        d_y = -sin(omsg.d_th) * omsg.d_dist
        self.x += cos(self.th)*d_x - sin(self.th)*d_y
        self.y += sin(self.th)*d_x + cos(self.th)*d_y
        self.th += omsg.d_th

        rospy.logdebug('pose (x,y,th_deg)=(%.2f, %.2f, %+d), (v,w)=(%.2f, %.2f)' % (self.x, self.y, int(self.th*180.0/pi), omsg.v, omsg.w))

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self._br.sendTransform((self.x, self.y, 0), quaternion, omsg.stamp, "base_link", "encoder_odom")

        odom = Odometry()
        odom.header.stamp = omsg.stamp
        odom.header.frame_id = self.frameID
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = omsg.v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = omsg.w

        # What about the covariance ?

        self.odomPub.publish(odom)


if __name__=='__main__':
    rospy.init_node('encoders_odo_node')
    node = EncodersOdoNode()
    rospy.spin()
