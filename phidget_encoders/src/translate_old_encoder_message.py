#!/usr/bin/env python
# -*- coding: utf-8 -*-


import roslib; roslib.load_manifest('phidget_encoders')
import rospy
from phidget_encoders.msg import *

rospy.init_node('translate_old_encoder_message')

pub = rospy.Publisher('encoder_counts', EncoderCounts)

def callback(msg):
    newmsg = EncoderCounts()
    newmsg.stamp = msg.stamp
    newmsg.dt = msg.dt
    newmsg.d_left = msg.d_count_left / 6000.0
    newmsg.d_right = msg.d_count_right / 6000.0
    pub.publish(newmsg)

sub = rospy.Subscriber('encoders', Encoders, callback)

rospy.spin()
