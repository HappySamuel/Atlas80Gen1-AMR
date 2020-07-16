#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry

class WheelOdom():
    def __init__(self):
	self.rpm_sub_topic = rospy.get_param("~speed_sub_topic")
	self.odom_pub_topic = rospy.get_param("~odom_pub_topic")
	self.wheel_diameter = rospy.get_param("~wheel_diameter")
	self.dist_between_wheels = rospy.get_param("~dist_between_wheels")

	self.yaw_init = 0.0
	self.last_time = rospy.Time.now()

	self.rpm_sub = rospy.Subscriber(self.rpm_sub_topic, String, self.rpmCB, queue_size=1)
	self.odom_pub = rospy.Publisher(self.odom_pub_topic, Odometry, queue_size=1)

    def rpmCB(self, msg):
	rpm = msg.data.split(",")
	R_rpm = rpm[0]
	L_rpm = rpm[1]
	self.v_w_calculation(R_rpm, L_rpm)

    def v_w_calculation(self, R_rotation, L_rotation):
	current_time = rospy.Time.now()
	speed_R = R_rotation * self.wheel_diameter/2
	speed_L = L_rotation * self.wheel_diameter/2

	speed_x = (v_R + v_L)/2
	speed_y = 0
	angular_velocity = (v_R - v_L)/self.dist_between_wheels

	dt = current_time - self.last_time

	delta_x = speed_x * np.cos(self.yaw_) * dt
	delta_y = speed_x * np.sin(self.yaw_) * dt
	delta_yaw = angular_velocity * dt

	x_ = x_ + delta_x
	y_ = y_ + delta_y
	self.yaw_ = self.yaw_ + delta_yaw
	self.last_time = current_time
	self.odom_publish(x_, y_, self.yaw_, speed_x, angular_velocity)


    def odom_publish(self, x_pos, y_pos, yaw_angle, current_speed, current_yaw_rate):
	wheel_odom = Odometry()
	wheel_odom.header.frame_id = "odom"
	wheel_odom.header.stamp = rospy.Time.now()
	wheel_odom.child_frame_id = "base_link"
	wheel_odom.pose.pose.position.x = x_pos
	wheel_odom.pose.pose.position.y = y_pos
	wheel_odom.pose.pose.position.z = 0.0
	wheel_odom.pose.pose.orientation.x = 0.0
	wheel_odom.pose.pose.orientation.y = 0.0
	wheel_odom.pose.pose.orientation.z = np.sin(yaw_angle/2.0)
	wheel_odom.pose.pose.orientation.w = np.cos(yaw_angle/2.0)
	wheel_odom.pose.covariance[0] = 0.2
	wheel_odom.pose.covariance[7] = 0.2
	wheel_odom.pose.covariance[35] = 0.4
	wheel_odom.twist.twist.linear.x = current_speed
	wheel_odom.twist.twist.linear.y = 0.0
	wheel_odom.twist.twist.angular.z = current_yaw_rate
	odom_pub.publish(wheel_odom)

if __name__=="__main__":
    rospy.init_node("wheel_odom")
    WheelOdom()
    rospy.spin()
	
