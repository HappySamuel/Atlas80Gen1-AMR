#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class InterpolateThrottle:
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.motor_in_topic   = rospy.get_param("~motor_in_topic")
        self.motor_out_topic  = rospy.get_param("~motor_out_topic")

        # Define Adjustable Parameters
        self.max_acceleration = rospy.get_param("~max_acceleration")
        self.max_speed = rospy.get_param("~max_speed")
        self.min_speed = rospy.get_param("~min_speed")
        self.throttle_smoother_rate = rospy.get_param("~throttle_smoother_rate")
        self.max_turning_speed = rospy.get_param("~max_turning_speed")
        self.max_turning = rospy.get_param("~max_turning")
        self.min_turning = rospy.get_param("~min_turning")

        # Internal Use Variables - Do not modify without consultation
        self.last_speed = 0.0
        self.desired_speed = 0.0
        self.last_turning = 0.0
        self.desired_turning = 0.0

        # Subscriber
        self.motor_sub = rospy.Subscriber(self.motor_in_topic, Twist, self.motorCB, queue_size=1)

        # Publisher
        self.motor_pub = rospy.Publisher(self.motor_out_topic, Twist, queue_size=1)

        self.max_delta_turning = abs(self.max_turning_speed / self.throttle_smoother_rate)
        self.max_delta_speed = abs(self.max_acceleration / self.throttle_smoother_rate)
        rospy.Timer(rospy.Duration(self.max_delta_speed), self._publish_drive_command)

    # Motor Cmd Callback Function
    def motorCB(self, msg):
        input_speed = msg.linear.x
        input_turning = msg.angular.z
        # Do some sanity clipping
        input_speed = np.clip(input_speed, self.min_speed, self.max_speed)
        input_turning = np.clip(input_turning, self.min_turning, self.max_turning)
	# Set the target speed and turning
        self.desired_speed = input_speed
        self.desired_turning = input_turning

    def _publish_drive_command(self, event):
	# For linear speed
        desired_delta_speed = self.desired_speed-self.last_speed
        clipped_delta_speed = np.clip(desired_delta_speed, -self.max_delta_speed, self.max_delta_speed)
        smoothed_speed = self.last_speed + clipped_delta_speed
        self.last_speed = smoothed_speed
	# For turning speed
        desired_delta_turning = self.desired_turning-self.last_turning
        clipped_delta_turning = np.clip(desired_delta_turning, -self.max_delta_turning, self.max_delta_turning)
        smoothed_turning = self.last_turning + clipped_delta_turning
        self.last_turning = smoothed_turning
        # publish the smoothed linear_speed and turning_speed
    	motor_msg = Twist()
    	motor_msg.linear.x = smoothed_speed
        motor_msg.linear.x = self.desired_speed
    	motor_msg.linear.y = 0
    	motor_msg.linear.z = 0
    	motor_msg.angular.x = 0
    	motor_msg.angular.y = 0
    	motor_msg.angular.z = smoothed_turning
        motor_msg.angular.z = self.desired_turning
    	self.motor_pub.publish(motor_msg)


if __name__=="__main__":
    rospy.init_node("throttle_interpolator")
    InterpolateThrottle()
    rospy.spin()
