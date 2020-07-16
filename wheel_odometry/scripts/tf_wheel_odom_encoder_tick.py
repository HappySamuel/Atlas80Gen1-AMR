#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TfWheelOdomEncoderTick():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.encoder_topic = rospy.get_param("~encoder_topic")
        self.wheel_odom_topic = rospy.get_param("~wheel_odom_topic")

        # Define adjustable Parameters
        self.tick_to_distance_gain = float(rospy.get_param("~tick_to_distance_gain"))  # (ticks/m)
        self.dist_between_wheels = float(rospy.get_param("~dist_between_wheels"))      # (m)

        # Internal Use Variables - Do not modify it without consultation
        self.yaw_ = 0
        self.x_ = 0
        self.y_ = 0
        self.counter = 0
        self.accumulate_distance = 0
        self.accumulate_yaw = 0
        self.average_xspeed = 0
        self.average_rotspeed = 0

        # Subscribers
        self.encoder_sub = rospy.Subscriber(self.encoder_topic, String, self.encoderCB, queue_size=1)

        # Publisher
        self.wheel_odom_pub = rospy.Publisher(self.wheel_odom_topic, Odometry, queue_size=1)

    # Encoder tick Callback Function
    def encoderCB(self, msg):
        encoder = msg.data.split(",")
        L_encoder = float(encoder[1])
        R_encoder = float(encoder[0])
        x_speed = float(encoder[2])
        rot_speed = float(encoder[3])
        self.pose_calculator(R_encoder, L_encoder)
        self.publish_odom(self.x_, self.y_, self.yaw_, x_speed, rot_speed)

    # Calculating the Position and Orientation of the AGV based on encoder ticks count
    def pose_calculator(self, R_encoder, L_encoder):
        # Yaw Angle Calculation
        delta_yaw = (R_encoder - L_encoder)/(self.dist_between_wheels*self.tick_to_distance_gain)
        self.yaw_ = self.yaw_ + delta_yaw                           # (rad)
        if(self.yaw_ > np.pi):
            self.yaw_ = self.yaw_ - 2.0*np.pi
        elif(self.yaw_ < -np.pi):
            self.yaw_ = self.yaw_ + 2.0*np.pi
        # Position X and Y Calculation
        delta_x = (R_encoder + L_encoder)*np.cos(self.yaw_)/(2.0*self.tick_to_distance_gain)
        delta_y = (R_encoder + L_encoder)*np.sin(self.yaw_)/(2.0*self.tick_to_distance_gain)
        self.x_ = self.x_ + delta_x				    # (m)
        self.y_ = self.y_ + delta_y				    # (m)
        print "x_distance [m] = " + str(self.x_)
        print "yaw_angle [deg] = " + str(np.rad2deg(self.yaw_))

    # Publish the Wheel Odometry onto the odom frame
    def publish_odom(self, x_pos, y_pos, yaw_angle, current_speed, current_yaw_rate):
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
        wheel_odom.pose.covariance[0] = 0.2     # x
        wheel_odom.pose.covariance[7] = 0.2     # y
        wheel_odom.pose.covariance[35] = 0.4    # yaw
        wheel_odom.twist.twist.linear.x = current_speed
        wheel_odom.twist.twist.linear.y = 0.0
        wheel_odom.twist.twist.angular.z = current_yaw_rate
        self.wheel_odom_pub.publish(wheel_odom)


if __name__=="__main__":
    rospy.init_node("tf_wheel_odom")
    TfWheelOdomEncoderTick()
    rospy.spin()

