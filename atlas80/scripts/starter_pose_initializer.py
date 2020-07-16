#!/usr/bin/env python
"""
   Author :  Samuel Chieng Kien Ho
   Function :  Temporary replace lidar navigation at the slope area. After finishing travelling, feedback lidar localization with a predefined Pose.
"""

import rospy
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion


class StarterPoseInitializer():
    def __init__(self):
        # Define Adjustable Parameters
        self.start_pt_x = -4.0    # [m]
        self.start_pt_y = 18.0    # [m]
		# psa x=-30.8, y=20.3
		# shado x=-4.0, y=18.0

        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(1)  

        # Publisher
        self.start_pt_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # Run Once
        self.starting()

        # Shut down this node
        rospy.signal_shutdown("done")

    # Receive Latest Position - Decide when to initiate the wheel encoder navi
    def starting(self):
        i = 0
        while not rospy.is_shutdown():
            start_pt = PoseWithCovarianceStamped()
            start_pt.header.frame_id = "/map"
            start_pt.header.stamp = rospy.Time.now()
            start_pt.pose.pose.position = Point(self.start_pt_x, self.start_pt_y, 0.0)
            start_pt.pose.pose.orientation = Quaternion(0.0, 0.0, 0.689144008469, 0.724624410016)
		# shado Quaternion(0.0, 0.0, 0.689144008469, 0.724624410016)
		# psa Quaternion(0.0, 0.0, -0.0315743073084, 0.999501407262)
            self.start_pt_pub.publish(start_pt)
            if(i == 2):
                break
            i = i + 1
            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("starter_pose_initializer")
    StarterPoseInitializer()
    rospy.spin()
