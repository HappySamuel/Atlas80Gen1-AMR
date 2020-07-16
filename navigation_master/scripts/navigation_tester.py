#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry


class NavigationTester():
    def __init__(self):
        # Define Topics for publishing and subscribing messages
        self.pose_topic = rospy.get_param("~pose_topic", "pf/pose/odom")

        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(20)   # 10 Hz <---> 0.1 sec
        self.x_pos = -10.9
        self.y_pos = 52.6

        # Publisher
        self.pose_pub = rospy.Publisher(self.pose_topic, Odometry, queue_size=1)

        # Main Loop
        self.fake_pose()

    def fake_pose(self):
        while not rospy.is_shutdown():
            fake_p = Odometry()
            fake_p.header.frame_id = "/map"
            fake_p.header.stamp = rospy.Time.now()
            fake_p.pose.pose.position.x = self.x_pos
            fake_p.pose.pose.position.y = self.y_pos
            self.pose_pub.publish(fake_p)
            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("navigation_tester")
    NavigationTester()
    rospy.spin()
