#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho

   Works :  (1) Read the status of Toggle Button for using suspend / resume function.
            (2) Report to Web Server about the suspend / resume status.
"""

import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


class SuspendResume():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.web_pub_topic = rospy.get_param("~web_pub_topic")
        self.suspend_status_topic = rospy.get_param("~suspend_status_topic")
        self.drive_topic = rospy.get_param("~drive_topic")

        # Internal Use Variables - Do not modify without consultation
        self.status = False
        self.pub_rate = rospy.Rate(30)    # 30 Hz

        # Subscriber
        self.suspend_status_sub = rospy.Subscriber(self.suspend_status_topic, Bool, self.suspend_statusCB, queue_size=1)

        # Publishers
        self.web_pub = rospy.Publisher(self.web_pub_topic, Bool, queue_size=1)
        self.drive_pub = rospy.Publisher(self.drive_topic, Twist, queue_size=1)

        # Running Main Loop
        self.update_status()

    def suspend_statusCB(self, msg):
        self.status = msg.data

    def update_status(self):
        while not rospy.is_shutdown():
            # True <---> Suspend    |    False <---> Resume
            suspend = self.status
            self.web_pub.publish(suspend)
            if (suspend == True):
                self.stop()
            else:
                print "Resume"
            self.pub_rate.sleep()

    def stop(self):
        print "Suspend"
        cmd = Twist()
        cmd.linear = Vector3(0, 0, 0)
        cmd.angular = Vector3(0, 0, 0)
        self.drive_pub.publish(cmd)


if __name__=="__main__":
    rospy.init_node("suspend_resume")
    SuspendResume()
    rospy.spin()
