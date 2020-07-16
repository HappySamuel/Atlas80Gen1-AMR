#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho

   Works :  Testing the suspend / resume package.
"""

import rospy

from std_msgs.msg import Bool


class WebSuspendTester():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.web_pub_topic = rospy.get_param("~pub_topic", "web/suspend_in")
        self.web_sub_topic = rospy.get_param("~sub_topic", "web/suspend_out")

        # Internal Use Variables - Do not modify without consultation
        self.pub_rate = rospy.Rate(1)    # 0.1 Hz
        self.status = False
        self.last_status = False
        self.counter = 0
        self.web_click = False

        # Publishers
        self.web_pub = rospy.Publisher(self.web_pub_topic, Bool, queue_size=1)

        # Running Main Loop
        self.tester()

        # Subscriber
        self.web_sub = rospy.Subscriber(self.web_sub_topic, Bool, self.webCB, queue_size=1)

    def webCB(self, msg):
        if (msg.data == True):
            print "Suspend"
        if (msg.data == False):
            print "Resume"
        self.last_status = msg.data

    def tester(self):
        while not rospy.is_shutdown():
            self.web_pub.publish(self.web_click)
            self.pub_rate.sleep()


if __name__=="__main__":
    rospy.init_node("web_suspend_tester")
    WebSuspendTester()
    rospy.spin()
