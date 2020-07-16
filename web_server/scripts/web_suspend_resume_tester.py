#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho
   Function :  Testing suspend / resume function of web_server_handler with fake msg
   Features :  (1) Sending suspend / resume based on keyboard input 1 / 0
               (2) Auto rechecking keyboard input with timeout
"""

import rospy
import sys

from select import select
from std_msgs.msg import Bool


class WebSuspendResumeTester():
    def __init__(self):
        # Internal USE Variables - Do not modify without consultation
        self.rate = rospy.Rate(10)
        self.suspend_status = None
        self.timeout = 2

        # Publisher
        self.suspend_resume_pub = rospy.Publisher("/suspend/output", Bool, queue_size=1)

        # Main Loop
        self.loop()

    # Main Loop
    def loop(self):
        while not rospy.is_shutdown():
            rlist, _, _ = select([sys.stdin], [], [], self.timeout)
            if rlist:
                print "-------------------------"
                s = sys.stdin.readline()
                print s
                self.suspend_status = bool(int(s))
            else:
                self.suspend_status = self.suspend_status
            self.suspend_resume_pub.publish(self.suspend_status)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("web_suspend_resume_tester")
    WebSuspendResumeTester()
    rospy.spin()
