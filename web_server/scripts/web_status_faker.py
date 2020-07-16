#!/usr/bin/env python

"""
   Author :  Samuel Chieng Kien Ho
   Function :  Testing web_server_handler with both fake robot_status and location
   Features :  (1) Sending robot_status / location based on keyboard input
               (2) Auto choose overwriting robot_status / location
               (3) Auto rechecking keyboard input with timeout
"""

import rospy
import sys

from select import select
from std_msgs.msg import String


class WebStatusFaker():
    def __init__(self):
        # Internal USE Variables - Do not modify without consultation
        self.rate = rospy.Rate(1)
        self.timeout = 4
        self.robot_status = ""
        self.location = ""

        # Publisher
        self.all_pub = rospy.Publisher("/atlas80/all_status", String, queue_size=1)

        # Main Loop
        self.loop()

    # Main Loop
    def loop(self):
        while not rospy.is_shutdown():
            rlist, _, _ = select([sys.stdin], [], [], self.timeout)
            if rlist:
                print "-------------------------"
                s = sys.stdin.readline()
                s = s.split("\n")
                if(s[0] == "started" or s[0] == "arrived" or s[0] == "departed" or s[0] == "completed" ):
                    self.robot_status = s[0]
                else:
                    self.location = s[0]
            else:
                self.location = self.location
                self.robot_status = self.robot_status
            sending_msg = "0,0,0,0,1,100,"+self.robot_status+","+self.location
            self.all_pub.publish(sending_msg)
            self.rate.sleep()



if __name__=="__main__":
    rospy.init_node("web_status_faker")
    WebStatusFaker()
    rospy.spin()
