#!/usr/bin/env python

'''
Author :  Samuel Chieng Kien Ho
Function :  Testing -180' ~ +180' Turn
'''

import rospy
import numpy as np

from std_msgs.msg import String


class TurnerTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(0.5)    # 0.5 [Hz] = 2 [sec] 
        self.command = "-50"     # -180' ~ +180' [deg]

        # Publisher
        self.turn_pub = rospy.Publisher("turning/init", String, queue_size=1)

        # Main Loop
        self.send_command()

    def send_command(self):
        while not rospy.is_shutdown():
            self.turn_pub.publish(self.command)
            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("turner_tester")
    TurnerTester()
    rospy.spin()
