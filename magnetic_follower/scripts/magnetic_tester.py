#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String


class MagneticTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.repeat_rate = rospy.Rate(0.5)
        self.goal = "exit"    # "4"   "5"   "exit"

        # Publisher
        self.init_mag_pub = rospy.Publisher("exit_navi/init", String, queue_size=1) # "exit_navi/init"   "mag_navi/init"

        # Publishing repeatedly
        self.repeat()

    def repeat(self):
        counter = 0
        while not rospy.is_shutdown():
            if(counter < 2):
                print "goal :  " + self.goal
                print "----------- publishing -----------"
                self.init_mag_pub.publish(self.goal)
            else:
                # Shut down this node
                rospy.signal_shutdown("done")
            counter = counter + 1
            self.repeat_rate.sleep()


if __name__=="__main__":
    rospy.init_node("magnetic_tester")
    MagneticTester()
    rospy.spin()
