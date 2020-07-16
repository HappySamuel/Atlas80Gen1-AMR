#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import String


class MagneticTesterV1_2():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.repeat_rate = rospy.Rate(0.5)

        # Publisher
        self.init_mag_pub = rospy.Publisher("mag_navi/init", String, queue_size=1)

        # Publishing repeatedly
        self.repeat()

    def repeat(self):
        counter = 0
        only_once = True
        correct = True
        while not rospy.is_shutdown():
            if(only_once == True):
                print "****************************************************************************"
                goal = raw_input("Choose among    \"in-4-right\"    \"out-4-left\"    \"in-5-right\"   \" out-5-left\" \n****************************************************************************\n")
                if(goal!="in-4-right" and  goal!="out-4-left" and goal!="in-5-right" and goal!="out-5-left" and goal!="in-4-exit"):
                    print "----------------------------------------------------------------------"
                    print "| Enter Wrong... Please Re-Run the Code again and Enter correctly... |"
                    print "----------------------------------------------------------------------"
                    # Shut down this node
                    rospy.signal_shutdown("done")
                    correct = False
                only_once = False
            if(counter < 2 and correct == True):
                self.init_mag_pub.publish(goal)
                print "----------- publishing -----------"
                print "goal :  " + goal
            else:
                # Shut down this node
                rospy.signal_shutdown("done")
            counter = counter + 1
            self.repeat_rate.sleep()


if __name__=="__main__":
    rospy.init_node("magnetic_tester_v1_2")
    MagneticTesterV1_2()
    rospy.spin()
