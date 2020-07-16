#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  Testing Left/Right 90' Turn
'''

import rospy
import numpy as np

from std_msgs.msg import String


class Turning90Tester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(2)    # 2 [Hz] = 0.5 [sec]         

        # Publisher
        self.turn_pub = rospy.Publisher("turning_90/init", String, queue_size=1)

        # Main Loop
        self.send_command()

    def send_command(self):
        counter = 0
        only_once = True
        correct = True
        while not rospy.is_shutdown():
            if(only_once == True):
                print "*********************************************"
                command = raw_input("Choose among    \"left\"    \"right\"    \"cancel\" \n*********************************************\n")
                if(command != "left" and  command  != "right" and command  != "cancel"):
                    print "----------------------------------------------------------------------"
                    print "| Enter Wrong... Please Re-Run the Code again and Enter correctly... |"
                    print "----------------------------------------------------------------------"
                    # Shut down this node
                    rospy.signal_shutdown("done")
                    correct = False
                only_once = False
            if(counter < 2 and correct == True):
                self.turn_pub.publish(command)
                print "----------- publishing -----------"
                print "command :  " + command
            else:
                # Shut down this node
                rospy.signal_shutdown("done")
            counter = counter + 1
            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("turning_90_tester")
    Turning90Tester()
    rospy.spin()
