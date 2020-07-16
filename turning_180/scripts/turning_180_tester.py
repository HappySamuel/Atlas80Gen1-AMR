#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
'''

import rospy

from std_msgs.msg import Bool, String


class Turning180Tester():
    def __init__(self):
        self.publish_rate = rospy.Rate(0.5)

        self.start_turning = True  # True <--> Do  @  False <--> Don't

        # Publisher
        self.turning_pub = rospy.Publisher("turning_180/init", Bool, queue_size=1)
#        self.turning_pub = rospy.Publisher("turning_180/init", String, queue_size=1)

        self.turning_loop()

    def turning_loop(self):
        i = 0
        while not rospy.is_shutdown():
#            if(i % 2 == 0):
#                self.start_turning = True
#            else:
#                self.start_turning = False
#            i += 1
            print self.start_turning
            if(self.start_turning == True):
                self.turning_pub.publish(True)
#                self.turning_pub.publish("turn")
            else:
                self.turning_pub.publish(False)
#                self.turning_pub.publish("cancel")
            self.publish_rate.sleep()


if __name__=="__main__":
    rospy.init_node("turning_180_tester")
    Turning180Tester()
    rospy.spin()
