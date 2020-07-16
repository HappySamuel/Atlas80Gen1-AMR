#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool


class ChargingTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.pub_rate = rospy.Rate(1)    # 1 [Hz]
        self.start = True

        # Subscriber
        self.finish_charging_sub = rospy.Subscriber("/self_charging/status", Bool, self.finish_chargingCB, queue_size=1)

        # Publisher
        self.init_charging_pub = rospy.Publisher("/self_charging/init", Bool, queue_size=1)

        self.tester()

    def finish_chargingCB(self, msg):
        print msg.data

    def tester(self):
        for i in range(0, 2):
            self.init_charging_pub.publish(self.start)
            self.pub_rate.sleep()


if __name__=="__main__":
    rospy.init_node("charging_tester")
    ChargingTester()
    rospy.spin()
