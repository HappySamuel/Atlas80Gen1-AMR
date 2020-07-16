#!/usr/bin/env python

import rospy

from std_msgs.msg import String


class ChargingTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.pub_rate = rospy.Rate(2)    # 1 [Hz]

        # Publisher
        self.kill = rospy.Publisher("/notify", String, queue_size=1)

        # Starting to Test
        self.tester()

        # Shut down this node
        rospy.signal_shutdown("done")

    def tester(self):
        i = 0
        while not rospy.is_shutdown():
            # OFF ardudrive
#            self.kill.publish("0,0")
            # ON ardudrive
            self.kill.publish("0,1")
            if(i == 2):
                break
            i = i + 1
            self.pub_rate.sleep()


if __name__=="__main__":
    rospy.init_node("charging_tester")
    ChargingTester()
    rospy.spin()
