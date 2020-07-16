#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Bool


class ContactorTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.publish_rate = rospy.Rate(2)
        self.switch = 0       # 1 <---> On   @   0 <---> Off

        # Publisher
        self.contactor_pub = rospy.Publisher("self_charging/contactor", Bool, queue_size=1)

        # Input ON or OFF
        self.input = raw_input(" ON  or  OFF  ???\n")
        if(self.input == "ON" or self.input == "on"):
            self.switch = 1
        elif(self.input == "OFF" or self.input == "off"):
            self.switch = 0
        else:
            self.switch = self.switch
            print "Input Error... Re-run the contactor_tester.py ..."
            time.sleep(1)
            # Shut down this node
            rospy.signal_shutdown("done")

        # Main Loop
        self.contactor_loop()

    # Main Loop for switching the contactor to be either ON or OFF
    def contactor_loop(self):
        counter = 0
        while not rospy.is_shutdown():
            if(counter < 3):
                if(self.switch == 1):
                    self.contactor_pub.publish(True)
                else:
                    self.contactor_pub.publish(False)
            else:
                time.sleep(1)
                # Shut down this node
                rospy.signal_shutdown("done")
            counter = counter + 1
            self.publish_rate.sleep()


if __name__=="__main__":
    rospy.init_node("contactor_tester")
    ContactorTester()
    rospy.spin()
