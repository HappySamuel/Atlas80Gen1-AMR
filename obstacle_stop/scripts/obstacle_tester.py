#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String


class ObstacleTester():
    def __init__(self):
        # Internal Use Variables - Do not modify without consultation
        self.publish_rate = rospy.Rate(2)
        self.switch = 0       # 1 <---> On   @   0 <---> Off

        # Publisher
        self.obstacle_mode_pub = rospy.Publisher("/obstacle_stop/mode", String, queue_size=1)

        # Input -- 0 or 1 or 2
        self.input = raw_input(" Obstalce Mode ---  0  or  1  or  2  ???\n")
        if(int(self.input) == 0):
            self.mode = "0"
        elif(int(self.input) == 1):
            self.mode = "1"
        elif(int(self.input) == 2):
            self.mode = "2"
        else:
            self.input = self.input
            print "Input Error... Re-run the obstacle_tester.py ..."
            time.sleep(1)
            # Shut down this node
            rospy.signal_shutdown("done")

        # Main Loop
        self.selecting_loop()

    # Main Loop for selecting obstacle detection mode
    def selecting_loop(self):
        counter = 0
        while not rospy.is_shutdown():
            if(counter < 3):
                self.obstacle_mode_pub.publish(self.mode)
            else:
                time.sleep(1)
                # Shut down this node
                rospy.signal_shutdown("done")
            counter = counter + 1
            self.publish_rate.sleep()



if __name__=="__main__":
    rospy.init_node("obstacle_tester")
    ObstacleTester()
    rospy.spin()
