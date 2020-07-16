#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

# lidars/scan_multi , lidars/front_lidar/scan , lidars/front_lidar/scan_filter , lidars/rear_lidar/scan , lidars/rear_lidar/scan_filter

class LaserscanTester():
    def __init__(self):
        # Define Topic for subscribing messages
        print ""
        print "Selectable Topics :"
        print "(1) /lidars/front_lidar/scan"
        print "(2) /lidars/front_lidar/scan_filter"
        print "(3) /lidars/rear_lidar/scan"
        print "(4) /lidars/rear_lidar/scan_filter"
        print "(5) /lidars/scan_multi"
        print ""
        print "------------------------------------------"
        self.scan_topic = raw_input("| Please Enter your desired \"scan_topic\" |\n------------------------------------------\n")

        # Subscriber
        laser_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.laserCB, queue_size=1)

    def laserCB(self, msg):
        print str(len(msg.ranges)) + " particles"
        print "----------------------"



if __name__=="__main__":
    rospy.init_node("laserscan_tester")
    LaserscanTester()
    rospy.spin()
