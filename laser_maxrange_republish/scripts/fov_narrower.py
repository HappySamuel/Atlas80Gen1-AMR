#!/usr/bin/env python
"""
   Author :  Samuel Chieng Kien Ho
   Function :  (1)
               (2)
"""

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan


class FOVNarrower():
    def __init__(self):
        # Define Adjustable Parameters
        # - Scan Region (Round)
        self.ang_min = float(rospy.get_param("~ang_min"))    # [deg]
        self.ang_max = float(rospy.get_param("~ang_max"))    # [deg]
        self.intensity = bool(rospy.get_param("~intensity"))    # [1 or 0]

        # Subscriber
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scanCB, queue_size=1)

        # Publisher
        self.scan_pub = rospy.Publisher("scan_filter", LaserScan, queue_size=1)

    def scanCB(self, msg):
        range_list = list(msg.ranges)
        intensity_list = list(msg.intensities)
        for i in xrange(len(msg.ranges)):
            current_angle = msg.angle_min + msg.angle_increment*i
            # Delete the first element
            if (current_angle < np.deg2rad(self.ang_min)):
                del range_list[0]
                del intensity_list[0]
            # Delete the last element
            elif (current_angle > np.deg2rad(self.ang_max)):
                del range_list[-1]
                del intensity_list[-1]
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = np.deg2rad(self.ang_min)
        new_scan.angle_max = np.deg2rad(self.ang_max)
        new_scan.angle_increment = msg.angle_increment
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        new_scan.ranges = range_list
        if(self.intensity == False):
            new_scan.intensities = []
        else:
            new_scan.intensities = intensity_list
        self.scan_pub.publish(new_scan)



if __name__=="__main__":
    rospy.init_node("fov_narrower")
    FOVNarrower()
    rospy.spin()
