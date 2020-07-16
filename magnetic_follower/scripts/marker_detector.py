#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tftr
from operator import add

from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

# Magnetic DATA:
#
#    [0/1/2/3/4/5/6/7/8/9/10/11/12/13/14/15]
#      LEFT        MIDDLE           RIGHT

class MarkerDetector():
    def __init__(self):
        # Internal USE Variables - Do not modify without consultation
        # Magnetic Data Filtering
        self.prev_1 = []
        self.prev_2 = []
        self.prev_3 = []
        self.prev_4 = []
        self.filtered = []
        # Marker Identifier
        self.left_marker = False
        self.right_marker = False
        self.prev_left_marker = False
        self.prev_right_marker = False
        self.starter = True
        self.side_registered = 2
        self.marker_count = None
        self.marker_msg = None
        self.reset = False

        # Subscriber
        self.mag_sub = rospy.Subscriber("/magnetic", String, self.magCB, queue_size=1)

    # Callback for receiving magnetic reading (50 Hz)
    def magCB(self, msg):
        now_data = []
        data = msg.data.split("/")
        for i in range(0, len(data)):
            a = -1 if(int(data[i]) <= -160) else 1 if (int(data[i]) >= 140) else 0
            now_data.append(a)
        if(self.prev_1 != [] and self.prev_2 != [] and self.prev_3 != [] and self.prev_4 != []):
            self.filtered = self.majority_filter(now_data, self.prev_1, self.prev_2, self.prev_3, self.prev_4)
        self.prev_4 = self.prev_3
        self.prev_3 = self.prev_2
        self.prev_2 = self.prev_1
        self.prev_1 = now_data
        if(self.filtered != []):
            self.marker_identifier(self.filtered)

    # Detect and Identify the magnetic code [ 1 | 2 | 3 | 4 | 5 ]
    def marker_identifier(self, mag_data):
        if(self.reset == True):
            self.left_marker = False
            self.right_marker = False
            self.prev_left_marker = False
            self.prev_right_marker = False
            self.starter = True
            self.side_registered = 2
            self.marker_count = None
#            self.marker_msg = None
            self.reset = False
        s_count = []
        for i in range(0,len(mag_data)):
            if(mag_data[i] == 1):
                s_count.append(i)
        if(s_count != []):
            for i in range(0,len(mag_data)):
                if(mag_data[i] == -1 and i < s_count[0] and self.starter == True):
                    self.side_registered = 0    # 0 - Left marker as base
                    self.starter = False
                elif(mag_data[i] == -1 and i > s_count[-1] and self.starter == True):
                    self.side_registered = 1    # 1 - Right marker as base
                    self.starter = False
            for i in range(0,s_count[0]):
                if(mag_data[i] == -1):
                    self.left_marker = True
                    break
                else:
                    self.left_marker = False
            for i in range(s_count[-1],16):
                if(mag_data[i] == -1):
                    self.right_marker = True
                    break
                else:
                    self.right_marker = False
            # Left marker as base, Right marker as counter
            if(self.starter == False and self.side_registered == 0):
                if(self.left_marker == True):    # When left marker is present
                    if(self.prev_left_marker == False):
                        self.marker_count = 0    # Clear counter when left marker first appears
                    elif(self.right_marker == True and self.prev_right_marker == False):
                        self.marker_count += 1    # increase counter for every appearance of right marker
                elif(self.prev_left_marker == True):    # left marker is absent but was present previously
                    if(self.marker_count == 1):
                        self.marker_msg = "out-1"
                    elif(self.marker_count == 2):
                        self.marker_msg = "out-2"
                    elif(self.marker_count == 3):
                        self.marker_msg = "out-3"
                    elif(self.marker_count == 4):
                        self.marker_msg = "out-4"
                    elif(self.marker_count == 5):
                        self.marker_msg = "out-5"
                    self.reset = True
                self.prev_left_marker = self.left_marker
                self.prev_right_marker = self.right_marker
            # Right marker as base, Left marker as counter
            elif(self.starter == False and self.side_registered == 1):
                if(self.right_marker == True):    # When right marker is present
                    if(self.prev_right_marker == False):
                        self.marker_count = 0    # Clear counter when right marker first appears
                    elif(self.left_marker == True and self.prev_left_marker == False):
                        self.marker_count += 1    # increase counter for every appearance of left marker
                elif(self.prev_right_marker == True):    # right marker is absent but was present previously
                    if(self.marker_count == 1):
                        self.marker_msg = "in-1"
                    elif(self.marker_count == 2):
                        self.marker_msg = "in-2"
                    elif(self.marker_count == 3):
                        self.marker_msg = "in-3"
                    elif(self.marker_count == 4):
                        self.marker_msg = "in-4"
                    elif(self.marker_count == 5):
                        self.marker_msg = "in-5"
                    self.reset = True
                self.prev_left_marker = self.left_marker
                self.prev_right_marker = self.right_marker

    # Filtering the mag_code
    def majority_filter(self, data_1, data_2, data_3, data_4, data_5):
        self.new_data = np.zeros(len(data_1))
        data = [data_1, data_2, data_3, data_4, data_5]
        for i in range(0,len(data)):
            self.new_data = list(map(add, self.new_data, data[i]))
        for i in range(0, len(self.new_data)):
            self.new_data[i] /= len(data)
        for i in range(0, len(self.new_data)):
            self.new_data[i] = 1 if(self.new_data[i]>0.5) else -1 if(self.new_data[i]<-0.5) else 0
        return self.new_data



if __name__=="__main__":
    rospy.init_node("marker_detector")
    MarkerDetector()
    rospy.spin()
