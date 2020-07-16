#!/usr/bin/env python

import rospy
import magnetic_utils as mag_util

from std_msgs.msg import String

# Magnetic DATA (16 channel):
#
#    [0/1/2/3/4/5/6/7/8/9/10/11/12/13/14/15]
#      LEFT        MIDDLE           RIGHT
#
# Filter Method: (1) data > 120         -->   1
#                (2) data < -150        -->  -1
#                (3) -150 < data < 120  -->   0
#
# Wherever "1" is found in the data, it is counted as "middle".
# Then the data on the left channels which next to the "middle" are counted as "LEFT side".
# While the data on the right channels which next to the "middle" are counted as "RIGHT side".
#
# Direction "in"  -->  1
#           "out" -->  2

class MarkerIdentifier():
    def __init__(self):
        # Internal USE Variables - Do not modify without consultation
        self.left_marker = False
        self.right_marker = False
        self.prev_left_marker = False
        self.prev_right_marker = False
        self.starter = True
        self.side_registered = 2
#        self.marker_identifier = mag_utils.MarkerIdentifier()

        # Subscriber
        self.mag_sub = rospy.Subscriber("/magnetic", String, self.magCB, queue_size=1)

        # Publisher
        self.marker_pub = rospy.Publisher("/mag_navi/mag_code", String, queue_size=1)

    def magCB(self, msg):
        now_data = []
        data = msg.data.split("/")
        for i in range(0, len(data)):
            a = -1 if(int(data[i]) < -160) else 1 if (int(data[i]) > 120) else 0
            now_data.append(a)
        self.identifier(now_data)
#        self.marker_identifier.identifier(now_data)

    def identifier(self, mag_data):
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
                        self.marker_count = 0    # clear counter when left marker first appears
                    elif(self.right_marker == True and self.prev_right_marker == False):
                        self.marker_count += 1   # increase counter at every appearance of right marker
                elif(self.prev_left_marker == True): # left marker is absent but was present previously
                    if(self.marker_count == 1):
                        print "Coming Out and Currently at \"1\""
                    elif(self.marker_count == 2):
                        print "Coming Out and Currently at \"2\""
                    elif(self.marker_count == 3):
                        print "Coming Out and Currently at \"3\""
                    elif(self.marker_count == 4):
                        print "Coming Out and Currently at \"4\""
                    elif(self.marker_count == 5):
                        print "Coming Out and Currently at \"5\""
                    self.starter = True
                self.prev_left_marker = self.left_marker
                self.prev_right_marker = self.right_marker
            # Right marker as base, Left marker as counter
            elif(self.starter == False and self.side_registered == 1):
                if(self.right_marker == True):    # When left marker is present
                    if(self.prev_right_marker == False):
                        self.marker_count = 0    # clear counter when left marker first appears
                    elif(self.left_marker == True and self.prev_left_marker == False):
                        self.marker_count += 1   # increase counter at every appearance of right marker
                elif(self.prev_right_marker == True): # left marker is absent but was present previously
                    if(self.marker_count == 1):
                        print "Going In and Currently at \"1\""
                    elif(self.marker_count == 2):
                        print "Going In and Currently at \"2\""
                    elif(self.marker_count == 3):
                        print "Going In and Currently at \"3\""
                    elif(self.marker_count == 4):
                        print "Going In and Currently at \"4\""
                    elif(self.marker_count == 5):
                        print "Going In and Currently at \"5\""
                    self.starter = True
                self.prev_left_marker = self.left_marker
                self.prev_right_marker = self.right_marker



if __name__=="__main__":
    rospy.init_node("marker_identifier")
    MarkerIdentifier()
    rospy.spin()
