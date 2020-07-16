#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  (1) Remotely turn off the magnetic navigation mode.
               (2) Return control to lidar navigation.

   Notes:
          buttons[5] <--> deadman button (RB)
          buttons[3] <--> magnetic_navi OFF (yellow Y)
'''

import rospy

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy


class RemoteArdudriveKiller():
    def __init__(self):
        # Subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

        # Publishers
        self.kill_ardudrive_pub = rospy.Publisher("/notify", String, queue_size=1)
        self.check_drive_pub = rospy.Publisher("/waypoint_navi/check_drive", Bool, queue_size=1)
        self.kill_mag_mission_pub = rospy.Publisher("/atlas80/cancel_mag", String, queue_size=1)

    # Status Feedback of Joystick Controller
    def joyCB(self, msg):
        if(msg.buttons[3] == 1 and msg.buttons[5] == 1):
            self.kill_ardudrive_pub.publish("0,0")
            self.check_drive_pub.publish(True)
            self.kill_mag_mission_pub.publish("cancel")



if __name__=="__main__":
    rospy.init_node("remote_ardudrive_killer")
    RemoteArdudriveKiller()
    rospy.spin()
