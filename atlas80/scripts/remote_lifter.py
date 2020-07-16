#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  Remotely operate lifter, can command lifter to do lifting or lowering

   Notes: 
          buttons[5] <--> deadman button (RB)
          buttons[6] <--> lowering down (Back)
          buttons[7] <--> lifting up (Start)
'''

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Joy


class RemoteLifter():
    def __init__(self):
        # Subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

        # Publisher
        self.cmd_lift_pub = rospy.Publisher("/lifter/init", String, queue_size=1)

        # Internal Use Variables - Do not modify without consultation
        self.cmd = "2"    # "1" <--> Going Up    |    "0" <--> Going Down    |    "2" <--> Stop

    # Status Feedback of Joystick Controller
    def joyCB(self, msg):
        if(msg.buttons[7] == 1 and msg.buttons[5] == 1):
            self.cmd = "1"
            self.cmd_lift_pub.publish(self.cmd)
        elif(msg.buttons[6] == 1 and msg.buttons[5] == 1):
            self.cmd = "0"
            self.cmd_lift_pub.publish(self.cmd)



if __name__=="__main__":
    rospy.init_node("remote_lifter")
    RemoteLifter()
    rospy.spin()
