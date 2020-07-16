#!/usr/bin/env python

'''
   Author :  Samuel Chieng Kien Ho
   Function :  (1) Remotely shutdown the PC.
               (2) Sending "save and exit" command to ROS logger before turning OFF the PC.
   Notes: 
          buttons[5] <--> deadman button (RB)
          buttons[8] <--> Power Off (Logitech)
'''

import rospy
import dbus
import time

from subprocess import call
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class RemoteShutdown():
    def __init__(self):
        # Subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

        # Publisher
        self.shutdown_pub = rospy.Publisher("atlas80/shutdown", Bool, queue_size=1)

        # Linking the Shutdown function between ROS and dbus
        self.sys_bus = dbus.SystemBus()
        self.ck_srv = self.sys_bus.get_object('org.freedesktop.ConsoleKit', '/org/freedesktop/ConsoleKit/Manager')
        self.ck_iface = dbus.Interface(self.ck_srv, 'org.freedesktop.ConsoleKit.Manager')
        self.stop_method = self.ck_iface.get_dbus_method("Stop")

    def joyCB(self, msg):
        print "-------------"
        print msg.buttons
        print "-------------"
        if(msg.buttons[8]==1 and msg.buttons[5]==1):
            self.shutdown_pub.publish(True)
            print "shutting down..."
            time.sleep(3)
            self.stop_method()
        print "shut_or_not :  " + str(msg.buttons[8])



if __name__=="__main__":
    rospy.init_node("remote_shutdown")
    RemoteShutdown()
    rospy.spin()
