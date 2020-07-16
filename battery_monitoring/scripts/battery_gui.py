#!/usr/bin/env python

import rospy
import wx

from visual import *
from std_msgs.msg import String


class BatteryGui():
    def __init__(self):
        # Main Scene
        scene = display(title="Battery GUI")

        # Battery Object
        background = box(length=0.18, height=0.38, width=0.01, color=color.blue, pos=(0,0,0))
        background = box(length=0.08, height=0.04, width=0.01, color=color.blue, pos=(0,0.21,0))
	
        # Subscriber
        self.battery_sub = rospy.Subscriber("battery/percentage", String, self.batteryCB, queue_size=1)

    # Battery Monitoring CallBack Function
    def batteryCB(self, msg):
        percent = int(msg.data)
        if percent >= 80:          # green - 6 bars
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,-0.155,0))
        elif 70 <= percent < 80:        # green - 5 bars
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.green, pos=(0,-0.155,0))
        elif 55 <= percent < 70:        # yellow - 4 bars
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.yellow, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.yellow, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.yellow, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.yellow, pos=(0,-0.155,0))
        elif 40 <= percent < 55:        # orange - 3 bars
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.orange, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.orange, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.orange, pos=(0,-0.155,0))
        elif 30 <= percent < 40:        # red - 2 bars
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.red, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.red, pos=(0,-0.155,0))
        elif 20 <= percent < 30:        # red - 1 bar
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.red, pos=(0,-0.155,0))
        else:                           # red - 1 line
            bar100 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.145,0))
            bar80 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.085,0))
            bar60 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,0.025,0))
            bar40 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,-0.035,0))
            bar20 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,-0.095,0))
            bar5 = box(length=0.14, height=0.04, width=0.01, color=color.blue, pos=(0,-0.155,0))
            bar1 = box(length=0.14, height=0.01, width=0.01, color=color.red, pos=(0,-0.17,0))
            print "---------- Please go for Charging. The Battery Level is too low ----------"
        percent_label = label(text=str(percent)+" %", pos=(0.18,0,0))



if __name__=="__main__":
    rospy.init_node("battery_gui")
    BatteryGui()
    rospy.spin()
    wx.Exit()
