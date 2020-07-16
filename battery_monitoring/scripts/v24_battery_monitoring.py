#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Bool, String


class V24BatteryMonitoring():
    def __init__(self):
        # Define Adjustable Parameter
        # - 24V Battery
        self.volt_24_100 = float(rospy.get_param("~volt_24_100"))
        self.volt_24_90 = float(rospy.get_param("~volt_24_90"))
        self.volt_24_80 = float(rospy.get_param("~volt_24_80"))
        self.volt_24_70 = float(rospy.get_param("~volt_24_70"))
        self.volt_24_60 = float(rospy.get_param("~volt_24_60"))
        self.volt_24_50 = float(rospy.get_param("~volt_24_50"))
        self.volt_24_40 = float(rospy.get_param("~volt_24_40"))
        self.volt_24_30 = float(rospy.get_param("~volt_24_30"))
        self.volt_24_20 = float(rospy.get_param("~volt_24_30"))
        self.volt_24_10 = float(rospy.get_param("~volt_24_10"))
        self.volt_24_0 = float(rospy.get_param("~volt_24_0"))
        # "+" <---> higher than actual    |    "-" <---> lower than actual
        self.volt_tolerance = float(rospy.get_param("~volt_tolerance"))

        # Internal Use Variables - Do not modify without consultation
        self.v_24 = 0.0
        self.publish_rate = rospy.Rate(1)    # 1 [Hz] <---> 1 [sec]

        # Subscriber
        self.voltage_sub = rospy.Subscriber(rospy.get_param("~voltage_topic"), String, self.voltageCB, queue_size=1)

        # Publisher
        self.battery_pub = rospy.Publisher(rospy.get_param("~battery_topic"), String, queue_size=1)

        # Main Loop
        self.monitoring()

    # Checking the latest Voltage Reading of Battery used
    def voltageCB(self, msg):
        self.v_24 = float(msg.data) - self.volt_tolerance

    # Main Loop
    def monitoring(self):
        while not rospy.is_shutdown():
            # Calculate Battery Percent [%] and publish
            self.battery_percent_calculator()
            self.publish_rate.sleep()

    # Calculate the Battery Percentage based on Battery Voltage
    def battery_percent_calculator(self):
        # 24V Battery
        if(self.v_24 > self.volt_24_100):
            battery_24 = "100"
        elif(self.volt_24_90 <= self.v_24 < self.volt_24_100):
            battery_24 = "90"
        elif(self.volt_24_80 <= self.v_24 < self.volt_24_90):
            battery_24 = "80"
        elif(self.volt_24_70 <= self.v_24 < self.volt_24_80):
            battery_24 = "70"
        elif(self.volt_24_60 <= self.v_24 < self.volt_24_70):
            battery_24 = "60"
        elif(self.volt_24_50 <= self.v_24 < self.volt_24_60):
            battery_24 = "50"
        elif(self.volt_24_40 <= self.v_24 < self.volt_24_50):
            battery_24 = "40"
        elif(self.volt_24_30 <= self.v_24 < self.volt_24_40):
            battery_24 = "30"
        elif(self.volt_24_20 <= self.v_24 < self.volt_24_30):
            battery_24 = "20"
        elif(self.volt_24_10 <= self.v_24 < self.volt_24_20):
            battery_24 = "10"
        elif(self.volt_24_0 <= self.v_24 < self.volt_24_10):
            battery_24 = "0"
        else:
            print "24V Battery - Voltage Too Low"
            battery_24 = "-100"    # Battery Faulty - Please Check
        self.battery_pub.publish(battery_24)



if __name__=="__main__":
    rospy.init_node("v24_battery_monitoring")
    V24BatteryMonitoring()
    rospy.spin()
