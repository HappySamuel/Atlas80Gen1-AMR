#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Bool, String


class V36BatteryMonitoring():
    def __init__(self):
        # Define adjustable Parameter
        # - 36V Battery
        self.volt_36_100 = float(rospy.get_param("~volt_36_100"))
        self.volt_36_95 = float(rospy.get_param("~volt_36_95"))
        self.volt_36_90 = float(rospy.get_param("~volt_36_90"))
        self.volt_36_85 = float(rospy.get_param("~volt_36_85"))
        self.volt_36_80 = float(rospy.get_param("~volt_36_80"))
        self.volt_36_75 = float(rospy.get_param("~volt_36_75"))
        self.volt_36_70 = float(rospy.get_param("~volt_36_70"))
        self.volt_36_65 = float(rospy.get_param("~volt_36_65"))
        self.volt_36_60 = float(rospy.get_param("~volt_36_60"))
        self.volt_36_55 = float(rospy.get_param("~volt_36_55"))
        self.volt_36_50 = float(rospy.get_param("~volt_36_50"))
        self.volt_36_45 = float(rospy.get_param("~volt_36_45"))
        self.volt_36_40 = float(rospy.get_param("~volt_36_40"))
        self.volt_36_35 = float(rospy.get_param("~volt_36_35"))
        self.volt_36_30 = float(rospy.get_param("~volt_36_30"))
        self.volt_36_25 = float(rospy.get_param("~volt_36_25"))
        self.volt_36_20 = float(rospy.get_param("~volt_36_20"))
        self.volt_36_15 = float(rospy.get_param("~volt_36_15"))
        self.volt_36_10 = float(rospy.get_param("~volt_36_10"))
        self.volt_36_5 = float(rospy.get_param("~volt_36_5"))
        self.volt_36_0 = float(rospy.get_param("~volt_36_0"))
        # "+" <---> higher than actual    |    "-" <---> lower than actual
        self.volt_tolerance = float(rospy.get_param("~volt_tolerance"))

        # Internal Use Variables - Do not modify without consultation
        self.v_36 = 0.0
        self.publish_rate = rospy.Rate(1)    # 1 [Hz] <---> 1 [sec]

        # Subscriber
        self.voltage_sub = rospy.Subscriber(rospy.get_param("~voltage_topic"), String, self.voltageCB, queue_size=1)

        # Publisher
        self.battery_pub = rospy.Publisher(rospy.get_param("~battery_topic"), String, queue_size=1)

        # Main Loop
        self.monitoring()

    # Checking the latest Voltage Reading of Battery used
    def voltageCB(self, msg):
        self.v_36 = float(msg.data) - self.volt_tolerance

    # Main Loop
    def monitoring(self):
        while not rospy.is_shutdown():
            # Calculate Battery Percent [%] and publish
            self.battery_percent_calculator()
            self.publish_rate.sleep()

    # Calculate the Battery Percentage based on Battery Voltage
    def battery_percent_calculator(self):
        # 36V Battery
        if(self.v_36 >= self.volt_36_100):
            battery_36 = "100"
        elif(self.volt_36_95 <= self.v_36 < self.volt_36_100):
            battery_36 = "95"
        elif(self.volt_36_90 <= self.v_36 < self.volt_36_95):
            battery_36 = "90"
        elif(self.volt_36_85 <= self.v_36 < self.volt_36_90):
            battery_36 = "85"
        elif(self.volt_36_80 <= self.v_36 < self.volt_36_85):
            battery_36 = "80"
        elif(self.volt_36_75 <= self.v_36 < self.volt_36_80):
            battery_36 = "75"
        elif(self.volt_36_70 <= self.v_36 < self.volt_36_75):
            battery_36 = "70"
        elif(self.volt_36_65 <= self.v_36 < self.volt_36_70):
            battery_36 = "65"
        elif(self.volt_36_60 <= self.v_36 < self.volt_36_65):
            battery_36 = "60"
        elif(self.volt_36_55 <= self.v_36 < self.volt_36_60):
            battery_36 = "55"
        elif(self.volt_36_50 <= self.v_36 < self.volt_36_55):
            battery_36 = "50"
        elif(self.volt_36_45 <= self.v_36 < self.volt_36_50):
            battery_36 = "45"
        elif(self.volt_36_40 <= self.v_36 < self.volt_36_45):
            battery_36 = "40"
        elif(self.volt_36_35 <= self.v_36 < self.volt_36_40):
            battery_36 = "35"
        elif(self.volt_36_30 <= self.v_36 < self.volt_36_35):
            battery_36 = "30"
        elif(self.volt_36_25 <= self.v_36 < self.volt_36_30):
            battery_36 = "25"
        elif(self.volt_36_20 <= self.v_36 < self.volt_36_25):
            battery_36 = "20"
        elif(self.volt_36_15 <= self.v_36 < self.volt_36_20):
            battery_36 = "15"
        elif(self.volt_36_10 <= self.v_36 < self.volt_36_15):
            battery_36 = "10"
        elif(self.volt_36_5 <= self.v_36 < self.volt_36_10):
            battery_36 = "5"
        elif(self.volt_36_0 <= self.v_36 < self.volt_36_5):
            battery_36 = "0"
        else:
            print "36V Battery - Voltage Too Low"
            battery_36 = "-100"    # Battery Faulty - Please Check
        self.battery_pub.publish(battery_36)



if __name__=="__main__":
    rospy.init_node("v36_battery_monitoring")
    V36BatteryMonitoring()
    rospy.spin()
