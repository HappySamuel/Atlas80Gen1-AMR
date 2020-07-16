#!/usr/bin/env python

import rospy
import time
import serial
import numpy as np
import random

from std_msgs.msg import Header, String, Bool
from geometry_msgs.msg import Twist

# Status from Arduino DUE:
# (0) robot_xspeed [m/s] , (1) robot_rotspeed [rad/s] , (2) motor_L [m/s] , (3) motor_R [m/s]
# (4) encoder_L [count] , (5) encoder_R [count] , (6) battery_voltage [V] , (7) LED_status [R/G/B/...]
# (8) suspend_status [1=suspend / 0=resume]
# (9) lifter_status [0=at_btm / 1=at_top / 2=ongoing_btm / 3=ongoing_top / 4=suddenly_stop]
# (10) charging_status [0=OFF / 1=ON / 2=on_progress]

# Command to Arduino DUE:
# X=1.0 [m/s]
# Z=1.0 [rad/s]
# LED=R [R=red , G=green , B=blue , T=turqoise , Y=yellow , P=purple , W=white]
# LIFTER=0 [0=going_down , 1=going_up, 2=stop]
# CHARGING=0 [0=OFF_contactor , 1=ON_contactor]
# SUSPEND=1 [1=toggle]

class DueSerial():
    def __init__(self):
        # Define Adjustable Parameters
        self.scale_xspeed = float(rospy.get_param("~scale_xspeed"))
        self.scale_rotspeed = float(rospy.get_param("~scale_rotspeed"))

        # Internal Use Variables - Do not modify without consultation
        self.move = "X=0&Z=0"
        self.led = "LED=R"
        self.con = "CHARGING=0"
        self.lift_cmd = "LIFTER=2"
        self.suspend = ""
        self.web_counter = 0
        self.last_web_msg = False
        self.navi_counter = 0
        self.last_navi_msg = False
        self.encoder_publish_rate = rospy.Rate(20)   # [Hz]
        self.L_encoder = 0       # [count]
        self.R_encoder = 0       # [count]
        self.veh_xspeed = 0.0    # [m/s]
        self.veh_rotspeed = 0.0  # [rad/s]
        self.L_speed = 0.0       # [m/s]
        self.R_speed = 0.0       # [m/s]
        self.voltage = "0"       # [V]
        self.led_status = "R"    # [R/G/B/T/Y/P/W]
        self.suspend_status = 0  # [1 = suspend , 0 = resume]
        self.lifter_status = 4
        self.bootup = False
        self.tog = True
        self.magnetic_status = ""
        self.ardu_drive = "L=-1"
        self.ardu_on = False
        self.boot_on = False
        self.reader_delay = 0
        self.lifting = 2     # [0 = lowering / 1 = lifting | 2 = stop]
        self.prev_suspend_status = 0
        self.suspend_counter = 1

        # Publishers
        self.voltage_pub = rospy.Publisher(rospy.get_param("~voltage_topic"), String, queue_size=1)
        self.lift_status_pub = rospy.Publisher(rospy.get_param("~lift_status_topic"), String, queue_size=1)
        self.suspend_status_pub = rospy.Publisher(rospy.get_param("~suspend_status_topic"), Bool, queue_size=1)
        self.wheel_odom_pub = rospy.Publisher(rospy.get_param("~wheel_odom_topic"), String, queue_size=1)
        self.encoder_pub = rospy.Publisher(rospy.get_param("~encoder_topic"), String, queue_size=1)
        self.magnetic_pub = rospy.Publisher(rospy.get_param("~magnetic_topic"), String, queue_size=1)
        self.to_due_pub = rospy.Publisher("/due/to", String, queue_size=1)
        self.from_due_pub = rospy.Publisher("/due/from", String, queue_size=1)

        # Subscribers
        self.twist_sub = rospy.Subscriber(rospy.get_param("~twist_topic"), Twist, self.twistCB, queue_size=1)
        self.lift_cmd_sub = rospy.Subscriber(rospy.get_param("~lift_cmd_topic"), String, self.lift_cmdCB, queue_size=1)
        self.contactor_sub = rospy.Subscriber(rospy.get_param("~contactor_topic"), Bool, self.contactorCB, queue_size=1)
        self.web_sub = rospy.Subscriber(rospy.get_param("~web_suspend_topic"), String, self.webCB, queue_size=1)
        self.navi_sub = rospy.Subscriber(rospy.get_param("~navi_suspend_topic"), Bool, self.naviCB, queue_size=1)
        self.notify_sub = rospy.Subscriber(rospy.get_param("~notify_topic"), String, self.notCB, queue_size=1)

        # Connecting to Arduino_DUE via Serial
        self.due_serial = serial.Serial('/dev/arduino_due', 115200, timeout=None)
        # Initialize the Arduino_DUE
        self.due_serial.isOpen()
        self.due_serial.flushInput()
        self.due_serial.flushOutput()
        time.sleep(1)

    def twistCB(self, msg):
        xspeed = msg.linear.x * self.scale_xspeed
        rotspeed = msg.angular.z * self.scale_rotspeed
        if((xspeed != 0.0 or rotspeed != 0.0) and self.suspend_status == 0 and self.bootup == False):
            self.led = "LED=G"
            self.ardu_drive = "L=-1"
        elif(xspeed == 0.0 and rotspeed == 0.0 and self.suspend_status == 0 and self.bootup == False):
            self.led = "LED=R"
            self.ardu_drive = "L=-1"
        elif(self.bootup == True):
            if(self.ardu_on):
                self.ardu_drive = "L=7.5"
            else:
                self.ardu_drive = "L=-1"
            if(self.tog):
                self.led = "LED=P"
                self.tog = False
            else:
                self.led = "LED=Y"
                self.tog = True
        elif(self.suspend_status == 1):
            self.led = "LED=B"
            self.ardu_drive = "L=-1"
        if(self.lifting == self.lifter_status):
            self.lift_cmd = "LIFTER=2"
        if(self.suspend != ""):
            self.move = "X="+str(xspeed)+"&Z="+str(rotspeed)+"&"+self.led+"&"+self.lift_cmd+"&"+self.con+"&"+self.suspend
            self.suspend = ""
        elif(self.suspend_status == 1): # Being Suspended
            self.move = "X=0&Z=0&"+self.led+"&"+self.lift_cmd+"&"+self.con
        else:
            self.move = "X="+str(xspeed)+"&Z="+str(rotspeed)+"&"+self.led+"&"+self.lift_cmd+"&"+self.con


        tid = "TID="+str(random.randrange(1, 100000))
        self.move = self.ardu_drive+"&"+tid+"&"+self.move
#        print "----------------------------------" #
#        print self.move                    # add by Samuel for monitoring purpose
#        print "----------------------------------" #
        self.to_due_pub.publish(self.move)
        try:       
            self.due_serial.write(self.move + "\n")
        except Exception as e:
            try:
                print e
                self.due_serial = serial.Serial('/dev/arduino_due', 115200, timeout=None)
            except Exception as e:
                print "Reconnecting Serial...."
            else:
                time.sleep(0.1)
                print "Reconnecting OK...."
                self.due_serial.isOpen()
                self.due_serial.flushInput()
        else:
            # Read the incoming data from Arduino_DUE
            read = ""
            while self.due_serial.inWaiting():
                tmp = self.due_serial.readline()
                read = tmp.split(",")
                if(len(read) == 12):
                    if(self.reader_delay == 20):
                        print tmp
                        print self.move
                        print "Notify:"+str(self.bootup)+", LowDrive:"+str(self.ardu_on)
                        self.reader_delay = 0
                    self.reader_delay = self.reader_delay + 1
                    self.veh_xspeed = float(read[0])
                    self.veh_rotspeed = float(read[1])
                    self.L_speed = -float(read[2])
                    self.R_speed = float(read[3])
                    self.L_encoder = int(read[4])
                    self.R_encoder = -int(read[5])
                    self.voltage = read[6]
                    self.led_status = read[7]
                    self.suspend_status = int(read[8])
                    self.lifter_status = int(read[9])
                    self.charging_status = read[10]
                    self.magnetic_status = read[11]
                    self.from_due_pub.publish(tmp)
                self.encoder_publishing()
                self.suspend_publishing()
                self.voltage_monitoring()
                self.magnetic_publishing()
                self.lifter_publishing()


    # Receive Lifting/Lowering message from Navigation Master
    def lift_cmdCB(self, msg):
        if(int(msg.data) == 1):
            self.lift_cmd = "LIFTER=1"    # Lifting Up
            self.lifting = 1
        elif(int(msg.data) == 0):
            self.lift_cmd = "LIFTER=0"    # Lowering Down
            self.lifting = 0
        elif(int(msg.data) == 2):
            self.lift_cmd = "LIFTER=2"    # Stop
            self.lifting = 2

    # Receive ON/OFF message from Self-Charging
    def contactorCB(self, msg):
        if(msg.data == True):
            self.con = "CHARGING=1"    # Contactor - ON
        elif(msg.data == False):
            self.con = "CHARGING=0"    # Contactor - OFF

    # Suspend/Resume from Web Server
    def webCB(self, msg):
        if(msg.data == "suspend" and self.suspend_status == 0 and self.suspend_counter == 1):
            self.suspend = "SUSPEND=1"
            self.suspend_counter = 0
        elif(msg.data == "resume" and self.suspend_status == 1 and self.suspend_counter == 1):
            self.suspend = "SUSPEND=1"
            self.suspend_counter = 0

    def notCB(self, msg):
        bd = msg.data.split(",")
        if(bd[0]=="1" or bd[1]=="1"):
            self.bootup = True
            if(bd[0]=="1"):
                self.boot_on = True
            elif(bd[1]=="1"):
                self.ardu_on = True
        elif(bd[0]=="0" and bd[1]=="0"):
            self.bootup = False
            self.boot_on = False
            self.ardu_on = False
        

    # Suspend/Resume from Navigation Master
    def naviCB(self, msg):
        if(msg.data == True and self.suspend_status == 0 and self.suspend_counter == 1):
            self.suspend = "SUSPEND=1"
            self.suspend_counter = 0
        elif(msg.data == False and self.suspend_status == 1 and self.suspend_counter == 1):
            self.suspend = "SUSPEND=1"
            self.suspend_counter = 0

    # Publishing voltage reading for monitoring
    def voltage_monitoring(self):
        self.voltage_pub.publish(self.voltage)

    # Publishing encoder reading for making into wheel odom
    def encoder_publishing(self):
        tick = str(self.L_encoder)+","+str(self.R_encoder)+","+str(self.veh_xspeed)+","+str(self.veh_rotspeed)
        self.encoder_pub.publish(tick)

    # Publishing magnetic line sensors for making use
    def magnetic_publishing(self):
        data = self.magnetic_status
        self.magnetic_pub.publish(data)

    # Publishing magnetic line sensors for making use
    def lifter_publishing(self):
        data = str(self.lifter_status) 
        self.lift_status_pub.publish(data)

    # Publishing suspend status for making use
    def suspend_publishing(self):
        data = bool(self.suspend_status)
        self.suspend_status_pub.publish(data)
        if(self.prev_suspend_status != self.suspend_status):
            self.suspend_counter = 1
        self.prev_suspend_status = self.suspend_status



if __name__=="__main__":
    rospy.init_node("due_serial")
    DueSerial()
    rospy.spin()
