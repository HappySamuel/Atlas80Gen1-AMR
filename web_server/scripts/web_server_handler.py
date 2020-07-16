#!/usr/bin/env python

"""
Author :  Samuel Chieng Kien Ho
Reference :  Muzzammil's comhand.py
"""

import rospy
import requests
import json
import time

from std_msgs.msg import String, Bool

# "status" : "started / arrived / departed / suspended / resumed / cancelled / completed"
# "lifter" : "up / down"

class WebServerHandler():
    def __init__(self):
        # Define Adjustable Parameters
        self.id = rospy.get_param("~id")

        # Internal Use Variables - Do not modify without consultation
        self.refresh_rate = rospy.Rate(0.5)   # 0.5 [Hz] <---> 2 [sec]
        self.x_pos = "0.0"
        self.y_pos = "0.0"
        self.yaw_ang = "0.0"
        self.lifter_status = "0"
        self.moving_status = "0"
        self.battery_level = "100"
        self.current_level = "0"
        self.robot_status = ""
        self.location = "Point1"
        self.current_job_id = "0"
        self.command = "none"
        self.complete_counter = 0
        self.start_counter = 1
        self.first_run = True
        self.run_once = True
        self.stop_send = False
        self.stop_send_counter = 0
        self.last_suspend = False    # recording last msg from suspendCB
        self.suspend_now = False     # True - suspend | False - resume
        self.suspend_count = 0       # Timer for publishing resume msg

        # Messages used for receiving and updating Web-Server
        self.url = "https://shado.brigosha.com/hardware/status"
        self.headers = {"token": "a857a1db2d53e7ea949e1",
                        "content-type": "application/json"}
        self.pub_msg = {"id": self.id,                    #
                        "job_id": self.current_job_id,    #
                        "x": self.x_pos,                  #
                        "y": self.y_pos,                  #
                        "angle": self.yaw_ang,            #
                        "lifter": self.lifter_status,     #  String
                        "moving": self.moving_status,     #  Array
                        "voltage": self.battery_level,    #
                        "current": "0",                   #
                        "error": "error01",               #
                        "status": self.robot_status,      #
                        "location": self.location}        #
        self.sub_msg = {"id": self.id}    # String Array

        # Subscribers
        self.all_sub = rospy.Subscriber(rospy.get_param("~all_topic"), String, self.allCB, queue_size=1)
        self.suspend_sub = rospy.Subscriber(rospy.get_param("~suspend_sub_topic"), Bool, self.suspendCB, queue_size=1)

        # Publishers
        self.suspend_pub = rospy.Publisher(rospy.get_param("~suspend_pub_topic"), String, queue_size=1)
        self.mission_pub = rospy.Publisher(rospy.get_param("~mission_topic"), String, queue_size=1)
        self.magnetic_pub = rospy.Publisher(rospy.get_param("~magnetic_topic"), String, queue_size=1)
        self.notify_pub = rospy.Publisher(rospy.get_param("~notify_topic"), String, queue_size=1)

        print "-------------------------"
        print "| Robot ID :  "+ self.id +" |"
        print "-------------------------"

        # Main Loop Running
        self.server_routine()

    # 1st - x_pos | 2nd - y_pos | 3rd - yaw | 4th - lifter | 5th - moving
    # 6th - voltage | 7th - delivery_status | 8th - location
    # Receive all type of msgs to be updated to the Web-Server
    def allCB(self, msg):
        receive_msg = msg.data.split(",")
        self.x_pos = receive_msg[0]
        self.y_pos = receive_msg[1]
        self.yaw_ang = receive_msg[2]
        self.lifter_status = receive_msg[3]
        self.moving_status = receive_msg[4]
        self.battery_level = receive_msg[5]
        if(self.suspend_now == True):
            self.robot_status = "suspended"
        elif(self.suspend_now == False and self.suspend_count < 8):
            self.robot_status = "resumed"
            self.suspend_count += 1
        else:
            self.robot_status = receive_msg[6]
        self.location = receive_msg[7]

    # Suspend or Resume from ROS to update Web-Server (30 Hz)
    def suspendCB(self, msg):
        if(msg.data == True and msg.data != self.last_suspend):
            self.suspend_now = True
        elif(msg.data == False and msg.data != self.last_suspend):
            self.suspend_now = False
            self.suspend_count = 0
        self.last_suspend = msg.data

    # Main Loop-ing for updating Status and receiving Mission from Web-Server
    def server_routine(self):
        while not rospy.is_shutdown():
            # First time initiating this package, receive new mission
            if(self.start_counter == 1):
                print "pending..."
                self.ready_for_new_mission()
                self.complete_counter = 0
                self.robot_status = ""
            # For Web-Server mission reporting (Mission Completed)
            elif(self.robot_status == "completed" and self.complete_counter < 5):
                print "finish mission and reporting completed..."
                self.update_status()
            # For receiving NEW mission when after reporting job is completed (5 times)
            elif(self.robot_status == "completed" and self.complete_counter >= 5):
                print "ready for next mission..."
                self.current_job_id = "0"
                self.ready_for_new_mission()
                self.start_counter = 1
            # Update Web-Server with the delivery_status if doing a job.
            else:
                print "doing mission now and updating webserver..."
                self.update_status()
            self.suspend_publish()
            if(self.first_run):
                #blink lights
                self.notify_pub.publish("1,0")
                self.first_run = False
            elif(self.run_once):
                self.notify_pub.publish("0,0")
                self.run_once = False
            self.refresh_rate.sleep()

    # Updating Robot Status to Web-Server (requests.post)
    def update_status(self):
        self.pub_msg["id"] = self.id
        self.pub_msg["job_id"] = self.current_job_id
        self.pub_msg["x"] = self.x_pos
        self.pub_msg["y"] = self.y_pos
        self.pub_msg["angle"] = self.yaw_ang
        self.pub_msg["lifter"] = self.lifter_status
        self.pub_msg["moving"] = self.moving_status
        self.pub_msg["voltage"] = self.battery_level
        self.pub_msg["status"] = self.robot_status
        self.pub_msg["location"] = self.location
        print self.pub_msg["status"]                    # DEBUG use
        # Convert String Array into json format
        jmsg = json.dumps(self.pub_msg)
        # Update Web-Server with information
        try:
            publish_server = requests.post(self.url, data=jmsg, headers=self.headers, timeout=5)
        except requests.exceptions.RequestException as e:
            print e
            print "----------------------------------------------------"
            print "| Status Updating Refused... Trying Again later... |"
            print "----------------------------------------------------"
        else:
            if(publish_server.status_code == 200):
                # Convert json format into String Array
                reply = json.loads(publish_server.text)
#                print reply                    # DEBUG use
                if("command" in reply):
                    self.command = reply["command"]
        if(self.robot_status == "completed"):
            self.complete_counter += 1

    # Requesting New Mission from Web-Server (requests.get)
    def ready_for_new_mission(self):
        station = []
        action = []
        self.total_msg = ""
        # Convert String Array into json format
        jmsg = json.dumps(self.sub_msg)
        # Get information from Web-Server
        try:
            receive_server = requests.get(self.url, data=jmsg, headers=self.headers, timeout=5)
        except requests.exceptions.RequestException as e:
            print e
            print "--------------------------------------------------------"
            print "| Command Downloading Refused... Trying Again later... |"
            print "--------------------------------------------------------"
        else:
            if(receive_server.status_code == 200):
                # Convert json format into String Array
                result = json.loads(receive_server.text)
                if("destinations" in result):
                    self.current_job_id = result["id"]
                    destinations = result["destinations"]
                    count = int(result["count"])
                    for i in xrange(len(destinations)):
                        station.append(destinations[i]["id"])
                        if(destinations[i]["action"] == "Pick Table"):
                            action.append("1")
                        elif(destinations[i]["action"] == "Drop Table"):
                            action.append("2")
                        elif(destinations[i]["action"] == "Stop & Wait"):
                            action.append("3")
                        elif(destinations[i]["action"] == "Stop & Go"):
                            action.append("4")
                        elif(destinations[i]["action"] == "No Action" or destinations[i]["action"] == None):
                            action.append("0")
                for i in xrange(len(station)):
                    job_msg = str(station[i]) + "-" + str(action[i])
                    if (self.total_msg == ""):
                        self.total_msg = job_msg
                    else:
                        self.total_msg = self.total_msg + "," + job_msg
        if(self.total_msg != ""):
            self.start_counter = 0
        if(int(self.current_job_id) != 0):
            self.mission_pub.publish(self.current_job_id + "=" + self.total_msg)
            print "--------------------"
            print "| Mission Received |"
            print "|     "+ self.current_job_id +"      |"
            print "| "+ self.total_msg +"  |"
            print "--------------------"

    # Keep on sending same message until any changes coming - Sending to Due Serial
    def suspend_publish(self):
        if(self.command == "suspend" or self.command == "resume"): 
            self.suspend_pub.publish(self.command)



if __name__=="__main__":
    rospy.init_node("web_server_handler")
    WebServerHandler()
    rospy.spin()
