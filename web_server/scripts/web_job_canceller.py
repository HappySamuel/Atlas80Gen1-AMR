#!/usr/bin/env python

"""
WARNING: KILL "web_server_handler.launch" BEFORE USE!!!
"""

import rospy
import requests
import json
import time
import tf.transformations

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# "status" : "started / arrived / departed / suspended / resumed / cancelled / completed"
# "lifter" : "up / down"

class WebJobCanceller():
    def __init__(self):
        # Define Topics for publishing and subscribing messages
        self.pose_topic = rospy.get_param("~pose_topic", "pf/pose/odom")
        self.lifter_topic = rospy.get_param("~lifter_topic", "lifter/up_down")
        self.moving_topic = rospy.get_param("~moving_topic", "low_level/twist_cmd_mux/output")
        self.battery_topic = rospy.get_param("~battery_topic", "battery/voltage")
        self.status_topic = rospy.get_param("~status_topic", "atlas80/status")
        self.location_topic = rospy.get_param("~location_topic", "atlas80/location")
        self.mission_topic = rospy.get_param("~mission_topic", "atlas80/mission")
        self.suspend_pub_topic = rospy.get_param("~suspend_pub_topic", "suspend/input/web")
        self.suspend_sub_topic = rospy.get_param("~suspend_sub_topic", "suspend/output")
        self.task_topic = rospy.get_param("~task_topic", "atlas80/task")

        # Define Adjustable Parameters
        self.url = rospy.get_param("~url", "https://shado.brigosha.com/hardware/status")
        self.id = str(rospy.get_param("~id", "Atlas80-B"))
        self.update_rate = rospy.Rate(0.5)

        # Internal Use Variables - Do not modify without consultation
        self.headers = {"token": "a857a1db2d53e7ea949e1",
                        "content-type": "application/json"}
        self.get_params = {"id": "Atlas80-B"}
        self.pub_params = {"id": "Atlas80-B",         #
                           "x": "13.21",            #
                           "y": "123.21",           #
                           "angle": "3.14",         #
                           "lifter": "up",          #  String
                           "moving": "true",        #  Array
                           "voltage": "50",         #
                           "current": "1",          #
                           "error": "error01",      #
                           "status": "arrived",     #
                           "location": "Point1"}    #
        self.x_pos = "0.0"
        self.y_pos = "0.0"
        self.yaw_ang = "0.0"
        self.lifter_status = "0"
        self.moving_status = "0"
        self.battery_level = "28.0"
        self.current_level = "0.0"
        self.robot_status = "resumed"
        self.current_job_id = 0
        self.station = []
        self.action = []
        self.command = "none"
        self.last_suspend = False
        self.cmd = False
        self.task_completed = True
        self.total_msg = []

        # Subscribers
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.positionCB, queue_size=1)
        self.lifter_sub = rospy.Subscriber(self.lifter_topic, Bool, self.lifterCB, queue_size=1)
        self.moving_sub = rospy.Subscriber(self.moving_topic, Twist, self.movingCB, queue_size=1)
        self.battery_sub = rospy.Subscriber(self.battery_topic, String, self.batteryCB, queue_size=1)
        self.status_sub = rospy.Subscriber(self.status_topic, String, self.statusCB, queue_size=1)
#        self.location_sub = rospy.Subscriber(self.location_topic, String, self.locationCB, queue_size=1)
        self.suspend_sub = rospy.Subscriber(self.suspend_sub_topic, Bool, self.suspendCB, queue_size=1)
        self.task_sub = rospy.Subscriber(self.task_topic, Bool, self.taskCB, queue_size=1)


        # Publishers
        self.mission_pub = rospy.Publisher(self.mission_topic, String, queue_size=1)
        self.suspend_pub = rospy.Publisher(self.suspend_pub_topic, Bool, queue_size=1)

        # Request Job ID from Keyboard Input
        print "----------------------------------------------------------"
        self.job_id = raw_input("| Please Enter the Job ID that wished to be cancelled... |\n----------------------------------------------------------\n")
        print self.job_id

        # Main Loop
        self.server_routine()

    # Status of the Vehicle from ROS
    def statusCB(self, msg):
        current_status = msg.split(",")
#        if (current_status[0] == "FREE"):

    # Suspend or Resume from ROS
    def suspendCB(self, msg):
        if (msg.data == True):
            self.robot_status = "suspended"
        if (msg.data == False):
            self.robot_status = "resumed"

    # Position of the Vehicle from ROS
    def positionCB(self, msg):
        self.x_pos = str(msg.pose.pose.position.x)
        self.y_pos = str(msg.pose.pose.position.y)
        self.yaw_ang = str(self.quaternion_to_angle(msg.pose.pose.orientation))

    # Battery Level from ROS
    def batteryCB(self, msg):
        self.battery_level = str(msg.data)

    # Lifter Status from ROS
    def lifterCB(self, msg):
        if (msg.data == True):
            self.lifter_status = "up"   # Up
        else:
            self.lifter_status = "down"   # Down

    # Moving Status
    def movingCB(self, msg):
        if (msg.linear.x != 0.0) or (msg.angular.z != 0.0):
            self.moving_status = "1"
        else:
            self.moving_status = "0"

    # Task status, if finished, only then is allowed to receive new mission
    def taskCB(self, msg):
        if (msg.data == True):
            self.task_completed = True
        else:
            self.task_completed = False

    # Main Loop-ing for updating the web server with relevant information
    def server_routine(self):
        while not rospy.is_shutdown():
            self.pub_params["id"] = self.id
            self.pub_params["job_id"] = self.job_id
            self.pub_params["x"] = self.x_pos
            self.pub_params["y"] = self.y_pos
            self.pub_params["angle"] = self.yaw_ang
            self.pub_params["lifter"] = self.lifter_status
            self.pub_params["moving"] = self.moving_status
            self.pub_params["voltage"] = self.battery_level
#            self.pub_params["current"] = self.current_level
            self.pub_params["status"] = "cancelled"
#            self.pub_params["status"] = "completed"
#            self.pub_params["status"] = self.robot_status
            self.pub_params["location"] = "Psa_A"
            # Convert String Array into json format
            jdata = json.dumps(self.pub_params)
            # Update Web_Server the Vehicle's Status if Doing a Job (requests.post)
#            if(self.task_completed == False):
            publish_server = requests.post(self.url, data=jdata, headers=self.headers)
            reply = json.loads(publish_server.text)
            if("command" in reply):
                self.command = reply["command"]
            self.suspend_publish()
            # Receive new Task when there the job is completed (requests.get)
#            else:
#            self.ready_for_new_task()
            # Mission with corresponding Job_ID has been cancelled
            print "--------------------------------------------------"
            print " Mission with selected Job_ID has been cancelled."
            print "--------------------------------------------------"
            self.update_rate.sleep()

    # Requesting new task from web server
    def ready_for_new_task(self):
        self.station = []
        self.action = []
        self.total_msg = ""
        receive_server = requests.get(self.url, data=json.dumps(self.get_params), headers=self.headers)
        if (receive_server.status_code == 200):
            # Convert json format into String Array
            result = json.loads(receive_server.text)
            if "destinations" in result:
                self.current_job_id = int(result["id"])
                destinations = result["destinations"]
                print destinations
                count = int(result["count"])
                for i in range(0, count):
                    self.station.append(destinations[i]["id"])
                    if (destinations[i]["action"] == "PickTable"):
                        self.action.append("1")
                    elif (destinations[i]["action"] == "DropTable"):
                        self.action.append("2")
                    elif (destinations[i]["action"] == "StopWait"):
                        self.action.append("3")
                    elif(destinations[i]["action"] == "StopGo"):
                        self.action.append("4")
                    elif (destinations[i]["action"] == "NoAction") or (destinations[i]["action"] == None):
                        self.action.append("0")
            for i in xrange(len(self.station)):
                job_msg = str(self.station[i])+"-"+str(self.action[i])
                if (self.total_msg == ""):
                    self.total_msg = job_msg
                else:
                    self.total_msg = self.total_msg + "," + job_msg
        print self.total_msg
        self.mission_pub.publish(self.total_msg)

    # Keep on sending same message until changed
    def suspend_publish(self):
        if(self.command == "suspend" or self.command == "resume"):
            self.cmd = not self.cmd
#        if (self.command == "suspend"):
#            cmd = True
#        elif(self.command == "resume"):
#            cmd = False
        else:
            self.cmd = self.last_suspend
        # True <---> Suspend   |   False <---> Resume
        self.suspend_pub.publish(self.cmd)
        self.last_suspend = self.cmd

    # Convert Quaternion to Yaw Angle [rad]
    def quaternion_to_angle(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw


if __name__=="__main__":
    rospy.init_node("web_job_canceller")
    WebJobCanceller()
    rospy.spin()
