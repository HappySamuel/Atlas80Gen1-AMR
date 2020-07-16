#!/usr/bin/env python

"""
   Authors :  (1) Samuel Chieng Kien Ho - ROS Integration
              (2) Khairul Muzzammil - Wifi TCP Connection
   Function :  Using Wifi TCP Connection to access the vehicle and control the motion of the vehicle
"""

import rospy
import socket
import select
from geometry_msgs.msg import Twist, Vector3


class WifiTcpTeleop():
	def __init__(self):
		# Define Topics for publishing or subscribing messages
		self.teleop_topic = rospy.get_param("~teleop_topic")

		# Define Adjustable Parameters
		self.ip_address = rospy.get_param("~ip_address")
		self.port = int(rospy.get_param("~port"))

		# Internal Use Variables - Do not modify without consultation
		self.refresh_rate = rospy.Rate(20)   # 20 [Hz] <--> 0.05 [sec]
		self.cmd = "S"
		self.prev_cmd = self.cmd
		self.xspeed = 0.0      # [m/s]
		self.rotspeed = 0.0    # [rad/s]
		self.ON_OFF = False    # True / False
		self.counter = 0

		# Publisher
		self.teleop_pub = rospy.Publisher(self.teleop_topic, Twist, queue_size=1)

		# Initialize the Wifi connection for Teleop USE
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.bind((self.ip_address, self.port))
		self.sock.listen(5)
		self.connection, self.address = self.sock.accept()

		# Run the main Loop
		self.receive_cmd()

	# Main Loop for receiving command from Wifi-TCP
	def receive_cmd(self):
		while not rospy.is_shutdown():
			incoming_msg, absent_msg, error_msg = select.select([self.sock], [], [], 0.05)
			for s in incoming_msg:
				if(s is self.sock):
					self.connection, self.address = self.sock.accept()
					self.cmd = self.connection.recv(8)
					self.connection.close()
			incoming_msg = []
			absent_msg = []
			error_msg = []
			print self.cmd
			drive_msg = Twist()
			if(self.cmd == "ON"):
				self.ON_OFF = True
			elif(self.cmd == "OFF"):
				self.ON_OFF = False
			if(self.ON_OFF == False):
				print "OFF"
				continue
			print "ON"
			if(self.cmd == "F"):                # Forward
				drive_msg.linear = Vector3(self.xspeed, 0, 0)
				drive_msg.angular = Vector3(0, 0, 0)
			elif(self.cmd == "G"):              # Forward Left
				drive_msg.linear = Vector3(self.xspeed, 0, 0)
				drive_msg.angular = Vector3(0, 0, self.rotspeed)
			elif(self.cmd == "I"):              # Forward Right
				drive_msg.linear = Vector3(self.xspeed, 0, 0)
				drive_msg.angular = Vector3(0, 0, -self.rotspeed)
			elif(self.cmd == "L"):              # Left
				drive_msg.linear = Vector3(0, 0, 0)
				drive_msg.angular = Vector3(0, 0, self.rotspeed)
			elif(self.cmd == "R"):              # Right
				drive_msg.linear = Vector3(0, 0, 0)
				drive_msg.angular = Vector3(0, 0, -self.rotspeed)
			elif(self.cmd == "B"):              # Backward
				drive_msg.linear = Vector3(-self.xspeed, 0, 0)
				drive_msg.angular = Vector3(0, 0, 0)
			elif(self.cmd == "H"):              # Back Left
				drive_msg.linear = Vector3(-self.xspeed, 0, 0)
				drive_msg.angular = Vector3(0, 0, self.rotspeed)
			elif(self.cmd == "J"):              # Back Right
				drive_msg.linear = Vector3(-self.xspeed, 0, 0)
				drive_msg.angular = Vector3(0, 0, -self.rotspeed)
			elif(self.cmd == "S"):              # Stop
				drive_msg.linear = Vector3(0, 0, 0)
				drive_msg.angular = Vector3(0, 0, 0)
			elif(self.cmd == "0"):              # 0.0 [m/s] [rad/s]
				self.xspeed = 0.0
				self.rotspeed = 0.0
			elif(self.cmd == "1"):              # 0.1 [m/s] [rad/s]
				self.xspeed = 0.1
				self.rotspeed = 0.3
			elif(self.cmd == "2"):              # 0.2 [m/s] [rad/s]
				self.xspeed = 0.2
				self.rotspeed = 0.3
			elif(self.cmd == "3"):              # 0.3 [m/s] [rad/s]
				self.xspeed = 0.3
				self.rotspeed = 0.5
			elif(self.cmd == "4"):              # 0.4 [m/s] [rad/s]
				self.xspeed = 0.4
				self.rotspeed = 0.5
			elif(self.cmd == "5"):              # 0.5 [m/s] [rad/s]
				self.xspeed = 0.5
				self.rotspeed = 0.5
			elif(self.cmd == "6"):              # 0.6 [m/s] [rad/s]
				self.xspeed = 0.6
				self.rotspeed = 0.75
			elif(self.cmd == "7"):              # 0.7 [m/s] [rad/s]
				self.xspeed = 0.7
				self.rotspeed = 0.75
			elif(self.cmd == "8"):              # 0.8 [m/s] [rad/s]
				self.xspeed = 0.8
				self.rotspeed = 0.75
			elif(self.cmd == "9"):              # 0.9 [m/s] [rad/s]
				self.xspeed = 0.9
				self.rotspeed = 1.0
			elif(self.cmd == "q"):              # 1.0 [m/s] [rad/s]
				self.xspeed = 1.0
				self.rotspeed = 1.0
			else:                               # Default
				drive_msg.linear = Vector3(0, 0, 0)
				drive_msg.angular = Vector3(0, 0, 0)
			self.teleop_pub.publish(drive_msg)
			self.cmd = None
			incoming_msg = []
			absent_msg = []
			error_msg = []
			self.connection.close()
			self.refresh_rate.sleep()

if __name__=="__main__":
	rospy.init_node("wifi_tcp_teleop")
	WifiTcpTeleop()
	rospy.spin()