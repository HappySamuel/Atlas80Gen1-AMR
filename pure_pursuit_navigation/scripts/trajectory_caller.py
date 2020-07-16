#!/usr/bin/env python

import rospy
import time, os

from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped
from utils import LineTrajectory


class TrajectoryCaller(object):
    """ Call a pre-defined trajectory from the file system and publishes it to a ROS topic.
    """
    def __init__(self):
        # Define Topics for Publishing and Subscribing Messages
        self.sub_topic = rospy.get_param("~sub_topic", "trajectory/new")
        self.pub_topic = rospy.get_param("~pub_topic", "trajectory/current")

        # Internal Use Variables - Do not modify without consultation
        self.counts = 0

        # Initialize and Load the trajectory
        self.trajectory = LineTrajectory("/called_trajectory")

        # Subscriber
        self.traj_sub = rospy.Subscriber(self.sub_topic, String, self.trajCB, queue_size=1)

        # Publisher
        self.traj_pub = rospy.Publisher(self.pub_topic, PolygonStamped, queue_size=1)

    # Callback Function for receiving path
    def trajCB(self, msg):
        path_id = msg.data
        if self.counts == 1:
            self.publish_trajectory(path_id)
            self.counts = 0
        self.counts = self.counts + 1

    # Republish the trajectory after being called
    def publish_trajectory(self, path_name):
        # clear the trajectory whenever going to load new path
        self.trajectory.clear()
        # load a new received path
        self.trajectory.load(path_name)
        print "Publishing trajectory to:", self.pub_topic
        self.traj_pub.publish(self.trajectory.toPolygon())


if __name__=="__main__":
    rospy.init_node("trajectory_caller")
    TrajectoryCaller()
    rospy.spin()

