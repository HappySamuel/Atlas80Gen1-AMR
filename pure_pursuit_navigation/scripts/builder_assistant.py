#!/usr/bin/env python
"""
   Author :  Samuel Chieng Kien Ho
   Function :  Assist the Offline Trajectory Builder with display the points previously recorded.
"""

import rospy
import json

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class BuilderAssistant():
    def __init__(self):
        # Define Adjustable Parameters
        self.points_file = rospy.get_param("~points_file")

        # Internal Use Variables - Do not modify without consultation
        self.display_pts = []
        self.refresh_rate = rospy.Rate(10)    # 10 [Hz] <---> 0.1 [sec]

        # Publisher
        self.viz_points_pub = rospy.Publisher("/viz/builder_assitant/points", MarkerArray, queue_size=1)

        # Main Loop
        self.viz_points()

    def load_file(self, path_to_file):
        print "Loading trajectory: ", path_to_file
        with open(path_to_file) as json_file:
            json_data = json.load(json_file)
            for p in json_data["points"]:
                temp_pt = Point()
                temp_pt.x = p["x"]
                temp_pt.y = p["y"]
                temp_pt.z = 0.0
                self.display_pts.append(temp_pt)

    def viz_points(self):
        self.load_file(self.points_file)
        markers = []
        for i in xrange(len(self.display_pts)):
            markers.append(self.point_marker(self.display_pts[i], i))
        points_array = MarkerArray(markers=markers)
        while not rospy.is_shutdown():
            self.viz_points_pub.publish(points_array)
            self.refresh_rate.sleep()

    def point_marker(self, point, index):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "builder_assitant"
        marker.id = index
        marker.type = 3   # Cylinder
        marker.action = 0    # Add / Modify
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        marker.color = ColorRGBA(0, 1, 0, 1)    # Green
        marker.pose.position = point
        return marker



if __name__=="__main__":
    rospy.init_node("builder_assistant")
    BuilderAssistant()
    rospy.spin()
