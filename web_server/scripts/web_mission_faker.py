#!/usr/bin/env python

import rospy

from std_msgs.msg import String


class WebMissionFaker():
    def __init__(self):
        # Internal Use Variables
        self.fake_mission = "Shado_Home-0,Shado_A-1,Shado_1-2"
        self.pub_times = 0
        self.refresh_rate = rospy.Rate(1)

        # Publisher
        self.mission_pub = rospy.Publisher("/atlas80/mission", String, queue_size=1)

        # Main Looping
        self.main_loop()

    def main_loop(self):
        while not rospy.is_shutdown():
            print self.pub_times
            if(self.pub_times <= 3):
                print self.fake_mission
                self.mission_pub.publish(self.fake_mission)
            elif(self.pub_times > 3):
                # Shut down this node
                rospy.signal_shutdown("done")
            self.pub_times = self.pub_times + 1
            self.refresh_rate.sleep()

if __name__=="__main__":
    rospy.init_node("web_mission_faker")
    WebMissionFaker()
    rospy.spin()
