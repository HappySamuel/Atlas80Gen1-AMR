#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import String


class TrajectoryTester():
    def __init__(self):
        # Define Adjustable Parameters
        self.path_directory = rospy.get_param("~path_directory", "/home/atlas80-b/catkin_ws/src/pure_pursuit_navigation/trajectories/psa")

        # Path location
        self.path_1 = self.path_directory + "/psa_home_A.traj"
        self.path_2 = self.path_directory + "/psa_home_B.traj"
        self.path_3 = self.path_directory + "/psa_home_1.traj"
        self.path_4 = self.path_directory + "/psa_home_2.traj"
        self.path_5 = self.path_directory + "/psa_home_3.traj"
        self.path_6 = self.path_directory + "/psa_home_4.traj"
        self.path_7 = self.path_directory + "/psa_home_5.traj"
        self.path_8 = self.path_directory + "/psa_A_1.traj"
        self.path_9 = self.path_directory + "/psa_A_3.traj"
        self.path_10 = self.path_directory + "/psa_A_5.traj"
        self.path_11 = self.path_directory + "/psa_B_2.traj"
        self.path_12 = self.path_directory + "/psa_B_4.traj"
        self.path_13 = self.path_directory + "/psa_1_A.traj"
        self.path_14 = self.path_directory + "/psa_2_B.traj"
        self.path_15 = self.path_directory + "/psa_3_A.traj"
        self.path_16 = self.path_directory + "/psa_4_B.traj"
        self.path_17 = self.path_directory + "/psa_5_A.traj"
        self.path_18 = self.path_directory + "/psa_A_home.traj"
        self.path_19 = self.path_directory + "/psa_B_home.traj"
        self.path_20 = self.path_directory + "/psa_1_home.traj"
        self.path_21 = self.path_directory + "/psa_2_home.traj"
        self.path_22 = self.path_directory + "/psa_3_home.traj"
        self.path_23 = self.path_directory + "/psa_4_home.traj"
        self.path_24 = self.path_directory + "/psa_5_home.traj"
        self.path_25 = self.path_directory + "/psa_charging.traj"

        # Internal Use Variable - Do not modify without consultation
        self.refresh_rate = rospy.Rate(1)  # 1 Hz <--> 1 sec

        # Publisher
        self.path_pub = rospy.Publisher("/trajectory/new", String, queue_size=1)

        # Main Loop
        self.pub_loop()

    def pub_loop(self):
        selected_path = 0
        time.sleep(1)
        i = 1
        while not rospy.is_shutdown():   # do single check use (comment)
#            selected_path = i        # do fast check use (uncomment)
            selected_path = 7   # do single check use (comment)
            if(i > 24):
                i = 0
            self.chosen_one(selected_path)         
            self.path_pub.publish(self.chosen_path)
            i = i + 1
            self.refresh_rate.sleep()

    def chosen_one(self, selected_path):
        if selected_path == 1:
            self.chosen_path = self.path_1
        elif selected_path == 2:
            self.chosen_path = self.path_2
        elif selected_path == 3:
            self.chosen_path = self.path_3
        elif selected_path == 4:
            self.chosen_path = self.path_4
        elif selected_path == 5:
            self.chosen_path = self.path_5
        elif selected_path == 6:
            self.chosen_path = self.path_6
        elif selected_path == 7:
            self.chosen_path = self.path_7
        elif selected_path == 8:
            self.chosen_path = self.path_8
        elif selected_path == 9:
            self.chosen_path = self.path_9
        elif selected_path == 10:
            self.chosen_path = self.path_10
        elif selected_path == 11:
            self.chosen_path = self.path_11
        elif selected_path == 12:
            self.chosen_path = self.path_12
        elif selected_path == 13:
            self.chosen_path = self.path_13
        elif selected_path == 14:
            self.chosen_path = self.path_14
        elif selected_path == 15:
            self.chosen_path = self.path_15
        elif selected_path == 16:
            self.chosen_path = self.path_16
        elif selected_path == 17:
            self.chosen_path = self.path_17
        elif selected_path == 18:
            self.chosen_path = self.path_18
        elif selected_path == 19:
            self.chosen_path = self.path_19
        elif selected_path == 20:
            self.chosen_path = self.path_20
        elif selected_path == 21:
            self.chosen_path = self.path_21
        elif selected_path == 22:
            self.chosen_path = self.path_22
        elif selected_path == 23:
            self.chosen_path = self.path_23
        elif selected_path == 24:
            self.chosen_path = self.path_24
        elif selected_path == 25:
            self.chosen_path = self.path_25
        elif selected_path == 26:
            self.chosen_path = self.path_26
        elif selected_path == 27:
            self.chosen_path = self.path_27
        elif selected_path == 28:
            self.chosen_path = self.path_28
        elif selected_path == 29:
            self.chosen_path = self.path_29
        elif selected_path == 30:
            self.chosen_path = self.path_30
        elif selected_path == 31:
            self.chosen_path = self.path_31
        elif selected_path == 32:
            self.chosen_path = self.path_32
        elif selected_path == 33:
            self.chosen_path = self.path_33
        elif selected_path == 34:
            self.chosen_path = self.path_34
        elif selected_path == 35:
            self.chosen_path = self.path_35



if __name__=="__main__":
    rospy.init_node("trajectory_tester")
    TrajectoryTester()
    rospy.spin()
