#!/usr/bin/env python

import rospy

from std_msgs.msg import String


class TrajectoryTester():
    def __init__(self):
        # Define Adjustable Parameters
        self.path_directory = rospy.get_param("~path_directory", "/home/pom/catkin_ws/src/pure_pursuit_navigation/trajectories/shado_testing")

        # Path location
        self.path_1 = self.path_directory + "/shado_home_A.traj"
        self.path_2 = self.path_directory + "/shado_home_B.traj"
        self.path_3 = self.path_directory + "/shado_home_C.traj"
        self.path_4 = self.path_directory + "/shado_home_1.traj"
        self.path_5 = self.path_directory + "/shado_home_2.traj"
        self.path_6 = self.path_directory + "/shado_home_3.traj"
        self.path_7 = self.path_directory + "/shado_A_1.traj"
        self.path_8 = self.path_directory + "/shado_A_2.traj"
        self.path_9 = self.path_directory + "/shado_A_3.traj"
        self.path_10 = self.path_directory + "/shado_B_1.traj"
        self.path_11 = self.path_directory + "/shado_B_2.traj"
        self.path_12 = self.path_directory + "/shado_B_3.traj"
        self.path_13 = self.path_directory + "/shado_C_1.traj"
        self.path_14 = self.path_directory + "/shado_C_2.traj"
        self.path_15 = self.path_directory + "/shado_C_3.traj"
        self.path_16 = self.path_directory + "/shado_1_A.traj"
        self.path_17 = self.path_directory + "/shado_1_B.traj"
        self.path_18 = self.path_directory + "/shado_1_C.traj"
        self.path_19 = self.path_directory + "/shado_2_A.traj"
        self.path_20 = self.path_directory + "/shado_2_B.traj"
        self.path_21 = self.path_directory + "/shado_2_C.traj"
        self.path_22 = self.path_directory + "/shado_3_A.traj"
        self.path_23 = self.path_directory + "/shado_3_B.traj"
        self.path_24 = self.path_directory + "/shado_3_C.traj"
        self.path_25 = self.path_directory + "/shado_A_home.traj"
        self.path_26 = self.path_directory + "/shado_B_home.traj"
        self.path_27 = self.path_directory + "/shado_C_home.traj"
        self.path_28 = self.path_directory + "/shado_1_home.traj"
        self.path_29 = self.path_directory + "/shado_2_home.traj"
        self.path_30 = self.path_directory + "/shado_3_home.traj"
        self.path_31 = self.path_directory + "/shado_charging_2.traj"
        self.path_32 = self.path_directory + "/shado_square.traj"
        self.path_33 = self.path_directory + "/shado_rectangular.traj"

        # Internal Use Variable - Do not modify without consultation
        self.refresh_rate = rospy.Rate(4)  # 1 Hz <--> 1 sec
        # Select path to be published - Path 1 or 2 or even more
#        self.selected_path = 0                # do fast check use (uncomment)
        self.selected_path = 31          # do single check use (comment)


        # Publisher
        self.path_pub = rospy.Publisher("/trajectory/new", String, queue_size=1)

        self.pub_loop()

    def chosen_one(self):
        if self.selected_path == 1:
            self.chosen_path = self.path_1
        elif self.selected_path == 2:
            self.chosen_path = self.path_2
        elif self.selected_path == 3:
            self.chosen_path = self.path_3
        elif self.selected_path == 4:
            self.chosen_path = self.path_4
        elif self.selected_path == 5:
            self.chosen_path = self.path_5
        elif self.selected_path == 6:
            self.chosen_path = self.path_6
        elif self.selected_path == 7:
            self.chosen_path = self.path_7
        elif self.selected_path == 8:
            self.chosen_path = self.path_8
        elif self.selected_path == 9:
            self.chosen_path = self.path_9
        elif self.selected_path == 10:
            self.chosen_path = self.path_10
        elif self.selected_path == 11:
            self.chosen_path = self.path_11
        elif self.selected_path == 12:
            self.chosen_path = self.path_12
        elif self.selected_path == 13:
            self.chosen_path = self.path_13
        elif self.selected_path == 14:
            self.chosen_path = self.path_14
        elif self.selected_path == 15:
            self.chosen_path = self.path_15
        elif self.selected_path == 16:
            self.chosen_path = self.path_16
        elif self.selected_path == 17:
            self.chosen_path = self.path_17
        elif self.selected_path == 18:
            self.chosen_path = self.path_18
        elif self.selected_path == 19:
            self.chosen_path = self.path_19
        elif self.selected_path == 20:
            self.chosen_path = self.path_20
        elif self.selected_path == 21:
            self.chosen_path = self.path_21
        elif self.selected_path == 22:
            self.chosen_path = self.path_22
        elif self.selected_path == 23:
            self.chosen_path = self.path_23
        elif self.selected_path == 24:
            self.chosen_path = self.path_24
        elif self.selected_path == 25:
            self.chosen_path = self.path_25
        elif self.selected_path == 26:
            self.chosen_path = self.path_26
        elif self.selected_path == 27:
            self.chosen_path = self.path_27
        elif self.selected_path == 28:
            self.chosen_path = self.path_28
        elif self.selected_path == 29:
            self.chosen_path = self.path_29
        elif self.selected_path == 30:
            self.chosen_path = self.path_30
        elif self.selected_path == 31:
            self.chosen_path = self.path_31
        elif self.selected_path == 32:
            self.chosen_path = self.path_32
        elif self.selected_path == 33:
            self.chosen_path = self.path_33

    def pub_loop(self):
        i = 1
        while not rospy.is_shutdown():   # do single check use (comment)
#            self.selected_path = i        # do fast check use (uncomment)
            self.selected_path = 31   # do single check use (comment)
            if(i > 31):
                i = 0
            self.chosen_one()         
            self.path_pub.publish(self.chosen_path)
            i = i + 1
            self.refresh_rate.sleep()


if __name__=="__main__":
    rospy.init_node("trajectory_tester")
    TrajectoryTester()
    rospy.spin()
