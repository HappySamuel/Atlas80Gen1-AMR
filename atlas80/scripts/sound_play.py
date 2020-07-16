#!/usr/bin/env python

"""
   Author: Samuel Chieng Kien Ho
   Reference: Muzzammil's Code
"""

import rospy

from sensor_msgs.msg import Joy
from pygame import mixer


class SoundPlay():
    def __init__(self):
        # Define Topics for publishing or subscribing messages
        self.joy_topic = rospy.get_param("~joy_topic")

        # Define adjustable Parameter
        self.sound_folder = str(rospy.get_param("~sound_folder"))
        self.sound_file_1 = str(rospy.get_param("~sound_file_1"))
        self.sound_file_2 = str(rospy.get_param("~sound_file_2"))
        self.sound_file_3 = str(rospy.get_param("~sound_file_3"))
        self.sound_file_4 = str(rospy.get_param("~sound_file_4"))
        self.volume_1 = rospy.get_param("~volume_1")
        self.volume_2 = rospy.get_param("~volume_2")
        self.volume_3 = rospy.get_param("~volume_3")
        self.volume_4 = rospy.get_param("~volume_4")

        # Internal Use Variables - Do not modify without consultation
        self.checker_rate = rospy.Rate(5)
        self.on_1 = False
        self.on_2 = False
        self.on_3 = False
        self.on_4 = False
        self.counter_1 = 1
        self.counter_2 = 1
        self.counter_3 = 1
        self.counter_4 = 1
        self.counter_reset = False

        # Subscriber
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.joyCB, queue_size=1)

        mixer.init()
        self.sound_1 = mixer.Sound(self.sound_folder +"/"+ self.sound_file_1)
        self.sound_2 = mixer.Sound(self.sound_folder +"/"+ self.sound_file_2)
        self.sound_3 = mixer.Sound(self.sound_folder +"/"+ self.sound_file_3)
        self.sound_4 = mixer.Sound(self.sound_folder +"/"+ self.sound_file_4)

    # Joy Cmd Callback Function
    def joyCB(self, msg):
        self.on_1 = bool(msg.buttons[0])
        self.on_2 = bool(msg.buttons[1])
        self.on_3 = bool(msg.buttons[2])
        self.on_4 = bool(msg.buttons[3])
        self.counter_reset = bool(msg.buttons[6])
        if self.on_1 == True:
            self.playing_sound(1)
        elif self.on_2 == True:
            self.playing_sound(2)
        elif self.on_3 == True:
            self.playing_sound(3)
        elif self.on_4 == True:
            self.playing_sound(4)
        elif self.counter_reset == True:
            self.counter_1 = 1
            self.counter_2 = 1
            self.counter_3 = 1
            self.counter_4 = 1
            mixer.pause()
            print "Done --------- Counter-Reset"

    # Play the appointed sound with the appointed volume
    def playing_sound(self, num):
        if num == 1:
            if self.counter_1 == 1:
                self.sound_1.set_volume(self.volume_1)
                mixer.pause()
                self.sound_1.play()
                self.counter_1 = 0
                print "playing --------- " + str(self.sound_file_1)
        if num == 2:
            if self.counter_2 == 1:
                self.sound_2.set_volume(self.volume_2)
                self.sound_2.play()
                self.counter_2 = 0
                print "playing --------- " + str(self.sound_file_2)
        if num == 3:
            if self.counter_3 == 1:
                self.sound_3.set_volume(self.volume_3)
                mixer.pause()
                self.sound_3.play()
                self.counter_3 = 0
                print "playing --------- " + str(self.sound_file_3)
        if num == 4:
            if self.counter_4 == 1:
                self.sound_4.set_volume(self.volume_4)
                mixer.pause()
                self.sound_4.play()
                self.counter_4 = 0
                print "playing --------- " + str(self.sound_file_4)


if __name__=="__main__":
    rospy.init_node("sound_play")
    SoundPlay()
    rospy.spin()
